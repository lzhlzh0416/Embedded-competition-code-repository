#include "can_bridge_cpp/socketcan.h"
#include "can_msgs/msg/frame.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include <fstream>
#include <sstream>
#include <string>
#include <fcntl.h>   // for open, O_WRONLY, O_TRUNC
#include <unistd.h>  // for close
#include <sys/stat.h>



class CanBridge : public rclcpp::Node
{
public:
    CanBridge() : Node("can_bridge")
    {
        // 创建发布者和订阅者
        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
        sub_ = this->create_subscription<can_msgs::msg::Frame>("to_can_bus", 10, std::bind(&CanBridge::send_can_frame, this, std::placeholders::_1));
        cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>("cmd_vel", 10, std::bind(&CanBridge::cmd_vel_callback, this, std::placeholders::_1));

        // 打开 CAN 设备
        if (!can_.open("can1", std::bind(&CanBridge::can_receive_callback, this, std::placeholders::_1), 10))
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to open CAN device.");
            return;
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "CAN device opened.");
        }

        // ✅ 定时读取 AI_result.txt 并发送 CAN 指令
       timer_ = this->create_wall_timer(
            std::chrono::milliseconds(500),
            std::bind(&CanBridge::check_and_send_ai_result, this)
        );

    }


private:
    void can_receive_callback(const struct can_frame &frame)
    {
        static bool has_101 = false;
        static bool has_102 = false;

        if (frame.can_id == 0x101)
        {
            odom.angle = (int16_t)(frame.data[0] << 8 | frame.data[1]) / 1e4;
            odom.x = (int16_t)(frame.data[2] << 8 | frame.data[3]) / 1e4;
            odom.y = (int16_t)(frame.data[4] << 8 | frame.data[5]) / 1e4;
            odom.linear_speed = (int16_t)(frame.data[6] << 8 | frame.data[7]) / 1e4;
            has_101 = true;
        }
        else if (frame.can_id == 0x102)
        {
            odom.angular_speed = (int16_t)(frame.data[0] << 8 | frame.data[1]) / 1e4;
            has_102 = true;
        }

        if (has_101 && has_102)
        {
            // 两帧数据都到齐，准备发布 odometry
            nav_msgs::msg::Odometry odom_msg;
            odom_msg.header.stamp = this->get_clock()->now();
            odom_msg.header.frame_id = "odom";
            odom_msg.pose.pose.position.x = odom.x;
            odom_msg.pose.pose.position.y = odom.y;
            odom_msg.pose.pose.orientation.w = cos(odom.angle * 0.5);
            odom_msg.pose.pose.orientation.x = 0;
            odom_msg.pose.pose.orientation.y = 0;
            odom_msg.pose.pose.orientation.z = sin(odom.angle * 0.5);
            odom_msg.child_frame_id = "base_footprint";
            odom_msg.twist.twist.linear.x = odom.linear_speed;
            odom_msg.twist.twist.angular.z = odom.angular_speed;

            odom_pub_->publish(odom_msg);

            // 发布后清除标志
            has_101 = false;
            has_102 = false;
        }
    }

void send_can_frame(const can_msgs::msg::Frame::SharedPtr msg)
{
    struct can_frame frame;
    frame.can_id = msg->id;
    frame.can_dlc = msg->dlc;
    memcpy(frame.data, msg->data.data(), msg->dlc);
    
    RCLCPP_INFO(this->get_logger(), "Attempting to send CAN frame: ID=0x%03X, DLC=%d", 
                frame.can_id, frame.can_dlc);
                
    // 直接调用write方法，不尝试获取返回值
    can_.write(&frame);
    
    // 添加日志确认发送
    RCLCPP_INFO(this->get_logger(), "CAN frame sent to device");
}

    void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        float linear = msg->linear.x;   // m/s
        float angular = msg->angular.z; // rad/s

        int16_t linear_scaled = static_cast<int16_t>(linear * 1e4);
        int16_t angular_scaled = static_cast<int16_t>(angular * 1e4);

        struct can_frame frame;
        frame.can_id = 0x200;
        frame.can_dlc = 4;
        frame.data[0] = (linear_scaled >> 8) & 0xFF;
        frame.data[1] = linear_scaled & 0xFF;
        frame.data[2] = (angular_scaled >> 8) & 0xFF;
        frame.data[3] = angular_scaled & 0xFF;

         can_.write(&frame);
    }

    void clear_file(const std::string &path)
    {
        int fd = open(path.c_str(), O_WRONLY | O_TRUNC);
        if (fd == -1)
        {
            RCLCPP_ERROR(this->get_logger(), "打开文件失败: %s", path.c_str());
            return;
        }
        close(fd);
        RCLCPP_INFO(this->get_logger(), "文件已被截断清空: %s", path.c_str());
    }

   void check_and_send_ai_result()
    {
        struct stat st;
        if (::stat("/app/mine/voice/AI_result.txt", &st) == 0)
        {
            RCLCPP_INFO(this->get_logger(), "文件大小: %ld bytes", st.st_size);
        }

        std::ifstream infile("/app/mine/voice/AI_result.txt");
        if (!infile.is_open())
        {
            RCLCPP_ERROR(this->get_logger(), "无法打开 AI_result.txt 文件");
            return;
        }

        std::string line;
        bool has_data = false;

        while (std::getline(infile, line))
        {
            std::stringstream ss(line);
            float value;
            ss >> value;
            if (ss.fail())
                continue;  // 跳过无效行

            has_data = true;

            int16_t scaled = static_cast<int16_t>(value * 1e4);
            struct can_frame frame;
            frame.can_id = 0x200;
            frame.can_dlc = 2;
            frame.data[0] = (scaled >> 8) & 0xFF;
            frame.data[1] = scaled & 0xFF;

            can_.write(&frame);

            RCLCPP_INFO(this->get_logger(),
                "已发送 AI 指令 → CAN ID: 0x%03X Data: [%02X %02X] (%.2f)",
                frame.can_id, frame.data[0], frame.data[1], value);

            // 可适当延时，避免 CAN 总线缓冲区满
            rclcpp::sleep_for(std::chrono::milliseconds(100));
        }

        infile.close();

        if (has_data)
        {
            // 发送完所有数据后清空文件
            std::ofstream outfile("/app/mine/voice/AI_result.txt", std::ios::trunc);
            if (!outfile.is_open())
            {
                RCLCPP_ERROR(this->get_logger(), "清空 AI_result.txt 文件失败");
            }
            else
            {
                RCLCPP_INFO(this->get_logger(), "清空 AI_result.txt 文件成功");
                outfile.close();
            }

            if (::stat("/app/mine/voice/AI_result.txt", &st) == 0)
            {
                RCLCPP_INFO(this->get_logger(), "清空后文件大小: %ld bytes", st.st_size);
            }
        }
    }





    rclcpp::TimerBase::SharedPtr timer_;
    struct odom_t
    {
        float x;
        float y;
        float angle;
        float linear_speed;
        float angular_speed;
    };
    odom_t odom;

    can::SocketCAN can_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Subscription<can_msgs::msg::Frame>::SharedPtr sub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CanBridge>());
    rclcpp::shutdown();
    return 0;
}

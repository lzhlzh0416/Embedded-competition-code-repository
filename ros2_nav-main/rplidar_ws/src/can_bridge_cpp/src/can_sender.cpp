#include "can_bridge_cpp/socketcan.h"
#include "can_msgs/msg/frame.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"

class CanBridge : public rclcpp::Node
{
public:
    CanBridge() : Node("can_bridge")
    {
        // 创建发布者和订阅者
        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
        sub_ = this->create_subscription<can_msgs::msg::Frame>("to_can_bus", 10, std::bind(&CanBridge::send_can_frame, this, std::placeholders::_1));

        // sub "cmd_vel",transform to can
        cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>("cmd_vel", 10, std::bind(&CanBridge::cmd_vel_callback, this, std::placeholders::_1));

        // 打开 CAN 设备并传入接收回调
        if (!can_.open("can0", std::bind(&CanBridge::can_receive_callback, this, std::placeholders::_1), 10))
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to open CAN device.");
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "CAN device opened.");
        }
    }

    ~CanBridge() { can_.close(); }

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
        //     struct can_frame frame;
        //     frame.can_id = msg->id;
        //     frame.can_dlc = msg->dlc;
        //     memcpy(frame.data, msg->data.data(), msg->dlc);
        //     can_.write(&frame);
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

        // 添加发送节流逻辑
        static rclcpp::Time last_send_time = this->get_clock()->now();
        auto now = this->get_clock()->now();
        if ((now - last_send_time).seconds() > 0.02) // 50Hz 发送率
        {
            can_.write(&frame);
            last_send_time = now;
        }
    }

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

#include "can_task.hpp"
#include "chassis_L_task.hpp"
#include "math_lib.hpp"

extern chassis_t chassis_move;
extern float ins_gyro[3];

sp::CAN can1(&hcan1);
sp::CAN can2(&hcan2);
odom_t odom;
void update_odom(chassis_t *chassis, odom_t *odom, float *ins_gyro, float dt);
void send_odom_to_ros_part1(odom_t *odom);
void send_odom_to_ros_part2(odom_t *odom);
void get_cmd_vel(uint8_t *data);
uint8_t odom_send_count = 0;
float v = 0;
float w = 0;
extern "C" void can_task()
{
    can1.config();
    can1.start();
    can2.config();
    can2.start();

    while (true)
    {
        update_odom(&chassis_move, &odom, ins_gyro, 0.001);
        if (odom_send_count > 10)
        {
            send_odom_to_ros_part1(&odom);
            send_odom_to_ros_part2(&odom);
            odom_send_count = 0;
        }
        odom_send_count++;
        osDelay(1);
    }
}

extern "C" void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    auto stamp_ms = osKernelSysTick();
    if ((hcan == &hcan1))
    {
        can1.recv();

        if (can1.rx_id == joint_motor_1.rx_id)
            joint_motor_1.read(can1.rx_data, stamp_ms);
        if (can1.rx_id == joint_motor_2.rx_id)
            joint_motor_2.read(can1.rx_data, stamp_ms);
        if (can1.rx_id == wheel_motor_left.rx_id)
            wheel_motor_left.read(can1.rx_data, stamp_ms);
        if (can1.rx_id == 0x200)
            get_cmd_vel(can1.rx_data);
    }

    if (hcan == &hcan2)
    {
        can2.recv();
        if (can2.rx_id == joint_motor_3.rx_id)
            joint_motor_3.read(can2.rx_data, stamp_ms);
        if (can2.rx_id == joint_motor_4.rx_id)
            joint_motor_4.read(can2.rx_data, stamp_ms);
        if (can2.rx_id == wheel_motor_right.rx_id)
            wheel_motor_right.read(can2.rx_data, stamp_ms);
    }
}

void update_odom(chassis_t *chassis, odom_t *odom, float *ins_gyro, float dt)
{
    odom->linear_speed = chassis->v_filter;
    odom->angular_speed = ins_gyro[2];
    odom->angle = radFormat(chassis->total_yaw);
    float delta_distance = odom->linear_speed * dt;
    odom->x += delta_distance * cos(odom->angle);
    odom->y += delta_distance * sin(odom->angle);
}
void send_odom_to_ros_part1(odom_t *odom)
{
    can1.tx_data[0] = (int16_t)(odom->angle * 1e4f) >> 8;
    can1.tx_data[1] = (int16_t)(odom->angle * 1e4f);
    can1.tx_data[2] = (int16_t)(odom->x * 1e4f) >> 8;
    can1.tx_data[3] = (int16_t)(odom->x * 1e4f);
    can1.tx_data[4] = (int16_t)(odom->y * 1e4f) >> 8;
    can1.tx_data[5] = (int16_t)(odom->y * 1e4f);
    can1.tx_data[6] = (int16_t)(odom->linear_speed * 1e4f) >> 8;
    can1.tx_data[7] = (int16_t)(odom->linear_speed * 1e4f);
    can1.send(0x101);
}

void send_odom_to_ros_part2(odom_t *odom)
{

    can1.tx_data[0] = (int16_t)(odom->angular_speed * 1e4f) >> 8;
    can1.tx_data[1] = (int16_t)(odom->angular_speed * 1e4f);
    can1.tx_data[2] = 0;
    can1.tx_data[3] = 0;
    can1.tx_data[4] = 0;
    can1.tx_data[5] = 0;
    can1.tx_data[6] = 0;
    can1.tx_data[7] = 0;
    can1.send(0x102);
}

void get_cmd_vel(uint8_t *data)
{
    int16_t vx = (int16_t)((data[0] << 8) | data[1]);
    v = vx / 10000.0f;

    // 解析 vtheta
    int16_t vw = (int16_t)((data[2] << 8) | data[3]);
    w = vw / 10000.0f;
}
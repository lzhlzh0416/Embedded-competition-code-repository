#ifndef CHASSIS_L_TASK_HPP
#define CHASSIS_L_TASK_HPP

#include "VMC_calc.hpp"
#include "motor/dm_motor/dm_motor.hpp"

// 定义达妙电机pvt最大值和最小值
//  p: position,  rad
//  v: velocity, rad/s
//  t: torque, N·m
constexpr float P_MIN = -3.141593f;
constexpr float P_MAX = 3.141593f;
constexpr float V_MIN = -30.0f;
constexpr float V_MAX = 30.0f;
constexpr float T_MIN = -10.0f;
constexpr float T_MAX = 10.0f;

constexpr float T_CHASSIS_L = 3e-3; // 底盘任务执行周期

// 腿长控制pid
constexpr float LEG_PID_KP = 350.0f;
constexpr float LEG_PID_KI = 0.0f;
constexpr float LEG_PID_KD = 20.0f;
constexpr float LEG_PID_MAX_OUT = 90.0f;
constexpr float LEG_PID_MAX_IOUT = 0.0f;
constexpr float Mg = 18.0f;

constexpr float RAD_45 = (3.1415926f / 4.0f);
constexpr float RAD_90 = (3.1415926f / 2.0f);

const float WHEEL_RADIUS = 0.05f; // 轮子半径

typedef struct
{
    float wheel_left_torque;
    float wheel_right_torque;
    float joint_1_torque;
    float joint_2_torque;
    float joint_3_torque;
    float joint_4_torque;

    float v_set; // 期望速度，单位是m/s
    float target_v;
    float x_set;    // 期望位置，单位是m
    float turn_set; // 期望yaw轴弧度
    float target_turn;
    float leg_set; // 期望腿长，单位是m
    float leg_lx_set;
    float target_leg_lx_set;
    float leg_left_set;
    float leg_right_set;
    float last_leg_set;

    float roll_force;
    float roll_target;
    float now_roll_set;
    float last_yaw;
    int yaw_round_count; // yaw 多圈编码

    float accel;
    float v_filter; // 滤波后的车体速度，单位是m/s
    float x_filter; // 滤波后的车体位置，单位是m

    float myPitchR;
    float myPitchGyroR;
    float myPitchL;
    float myPitchGyroL;
    float roll;
    float total_yaw;
    float theta_err; // 两腿夹角误差

    float turn_T; // yaw轴补偿
    float leg_tp; // 防劈叉补偿

    uint8_t start_flag; // 启动标志

    uint8_t recover_flag; // 一种情况下的倒地自起标志

    uint8_t left_ground_flag;
    uint8_t right_ground_flag;

    uint32_t count_key;
    uint8_t jump_flag;
    float jump_leg;
    uint32_t jump_time_r;
    uint32_t jump_time_l;
    uint8_t jump_status_r;
    uint8_t jump_status_l;

    float vx; // 导航vx
    float vw; // 导航vw

} chassis_t;

extern vmc_leg_t vmc_left;
extern float LQR_K[12];
extern float Poly_Coefficient[12][4];
extern chassis_t chassis_move;

#endif
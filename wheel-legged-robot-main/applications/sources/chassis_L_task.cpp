#include "chassis_L_task.hpp"

#include "can_task.hpp"
#include "chassis_R_task.hpp"
#include "cmsis_os.h"
#include "imu_task.hpp"
#include "io/can/can.hpp"
#include "io/dbus/dbus.hpp"
#include "math_lib.hpp"
#include "motor/dm_motor/dm_motor.hpp"
#include "tools/pid/pid.hpp"
#include <cstdint>

chassis_t chassis_move;
vmc_leg_t vmc_left;

extern sp::DBus remote;

float LQR_K[12] = {-1.7325, -0.1273, -0.0, -0.4183, 0.8738, 0.0946, 1.3237, 0.1247, 0.0, 0.0, 9.6184, 0.3574};

float Poly_Coefficient[12][4] = {{-108.9569f, 75.2649f, -25.7233f, -1.3004f}, {-1.2397f, 0.8637f, -2.4329f, -0.0180f},     {-16.7445f, 9.3905f, -1.7967f, -0.5868f},
                                 {-4.6125f, 1.8123f, -0.3693f, -0.9924f},     {-390.5272f, 236.9751f, -51.8901f, 4.6783f}, {-16.5347f, 10.3695f, -2.4112f, 0.2704f},
                                 {-570.4547f, 348.4034f, -77.2021f, 7.3485f}, {-25.6506f, 15.8712f, -3.6479f, 0.4239f},    {-228.2999f, 137.6474f, -29.7160f, 2.5297f},
                                 {-362.9416f, 216.3602f, -45.9015f, 3.8010f}, {687.9892f, -388.1504f, 75.0354f, 17.4521f}, {36.7476f, -21.1837f, 4.2490f, 0.5666f}};
sp::DM_Motor joint_motor_1(0x04, 0x08, P_MAX, V_MAX, T_MAX); // 左前关节电机
sp::DM_Motor joint_motor_2(0x03, 0x06, P_MAX, V_MAX, T_MAX); // 左后关节电机

sp::DM_Motor wheel_motor_left(0x01, 0x00, 12.5f, 45.0f, 10.0f); // 左轮毂电机

sp::PID leg_left_pid(T_CHASSIS_L, LEG_PID_KP, LEG_PID_KI, LEG_PID_KD, LEG_PID_MAX_OUT, LEG_PID_MAX_IOUT); // 左腿长pid

void chassis_set_input(chassis_t *chassis);
void jump_loop_l(chassis_t *chassis, vmc_leg_t *vmcl);
void chassis_L_init(vmc_leg_t *vmc);
void chassisL_feedback_update(chassis_t *chassis, vmc_leg_t *vmc, sp::Mahony *imu); //, INS_t *ins
void left_ground_detect(chassis_t *chassis, vmc_leg_t *vmcl, sp::Mahony *imu, float *LQR_K);
void chassisL_control_loop(chassis_t *chassis, vmc_leg_t *vmcl, sp::Mahony *imu, float *LQR_K);

uint16_t jump_count; // 跳跃冷却时间

extern "C" void chassis_L_task(void const *argument)
{
    osDelay(1000);             // 等待电机上电 再使能电机
    chassis_L_init(&vmc_left); // 初始化腿部

    while (1)
    {
        chassis_set_input(&chassis_move);                         // 设置控制输入量
        chassisL_feedback_update(&chassis_move, &vmc_left, &imu); // 更新数据

        chassisL_control_loop(&chassis_move, &vmc_left, &imu, LQR_K); // 控制计算

        if (chassis_move.start_flag == 1)
        {
            joint_motor_1.cmd(vmc_left.torque_set[1]);
            joint_motor_2.cmd(vmc_left.torque_set[0]);
            wheel_motor_left.cmd(chassis_move.wheel_left_torque);
        }
        if (chassis_move.start_flag == 0)
        {
            joint_motor_1.cmd(0.0f);
            joint_motor_2.cmd(0.0f);
            wheel_motor_left.cmd(0.0f);
        }

        joint_motor_1.write(can1.tx_data);
        can1.send(joint_motor_1.tx_id);
        osDelay(1);
        joint_motor_2.write(can1.tx_data);
        can1.send(joint_motor_2.tx_id);
        osDelay(1);
        wheel_motor_left.write(can1.tx_data);
        can1.send(wheel_motor_left.tx_id);
        osDelay(1);
    }
}

void chassis_L_init(vmc_leg_t *vmc)
{
    VMC_init(vmc); // 给杆长赋值

    // 使能达妙电机
    for (int j = 0; j < 10; j++)
    {
        joint_motor_1.write_enable(can1.tx_data);
        can1.send(joint_motor_1.tx_id);
        osDelay(1);
    }
    for (int j = 0; j < 10; j++)
    {
        joint_motor_2.write_enable(can1.tx_data);
        can1.send(joint_motor_2.tx_id);
        osDelay(1);
    }
    for (int j = 0; j < 10; j++)
    {
        wheel_motor_left.write_enable(can1.tx_data);
        can1.send(wheel_motor_left.tx_id);
        osDelay(1);
    }
}

// 设置输入量
void chassis_set_input(chassis_t *chassis)
{
    // 左拨杆拨到中间
    if (remote.sw_l == sp::DBusSwitchMode::MID)
    {
        chassis->start_flag = 1;
        // 前倾后倾 超过45°但不超过90°
        if (chassis->recover_flag == 0 && ((chassis->myPitchL < -RAD_45 && chassis->myPitchL > -RAD_90) || (chassis->myPitchL > RAD_45 && chassis->myPitchL < RAD_90)))
        {
            chassis->recover_flag = 1; // 需要自起
        }
    }
    else
    {
        chassis->start_flag = 0;
        chassis->recover_flag = 0;
    }

    if (chassis->start_flag == 1)
    {
        // if(chassis->recover_flag==1)
        // {
        //     chassis->v_set = 0.0f;                  // 清零
        //     chassis->x_set = chassis->x_filter;     // 保存
        //     chassis->turn_set = chassis->total_yaw; // 保存
        //     chassis->leg_set = 0.1f;                // 原始腿长
        // }
        chassis->v_set = remote.ch_lv * 1.5f; // 左摇杆前后
        chassis->x_set = chassis->x_set + chassis->v_set * T_CHASSIS_L;
        chassis->turn_set += remote.ch_lh * (-0.01f); // 左摇杆左右

        // 腿长变化
        if (fabs(remote.ch_rh) > 0.01)
            chassis->leg_set += remote.ch_rh * 0.0005f; // 右摇杆左右
        limitFloat(chassis->leg_set, 0.065f, 0.18f);
        // chassis->roll_target = ((float)(data->lx - 127)) * (0.0025f);
        // slope_following(&chassis->roll_target, &chassis->roll_set, 0.0075f);

        chassis->leg_left_set = chassis->leg_set;
        chassis->leg_right_set = chassis->leg_set;

        if (remote.sw_r == sp::DBusSwitchMode::MID && jump_count == 0)
        {
            chassis->jump_flag = 1;
            jump_count = 500; // 跳跃冷却 1.5s
        }
        if (jump_count > 0)
            jump_count--;

        if (fabsf(chassis->last_leg_set - chassis->leg_set) > 0.0001f)
        {
            // 遥控器控制腿长在变化
            // 为1标志着腿长在主动伸缩(不包括自适应伸缩)，根据这个标志可以不进行离地检测，因为当腿长在主动伸缩时，离地检测会误判端为离地了
            vmc_right.leg_control_flag = 1;
            vmc_left.leg_control_flag = 1;
        }
        chassis->last_leg_set = chassis->leg_set;
    }

    if (chassis->start_flag == 0)
    {                                           // 关闭
        chassis->v_set = 0.0f;                  // 清零
        chassis->x_set = chassis->x_filter;     // 保存
        chassis->turn_set = chassis->total_yaw; // 保存
        chassis->leg_set = 0.1f;                // 原始腿长
    }
}

// 姿态数据更新
void chassisL_feedback_update(chassis_t *chassis, vmc_leg_t *vmc, sp::Mahony *imu)
{
    vmc->phi1 = pi / 2.0f + joint_motor_2.angle;
    vmc->phi4 = pi / 2.0f + joint_motor_1.angle;

    chassis->myPitchL = imu->pitch;
    chassis->myPitchGyroL = ins_gyro[1];
}

void chassisL_control_loop(chassis_t *chassis, vmc_leg_t *vmcl, sp::Mahony *imu, float *LQR_K)
{
    VMC_calc_left(vmcl, imu, T_CHASSIS_L); // 计算theta和d_theta给lqr用，同时也计算左腿长L0

    for (int i = 0; i < 12; i++)
    {
        LQR_K[i] = LQR_K_calc(&Poly_Coefficient[i][0], vmcl->L0); // 根据腿长和拟合公式，算出不同腿长下的lqr参数
    }

    // if (fabsf(chassis->v_filter) > 0.08)
    // {
    //     chassis->x_set = chassis->x_filter;
    // }

    chassis->wheel_left_torque = (LQR_K[0] * (vmcl->theta - 0.0f) + LQR_K[1] * (vmcl->d_theta - 0.0f) + LQR_K[2] * (chassis->x_set - chassis->x_filter) +
                                  LQR_K[3] * (chassis->v_set - chassis->v_filter) + LQR_K[4] * (chassis->myPitchL - 0.0f) + LQR_K[5] * (chassis->myPitchGyroL - 0.0f));

    chassis->wheel_left_torque -= chassis->turn_T; // 轮毂电机输出力矩(平动扭矩-转向扭矩)
    limitFloat(chassis->wheel_left_torque, -1.0f, 1.0f);

    // 左边髋关节输出力矩
    vmcl->Tp = (LQR_K[6] * (vmcl->theta - 0.0f) + LQR_K[7] * (vmcl->d_theta - 0.0f) + LQR_K[8] * (chassis->x_set - chassis->x_filter) + LQR_K[9] * (chassis->v_set - chassis->v_filter) +
                LQR_K[10] * (chassis->myPitchL - 0.0f) + LQR_K[11] * (chassis->myPitchGyroL - 0.0f));

    vmcl->Tp += chassis->leg_tp; // 髋关节输出力矩(平动力矩+放劈叉力矩)

    jump_loop_l(chassis, vmcl);

    left_ground_detect(chassis, vmcl, imu, LQR_K);

    limitFloat(vmcl->F0, -150.0f, 150.0f); // 支持力限幅

    VMC_calc_2(vmcl); // 计算期望的关节输出力矩

    // 关节电机扭矩限幅
    limitFloat(vmcl->torque_set[1], -3.0f, 3.0f);
    limitFloat(vmcl->torque_set[0], -3.0f, 3.0f);
}

// 离地检测
void left_ground_detect(chassis_t *chassis, vmc_leg_t *vmcl, sp::Mahony *imu, float *LQR_K)
{
    chassis->left_ground_flag = ground_detectionL(vmcl, imu); // 左腿离地检测

    if (chassis->recover_flag == 0)
    { // 倒地自起不需要检测是否离地
        if (chassis->left_ground_flag == 1 && chassis->right_ground_flag == 1 && vmcl->leg_control_flag == 0)
        { // 当两腿同时离地并且遥控器没有在控制腿的伸缩时，才认为离地
            chassis->wheel_left_torque = 0.0f;
            vmcl->Tp = LQR_K[6] * (vmcl->theta - 0.0f) + LQR_K[7] * (vmcl->d_theta - 0.0f);

            chassis->x_filter = 0.0f; // 对位移清零
            chassis->x_set = chassis->x_filter;
            chassis->turn_set = chassis->total_yaw;
            vmcl->Tp = vmcl->Tp + chassis->leg_tp;
        }
        else
        {                               // 没有离地
            vmcl->leg_control_flag = 0; // 置为0
        }
    }
    else if (chassis->recover_flag == 1)
    {
        vmcl->Tp = 0.0f;
    }
}
// 跳跃四个阶段 （0-收腿 1-蹬腿 2-收腿缓冲 3-正常状态 ）
void jump_loop_l(chassis_t *chassis, vmc_leg_t *vmcl)
{
    if (chassis->jump_flag == 1)
    {
        if (chassis->jump_status_l == 0)
        {
            leg_left_pid.calc(0.07f, vmcl->L0);
            vmcl->F0 = Mg / fast_cos(vmcl->theta) + leg_left_pid.out; // 前馈+pd
            if (vmcl->L0 < 0.1f)
            {
                chassis->jump_time_l++;
            }
        }
        else if (chassis->jump_status_l == 1)
        {
            leg_left_pid.calc(0.5f, vmcl->L0);
            vmcl->F0 = Mg / fast_cos(vmcl->theta) + leg_left_pid.out; // 前馈+pd
            if (vmcl->L0 > 0.16f)
            {
                chassis->jump_time_l++;
            }
        }
        else if (chassis->jump_status_l == 2)
        {
            leg_left_pid.calc(0.0f, vmcl->L0);                           // chassis->leg_right_set
            vmcl->F0 = 10.0f / fast_cos(vmcl->theta) + leg_left_pid.out; // 前馈+pd Mg
            if (vmcl->L0 < (chassis->leg_right_set + 0.01f))
            {
                chassis->jump_time_l++;
            }
        }
        else
        {
            leg_left_pid.calc(chassis->leg_left_set, vmcl->L0);
            vmcl->F0 = Mg / fast_cos(vmcl->theta) + leg_left_pid.out; // 前馈+pd
        }
    }
    else
    {
        leg_left_pid.calc(chassis->leg_left_set, vmcl->L0);
        vmcl->F0 = Mg / fast_cos(vmcl->theta) + leg_left_pid.out + chassis->roll_force; // 前馈+pd
    }
}

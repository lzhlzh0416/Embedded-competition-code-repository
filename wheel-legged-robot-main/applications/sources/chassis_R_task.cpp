#include "chassis_R_task.hpp"
#include "VMC_calc.hpp"
#include "can_task.hpp"
#include "chassis_L_task.hpp"
#include "cmsis_os.h"
#include "imu_task.hpp"
#include "io/can/can.hpp"
#include "math_lib.hpp"
#include "motor/dm_motor/dm_motor.hpp"
#include "tools/pid/pid.hpp"

vmc_leg_t vmc_right;

sp::DM_Motor joint_motor_3(0x04, 0x08, P_MAX, V_MAX, T_MAX);
sp::DM_Motor joint_motor_4(0x03, 0x06, P_MAX, V_MAX, T_MAX);

sp::DM_Motor wheel_motor_right(0x01, 0x00, 12.5f, 45.0f, 10.0f);

sp::PID leg_right_pid(T_CHASSIS_R, LEG_PID_KP, LEG_PID_KI, LEG_PID_KD, LEG_PID_MAX_OUT, LEG_PID_MAX_IOUT); // 伸腿pid
sp::PID roll_pid(T_CHASSIS_R, ROLL_PID_KP, ROLL_PID_KI, ROLL_PID_KD, ROLL_PID_MAX_OUT, ROLL_PID_MAX_IOUT);
sp::PID tp_pid(T_CHASSIS_R, TP_PID_KP, TP_PID_KI, TP_PID_KD, TP_PID_MAX_OUT, TP_PID_MAX_IOUT);

void jump_loop_r(chassis_t *chassis, vmc_leg_t *vmcl);
void chassis_R_init(vmc_leg_t *vmc);
void chassisR_feedback_update(chassis_t *chassis, vmc_leg_t *vmc, sp::Mahony *imu);
void right_ground_detect(chassis_t *chassis, vmc_leg_t *vmcr, sp::Mahony *imu, float *LQR_K);
void chassisR_control_loop(chassis_t *chassis, vmc_leg_t *vmcl, sp::Mahony *imu, float *LQR_K);

extern "C" void chassis_R_task(void const *argument)
{
    osDelay(1000);              // 等待电机上电 再使能
    chassis_R_init(&vmc_right); // 初始化腿部

    while (1)
    {
        chassisR_feedback_update(&chassis_move, &vmc_right, &imu); // 更新数据

        chassisR_control_loop(&chassis_move, &vmc_right, &imu, LQR_K); // 控制计算

        if (chassis_move.start_flag == 1)
        {
            joint_motor_4.cmd(vmc_right.torque_set[0]);
            joint_motor_3.cmd(vmc_right.torque_set[1]);
            wheel_motor_right.cmd(chassis_move.wheel_right_torque);
        }
        if (chassis_move.start_flag == 0)
        {
            joint_motor_4.cmd(0.0f);
            joint_motor_3.cmd(0.0f);
            wheel_motor_right.cmd(0.0f);
        }
        joint_motor_3.write(can2.tx_data);
        can2.send(joint_motor_3.tx_id);
        osDelay(1);
        joint_motor_4.write(can2.tx_data);
        can2.send(joint_motor_4.tx_id);
        osDelay(1);
        wheel_motor_right.write(can2.tx_data);
        can2.send(wheel_motor_right.tx_id);
        osDelay(1);
    }
}

void chassis_R_init(vmc_leg_t *vmc)
{
    VMC_init(vmc); // 给杆长赋值

    // 使能达妙电机
    for (int j = 0; j < 10; j++)
    {
        joint_motor_3.write_enable(can2.tx_data);
        can2.send(joint_motor_3.tx_id);
        osDelay(1);
    }
    for (int j = 0; j < 10; j++)
    {
        joint_motor_4.write_enable(can2.tx_data);
        can2.send(joint_motor_4.tx_id);
        osDelay(1);
    }
    for (int j = 0; j < 10; j++)
    {
        wheel_motor_right.write_enable(can2.tx_data);
        can2.send(wheel_motor_right.tx_id);
        osDelay(1);
    }
}

void chassisR_feedback_update(chassis_t *chassis, vmc_leg_t *vmc, sp::Mahony *imu)
{
    vmc->phi1 = pi / 2.0f + joint_motor_4.angle;
    vmc->phi4 = pi / 2.0f + joint_motor_3.angle;

    chassis->myPitchR = 0.0f - imu->pitch;
    chassis->myPitchGyroR = 0.0f - ins_gyro[1];

    Multiturn_coding(&chassis->total_yaw, &imu->yaw, &chassis->last_yaw, &chassis->yaw_round_count, sp ::PI); // yaw多圈编码

    chassis->roll = imu->roll;
    chassis->theta_err = 0.0f - (vmc->theta + vmc_left.theta);

    if (imu->pitch < (3.1415926f / 6.0f) && imu->pitch > (-3.1415926f / 6.0f))
    { // 根据pitch角度判断倒地自起是否完成
        chassis->recover_flag = 0;
    }
}

void chassisR_control_loop(chassis_t *chassis, vmc_leg_t *vmcr, sp::Mahony *imu, float *LQR_K)
{
    VMC_calc_right(vmcr, imu, T_CHASSIS_R); // 计算theta和d_theta给lqr用，同时也计算左腿长L0,该任务控制周期是3ms秒

    for (int i = 0; i < 12; i++)
    {
        LQR_K[i] = LQR_K_calc(&Poly_Coefficient[i][0], vmcr->L0); // 根据腿长和拟合公式，算出不同腿长下的lqr参数
    }

    chassis->turn_T = TURN_PID_KP * (chassis->turn_set - chassis->total_yaw) - TURN_PID_KD * ins_gyro[2]; // 这样计算更稳一点，陀螺仪噪声小

    chassis->wheel_right_torque = (LQR_K[0] * (vmcr->theta - 0.0f) + LQR_K[1] * (vmcr->d_theta - 0.0f) + LQR_K[2] * (chassis->x_filter - chassis->x_set) +
                                   LQR_K[3] * (chassis->v_filter - chassis->v_set) + LQR_K[4] * (chassis->myPitchR - 0.0f) + LQR_K[5] * (chassis->myPitchGyroR - 0.0f));

    chassis->wheel_left_torque -= chassis->turn_T; // 轮毂电机输出力矩(平动扭矩-转向扭矩)
    limitFloat(chassis->wheel_right_torque, -1.0f, 1.0f);

    // 右边髋关节输出力矩
    vmcr->Tp = (LQR_K[6] * (vmcr->theta - 0.0f) + LQR_K[7] * (vmcr->d_theta - 0.0f) + LQR_K[8] * (chassis->x_filter - chassis->x_set) + LQR_K[9] * (chassis->v_filter - chassis->v_set) +
                LQR_K[10] * (chassis->myPitchR - 0.0f) + LQR_K[11] * (chassis->myPitchGyroR - 0.0f));

    // 防劈叉pid计算
    tp_pid.calc(0.0f, chassis->theta_err);
    chassis->leg_tp = tp_pid.out;

    // roll轴补偿
    roll_pid.calc(0.0f, chassis->roll);
    chassis->roll_force = roll_pid.out;

    vmcr->Tp += chassis->leg_tp; // 髋关节输出力矩(平动力矩+防止劈叉力矩)
    jump_loop_r(chassis, vmcr);

    right_ground_detect(chassis, vmcr, imu, LQR_K);
    limitFloat(vmcr->F0, -150.0f, 150.0f); // 限幅

    VMC_calc_2(vmcr); // 计算期望的关节输出力矩

    // 额定扭矩
    limitFloat(vmcr->torque_set[1], -3.0f, 3.0f);
    limitFloat(vmcr->torque_set[0], -3.0f, 3.0f);
}

void right_ground_detect(chassis_t *chassis, vmc_leg_t *vmcr, sp::Mahony *imu, float *LQR_K)
{
    chassis->right_ground_flag = ground_detectionR(vmcr, imu); // 左腿离地检测

    if (chassis->recover_flag == 0)
    { // 倒地自起不需要检测是否离地
        if (chassis->left_ground_flag == 1 && chassis->right_ground_flag == 1 && vmcr->leg_control_flag == 0)
        { // 当两腿同时离地并且遥控器没有在控制腿的伸缩时，才认为离地
            chassis->wheel_right_torque = 0.0f;

            vmcr->Tp = LQR_K[6] * (vmcr->theta - 0.0f) + LQR_K[7] * (vmcr->d_theta - 0.0f);

            chassis->x_filter = 0.0f; // 对位移清零
            chassis->x_set = chassis->x_filter;
            chassis->turn_set = chassis->total_yaw;
            vmcr->Tp = vmcr->Tp + chassis->leg_tp;
        }
        else
        {                               // 没有离地
            vmcr->leg_control_flag = 0; // 置为0
        }
    }
    else if (chassis->recover_flag == 1)
    {
        vmcr->Tp = 0.0f;
    }
}

void jump_loop_r(chassis_t *chassis, vmc_leg_t *vmcr)
{
    if (chassis->jump_flag == 1)
    {
        if (chassis->jump_status_r == 0)
        {
            leg_right_pid.calc(0.07f, vmcr->L0);

            vmcr->F0 = Mg / fast_cos(vmcr->theta) + leg_right_pid.out; // 前馈+pd
            if (vmcr->L0 < 0.1f)
            {
                chassis->jump_time_r++;
            }
            if (chassis->jump_time_r >= 10 && chassis->jump_time_l >= 10)
            {
                chassis->jump_time_r = 0;
                chassis->jump_status_r = 1;
                chassis->jump_time_l = 0;
                chassis->jump_status_l = 1;
            }
        }
        else if (chassis->jump_status_r == 1)
        {
            leg_right_pid.calc(0.5f, vmcr->L0);
            vmcr->F0 = Mg / fast_cos(vmcr->theta) + leg_right_pid.out; // 前馈+pd
            if (vmcr->L0 > 0.16f)
            {
                chassis->jump_time_r++;
            }
            if (chassis->jump_time_r >= 10 && chassis->jump_time_l >= 10)
            {
                chassis->jump_time_r = 0;
                chassis->jump_status_r = 2;
                chassis->jump_time_l = 0;
                chassis->jump_status_l = 2;
            }
        }
        else if (chassis->jump_status_r == 2)
        {
            leg_right_pid.calc(0.0f, vmcr->L0);                           // chassis->leg_right_set
            vmcr->F0 = 10.0f / fast_cos(vmcr->theta) + leg_right_pid.out; // 前馈+pd Mg
            if (vmcr->L0 < (chassis->leg_right_set + 0.01f))
            {
                chassis->jump_time_r++;
            }
            if (chassis->jump_time_r >= 10 && chassis->jump_time_l >= 10)
            {
                chassis->jump_time_r = 0;
                chassis->jump_status_r = 3;
                chassis->jump_time_l = 0;
                chassis->jump_status_l = 3;
            }
        }
        else
        {
            leg_right_pid.calc(chassis->leg_right_set, vmcr->L0);
            vmcr->F0 = Mg / fast_cos(vmcr->theta) + leg_right_pid.out; // 前馈+pd
        }

        if (chassis->jump_status_r == 3 && chassis->jump_status_l == 3)
        {
            chassis->jump_flag = 0;
            chassis->jump_time_r = 0;
            chassis->jump_status_r = 0;
            chassis->jump_time_l = 0;
            chassis->jump_status_l = 0;
        }
    }
    else
    {
        leg_right_pid.calc(chassis->leg_right_set, vmcr->L0);
        vmcr->F0 = Mg / fast_cos(vmcr->theta) + leg_right_pid.out - chassis->roll_force; // 前馈+pd
    }
}

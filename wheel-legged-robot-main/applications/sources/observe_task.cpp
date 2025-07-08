#include "can_task.hpp"
#include "chassis_L_task.hpp"
#include "chassis_R_task.hpp"
#include "cmsis_os.h"
#include "imu_task.hpp"
#include "io/dbus/dbus.hpp"
#include "kalman_filter.hpp"
#include "math.h"
#include "math_lib.hpp"
#include "motor/rm_motor/rm_motor.hpp"
Kalman_t vaKF; // 速度加速度kalman
extern sp::DBus remote;
extern sp::RM_Motor trigger_motor;
extern sp::DM_Motor wheel_motor_left;
extern sp::DM_Motor wheel_motor_right;
extern chassis_t chassis_move;
// 遥控器数据错误标志 1为出错 0为正常
extern uint8_t remote_error_flag;
// 翻车标志 1为翻车 0为正常
uint8_t rollover_flag;
// 遥控器数据接收状态检测 1为未正常收到值 0为正常
uint8_t rc_receive_flag;

float observe_T = 0.002f;

uint16_t rc_receive_num = 0;

void error_flag_init(void);
uint8_t rollover_test(void);
uint8_t rc_work_state_test(void);
float accel_x_solve(void);
extern "C" void observe_task(void const *argument)
{

    // 等待一段时间
    vTaskDelay(100);

    Kalman_Init(&vaKF, observe_T);

    // 正常化后延时 确保检测时所有数据均为正常数据
    vTaskDelay(100);

    static float wr, wl = 0.0f;
    static float vrb, vlb = 0.0f;
    static float aver_v = 0.0f;

    while (1)
    {

        wr = -wheel_motor_right.speed - ins_gyro[1] + vmc_right.d_alpha;
        vrb = wr * 0.0603f + vmc_right.L0 * vmc_right.d_theta * fast_cos(vmc_right.theta) + vmc_right.d_L0 * fast_sin(vmc_right.theta);

        wl = -wheel_motor_left.speed + ins_gyro[1] + vmc_left.d_alpha;
        vlb = wl * 0.0603f + vmc_left.L0 * vmc_left.d_theta * fast_cos(vmc_left.theta) + vmc_left.d_L0 * fast_sin(vmc_left.theta);

        aver_v = (vrb - vlb) / 2.0f;
        chassis_move.accel = accel_x_solve();
        Kalman_Update(&vaKF, aver_v, chassis_move.accel);

        chassis_move.v_filter = vaKF.x[0];
        chassis_move.x_filter = chassis_move.x_filter + chassis_move.v_filter * observe_T;

        // 遥控器数据是否出错
        //  remote_error_flag = fn_RemoteControlDataErrorTest();

        // 翻车检测
        rollover_flag = rollover_test();

        // 遥控器状态检测
        rc_receive_flag = rc_work_state_test();

        vTaskDelay(2);
    }
}

// 遥控器错误检测
uint8_t remote_error_test(void)
{
    uint8_t ErrorNum = 0;

    if (abs(remote.ch_lh) > 1 || abs(remote.ch_lv) > 1 || abs(remote.ch_rh) > 1 || abs(remote.ch_rv) > 1)
    {
        ErrorNum++;
    }

    if ((remote.sw_l != sp::DBusSwitchMode::DOWN) && (remote.sw_l != sp::DBusSwitchMode::MID) && (remote.sw_l != sp::DBusSwitchMode::UP))
    {
        ErrorNum++;
    }

    if ((remote.sw_r != sp::DBusSwitchMode::DOWN) && (remote.sw_r != sp::DBusSwitchMode::MID) && (remote.sw_r != sp::DBusSwitchMode::UP))
    {
        ErrorNum++;
    }

    if (abs(remote.mouse.vx) > 1)
    {
        ErrorNum++;
    }

    if (abs(remote.mouse.vy) > 1)
    {
        ErrorNum++;
    }

    if (abs(remote.mouse.vs) > 1)
    {
        ErrorNum++;
    }

    if (ErrorNum == 0)
    {
        return 0;
    }

    return 1;
}

// 翻车检测 翻车返回1 正常返回0
uint8_t rollover_test(void)
{
    uint8_t count = 0;
    static uint16_t rollover_num = 0;
    for (uint8_t i = 0; i < 6; i++)
    {
        if (ins_accel_z[i] < 2.5f)
        {
            count++;
        }
    }
    if (count == 6)
    {
        if (rollover_num < 100)
        {
            rollover_num++;
        }
    }
    else
    {
        rollover_num = 0;
    }

    if (rollover_num == 100)
    {
        return 1;
    }
    return 0;
}

// 遥控器工作状态检测
uint8_t rc_work_state_test(void)
{
    static uint16_t rc_num = 0;
    static uint16_t last_rc_receive_num = 0;

    if (rc_receive_num == last_rc_receive_num)
    {
        if (rc_num < 100)
        {
            rc_num++;
        }
    }
    else
    {
        rc_num = 0;
    }
    last_rc_receive_num = rc_receive_num;
    if (rc_num == 100)
    {
        return 1;
    }
    else
    {
        return 0;
    }
}

float accel_x_solve(void)
{
    float q0 = imu.q[0];
    float q1 = imu.q[1];
    float q2 = imu.q[2];
    float q3 = imu.q[3];
    float a0 = ins_accel[0];
    float a1 = ins_accel[1];
    float a2 = ins_accel[2];
    return a0 * (1 - 2 * q2 * q2 - 2 * q3 * q3) + a1 * (2 * q1 * q2 - 2 * q0 * q3) + a2 * (2 * q1 * q3 + 2 * q0 * q2);
}

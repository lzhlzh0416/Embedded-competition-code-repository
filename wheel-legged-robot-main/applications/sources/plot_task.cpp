#include "cmsis_os.h"
#include "io/dbus/dbus.hpp"
#include "io/plotter/plotter.hpp"
#include "math_lib.hpp"
#include "motor/dm_motor/dm_motor.hpp"
#include "motor/rm_motor/rm_motor.hpp"

#include "tools/mahony/mahony.hpp"
#include "tools/pid/pid.hpp"

#include "can_task.hpp"
#include "chassis_L_task.hpp"
#include "chassis_R_task.hpp"
#include "kalman_filter.hpp"
sp::Plotter plotter(&huart1);
extern sp::Mahony imu;
extern float joint_1_t;
extern float joint_2_t;
extern float joint_3_t;
extern float joint_4_t;
extern float LQR_K[12]; // 1.2784
extern vmc_leg_t vmc_right;
extern chassis_t chassis_move;
extern sp::DM_Motor wheel_motor_left;
extern float v_filter;
extern float x_filter;
extern Kalman_t vaKF; // 速度加速度kalman

extern "C" void plot_task()
{
    while (true)
    {
        plotter.plot(
            // imu.yaw, imu.pitch, imu.roll
            // joint_1_t, joint_2_t, joint_3_t, joint_4_t
            // LQR_K[0] * (-vmc_right.theta - 0.0f), LQR_K[1] * (-vmc_right.d_theta - 0.0f), LQR_K[2] * (chassis_move.x_filter - chassis_move.x_set),
            // LQR_K[3] * (chassis_move.v_filter - chassis_move.v_set), LQR_K[4] * (-chassis_move.myPitchR - 0.0f), LQR_K[5] * (-chassis_move.myPitchGyroR - 0.0f)
            // chassis_move.wheel_left_torque,chassis_move.wheel_right_torque
            //  chassis_move.wheel_left_torque,chassis_move.wheel_right_torque
            //  chassis_move.wheel_left_torque,chassis_move.wheel_right_torque
            // imu.pitch,imu.roll,imu.yaw
            // vmc_right.F0, vmc_right.torque_set[0], vmc_right.torque_set[1], chassis_move.leg_set, vmc_right.L0, vmc_right.theta // 腿长控制
            // LQR_K[6] * (vmc_left.theta - 0.0f), LQR_K[7] * (vmc_left.d_theta - 0.0f), LQR_K[8] * (chassis_move.x_set - chassis_move.x_filter),
            // LQR_K[9] * (chassis_move.v_set - chassis_move.v_filter), LQR_K[10] * (chassis_move.myPitchL - 0.0f), LQR_K[11] * (chassis_move.myPitchGyroL - 0.0f)
            // vmc_left.theta, -imu.pitch, vmc_left.phi0
            // LQR_K[6] * (vmc_right.theta - 0.0f), LQR_K[7] * (vmc_right.d_theta - 0.0f), LQR_K[8] * (chassis_move.x_set - chassis_move.x_filter),
            // LQR_K[9] * (chassis_move.v_set - chassis_move.v_filter), LQR_K[10] * (-chassis_move.myPitchL - 0.0f), LQR_K[11] * (-chassis_move.myPitchGyroL - 0.0f)
            // vmc_left.L0,vmc_right.L0
            // chassis_move.v_filter, chassis_move.v_set, chassis_move.turn_set, chassis_move.turn_T
            // chassis_move.left_ground_flag, chassis_move.right_ground_flag
            // chassis_move.v_filter, chassis_move.x_filter, vaKF.x[0], vaKF.x[1]
            // vmc_left.F0, vmc_right.F0, vmc_left.torque_set[0], chassis_move.left_ground_flag, chassis_move.right_ground_flag, vmc_left.FN, vmc_right.FN
           // chassis_move.wheel_left_torque, chassis_move.wheel_right_torque,chassis_move.v_filter,chassis_move.recover_flag,chassis_move.myPitchL,chassis_move.myPitchR
chassis_move.roll
        );
        osDelay(10);
    }
}

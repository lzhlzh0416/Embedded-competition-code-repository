#ifndef __CHASSISR_TASK_H
#define __CHASSISR_TASK_H

#include "VMC_calc.hpp"
#include "motor/dm_motor/dm_motor.hpp"
#define TP_PID_KP 10.0f
#define TP_PID_KI 0.0f
#define TP_PID_KD 0.1f
#define TP_PID_MAX_OUT 2.0f
#define TP_PID_MAX_IOUT 0.0f

#define TURN_PID_KP 2.0f
#define TURN_PID_KI 0.0f
#define TURN_PID_KD 0.2f
#define TURN_PID_MAX_OUT 1.0f
#define TURN_PID_MAX_IOUT 0.0f

#define ROLL_PID_KP 100.0f
#define ROLL_PID_KI 0.0f
#define ROLL_PID_KD 0.0f
#define ROLL_PID_MAX_OUT 100.0f
#define ROLL_PID_MAX_IOUT 0.0f

constexpr float T_CHASSIS_R = 3e-3; // 云台任务执行周期

extern vmc_leg_t vmc_right;
extern sp::DM_Motor wheel_motor_right;

#endif

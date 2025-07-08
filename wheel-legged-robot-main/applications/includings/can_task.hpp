#ifndef CAN_TASK_H
#define CAN_TASK_H

#include "cmsis_os.h"
#include "io/can/can.hpp"
#include "io/dbus/dbus.hpp"
#include "motor/dm_motor/dm_motor.hpp"
#include "motor/rm_motor/rm_motor.hpp"
#include "tools/mahony/mahony.hpp"
struct odom_t
{
    float x;
    float y;
    float angle;
    float linear_speed;
    float angular_speed;
};

extern sp::DM_Motor joint_motor_1;
extern sp::DM_Motor joint_motor_2;
extern sp::DM_Motor wheel_motor_left;
extern sp::DM_Motor joint_motor_3;
extern sp::DM_Motor joint_motor_4;
extern sp::DM_Motor wheel_motor_right;

extern sp::CAN can1;
extern sp::CAN can2;
// 达妙电机使能帧
void DM_motor_enable(void);
void motor_enable(void);

#endif
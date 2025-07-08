// 无头文件
#include "buzzer_task.hpp"
#include "cmsis_os.h"
#include "imu_task.hpp"
#include "io/buzzer/buzzer.hpp"
#include "io/dbus/dbus.hpp"

extern sp::DBus remote;
extern sp::Buzzer buzzer;

uint8_t calibrate_flag;
uint16_t start_count;
uint16_t calibration_count;

// 陀螺仪零飘数据
float gyro_x_zero;
float gyro_y_zero;
float gyro_z_zero;

float sum_x;
float sum_y;
float sum_z;

// 初始化函数
static void calibration_init(void);
// 校准函数
// 右拨为下,遥感拨到"/ \"累计2s开始陀螺仪校准,校准开始有提示音,10s校准完毕有同样提示音
static void calibration_start(void);
// 陀螺仪均值校准函数
static void gyro_calibration(void);

extern "C" void calibrate_task(void const *argument)
{
    osDelay(500);
    calibration_init();

    while (1)
    {
        calibration_start();
        gyro_calibration();
        osDelay(1);
    }
}

// 初始化函数
void calibration_init(void)
{
    gyro_x_zero = GyroXZero;
    gyro_y_zero = GyroYZero;
    gyro_z_zero = GyroZZero;
    sum_x = 0.0f;
    sum_y = 0.0f;
    sum_z = 0.0f;
    calibrate_flag = 0;
    start_count = 0;
    calibration_count = 0;
}

void calibration_start(void)
{
    // 当摇杆"/\"",右拨杆在下时
    if ((remote.sw_r == sp::DBusSwitchMode::DOWN) && (remote.ch_rh > 0.9) && (remote.ch_rv < -0.9) && (remote.ch_lh < -0.9) && (remote.ch_lv < -0.9))
    {
        // 如果没有在校准
        if (calibrate_flag == 0)
        {
            if (start_count < 2000)
            {
                start_count++;
            }
            // 校准开始
            if (start_count > 2000 - 1)
            {
                start_count = 0;
                calibrate_flag = 1;
                sum_x = 0.0f;
                sum_y = 0.0f;
                sum_z = 0.0f;
                calibrate_buzzer();
            }
        }
    }
}

// 陀螺仪均值校准
void gyro_calibration(void)
{
    if (calibrate_flag == 1)
    {
        if (calibration_count < 20000)
        {
            sum_x += ins_gyro[0];
            sum_y += ins_gyro[1];
            sum_z += ins_gyro[2];
            calibration_count++;
            if (calibration_count == 20000)
            {
                gyro_x_zero += sum_x / 20000;
                gyro_y_zero += sum_y / 20000;
                gyro_z_zero += sum_z / 20000;
            }
        }
        else
        {
            calibrate_flag = 0;
            calibration_count = 0;
            calibrate_buzzer();
        }
    }
}

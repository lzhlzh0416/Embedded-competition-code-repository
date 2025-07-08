#ifndef IMU_TASK_HPP
#define IMU_TASK_HPP
#include "cmsis_os.h"
#include "io/bmi088/bmi088.hpp"
#include "math_lib.hpp"
#include "tim.h"
#include "tools/low_pass_filter/low_pass_filter.hpp"
#include "tools/mahony/mahony.hpp"
#include "tools/pid/pid.hpp"

extern sp::BMI088 bmi088; // C板, TODO 达妙
extern sp::Mahony imu;

// IMU温度
constexpr float IMU_TEMP = 50.0f;
// PID参数
#define ImuTemp_kp 1600.0f
#define ImuTemp_ki 0.2f
#define ImuTemp_kd 0.0f

#define ImuTempMInOut -4500.0f
#define ImuTempMaxOut 4500.0f
#define ImuTempMInIOut -1000.0f
#define ImuTempMaxIOut 1000.0f

// 陀螺仪零漂校准参数
constexpr float GyroXZero = -0.00467737485f;
constexpr float GyroYZero = 0.00194033689;
constexpr float GyroZZero = 0.00181531964;

//     0.00340813026f;
// - 4.69932565e-005f;
// - 0.000117754331f;
// 温度控制函数
void imu_temp_control(float temp);

extern uint8_t calibrate_flag;
extern float gyro_x_zero;
extern float gyro_y_zero;
extern float gyro_z_zero;
extern float ins_gyro[3];
extern float INS_eulers[3];
extern float ins_accel_z[6];
extern float ins_accel[3];

// extern float INS_quat[4];

void fn_CalibrationInit(void);

void fn_CalibrationStart(void);

void fn_GyroCalibration(void);

#endif

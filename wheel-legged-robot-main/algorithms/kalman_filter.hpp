#ifndef KALMAN_FILTER_HPP
#define KALMAN_FILTER_HPP
typedef struct
{
    float x[2];    // 状态变量：x[0]速度，x[1]加速度
    float p[2][2]; // 协方差矩阵
    float q[2];    // 过程噪声协方差 Q
    float r[2];    // 观测噪声协方差 R
    float dt;      // 控制周期（秒）
} Kalman_t;

void Kalman_Init(Kalman_t *kf, float dt);

void Kalman_Update(Kalman_t *kf, float meas_speed, float meas_acc);

#endif
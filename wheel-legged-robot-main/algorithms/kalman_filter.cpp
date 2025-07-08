#include "kalman_filter.hpp"

void Kalman_Init(Kalman_t *kf, float dt)
{
    kf->x[0] = 0.0f;
    kf->x[1] = 0.0f;

    kf->p[0][0] = 1.0f;
    kf->p[0][1] = 0.0f;
    kf->p[1][0] = 0.0f;
    kf->p[1][1] = 1.0f;

    kf->q[0] = 0.01f; // 速度过程噪声
    kf->q[1] = 0.01f; // 加速度过程噪声
    kf->r[0] = 50.0f;  // 轮速观测噪声
    kf->r[1] = 1.0f;  // 加速度观测噪声
    kf->dt = dt;
}

void Kalman_Update(Kalman_t *kf, float meas_speed, float meas_acc)
{
    float dt = kf->dt;

    // Step 1: 状态预测
    kf->x[0] += dt * kf->x[1]; // v = v + a * dt
    // 加速度保持不变（匀加速模型）

    // Step 2: 协方差预测
    float p00 = kf->p[0][0] + dt * (kf->p[1][0] + kf->p[0][1]) + dt * dt * kf->p[1][1] + kf->q[0];
    float p01 = kf->p[0][1] + dt * kf->p[1][1];
    float p10 = kf->p[1][0] + dt * kf->p[1][1];
    float p11 = kf->p[1][1] + kf->q[1];
    kf->p[0][0] = p00;
    kf->p[0][1] = p01;
    kf->p[1][0] = p10;
    kf->p[1][1] = p11;

    // Step 3: 计算卡尔曼增益
    float s0 = kf->p[0][0] + kf->r[0];
    float s1 = kf->p[1][1] + kf->r[1];
    float k0 = kf->p[0][0] / s0;
    float k1 = kf->p[1][1] / s1;

    // Step 4: 更新状态
    kf->x[0] += k0 * (meas_speed - kf->x[0]);
    kf->x[1] += k1 * (meas_acc - kf->x[1]);

    // Step 5: 更新协方差
    kf->p[0][0] *= (1 - k0);
    kf->p[1][1] *= (1 - k1);
}
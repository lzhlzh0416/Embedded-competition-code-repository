#ifndef MATH_LIB_HPP
#define MATH_LIB_HPP

#include <cstdint>

#define TABLE_SIZE 1024
#define TWO_PI (2.0f * PI)

constexpr float PI = 3.14159265358979323846f;

extern const float sin_table[TABLE_SIZE];

float wrap_angle(float x);
float fast_sin(float x);
float fast_cos(float x);

// 短整型限幅
void limitUint16(uint16_t &value, uint16_t min, uint16_t max);

// 浮点型限幅
void limitFloat(float &value, float min, float max);

// 短整型循环限幅
uint16_t loopLimitUint16(uint16_t value, uint16_t min, uint16_t max);

// 浮点型循环限幅
float loopLimitFloat(float value, float min, float max);

// 平方根倒数
float invSqrt(float x);

// 范围判断
bool isOutOfScope(float value, float min, float max);

// 弧度格式化（沿劣弧） -PI~PI
float radFormat(float rad);

// 一阶低通滤波器
void lowPassFilter(float &filteredValue, float rawValue, float alpha);

// 斜坡跟随
void slopeFollowing(float *target, float *set, float acc);
// 多圈编码
void Multiturn_coding(float *total_num, float *relative_num, float *last_num, int *round_count, float T);

#endif // MATH_LIB_HPP

#include "math_lib.hpp"
#include "sin_table_data.inc"
#include <cmath>

// 短整型限幅
void limitUint16(uint16_t &value, uint16_t min, uint16_t max)
{
    if (value > max)
        value = max;
    else if (value < min)
        value = min;
}

// 浮点型限幅
void limitFloat(float &value, float min, float max)
{
    if (value > max)
        value = max;
    else if (value < min)
        value = min;
}

// 短整型循环限幅
uint16_t loopLimitUint16(uint16_t value, uint16_t min, uint16_t max)
{
    if (max < min)
        return value;

    uint16_t range = max - min + 1;
    return min + (value - min + range) % range;
}

// 浮点型循环限幅
float loopLimitFloat(float value, float min, float max)
{
    if (max < min)
        return value;

    float range = max - min;
    while (value > max)
        value -= range;
    while (value < min)
        value += range;

    return value;
}

// 平方根倒数
float invSqrt(float x)
{
    float halfx = 0.5f * x;
    float y = x;
    long i = *(long *)&y;
    i = 0x5f3759df - (i >> 1); // 魔法常数
    y = *(float *)&i;
    y = y * (1.5f - (halfx * y * y)); // 牛顿迭代法提高精度
    return y;
}

// 范围判断
bool isOutOfScope(float value, float min, float max) { return !(min < value && value < max); }

// 弧度格式化
float radFormat(float rad)
{
    while (rad > PI)
        rad -= 2 * PI;
    while (rad < -PI)
        rad += 2 * PI;
    return rad;
}

// 一阶低通滤波器
void lowPassFilter(float &filteredValue, float rawValue, float alpha) { filteredValue = filteredValue * (1.0f - alpha) + rawValue * alpha; }

void slopeFollowing(float *target, float *set, float acc)
{
    if (acc <= 0)
        return; // 防止传入负数或0导致死循环

    if (*target > *set)
    {
        *set += acc;
        if (*set >= *target)
            *set = *target;
    }
    else if (*target < *set)
    {
        *set -= acc;
        if (*set <= *target)
            *set = *target;
    }
}

// 多圈编码
void Multiturn_coding(float *total_num, float *relative_num, float *last_num, int *round_count, float T)
{
    float delta = *relative_num - *last_num;

    if (delta > T)
    {
        (*round_count)--;
    }
    else if (delta < -T)
    {
        (*round_count)++;
    }

    *last_num = *relative_num;
    *total_num = 2 * T * (*round_count) + *relative_num;
}

float wrap_angle(float x)
{
    while (x < 0)
        x += TWO_PI;
    while (x >= TWO_PI)
        x -= TWO_PI;
    return x;
}

float fast_sin(float x)
{
    x = wrap_angle(x);
    float index = x * (TABLE_SIZE / TWO_PI);
    int idx = (int)index;
    float frac = index - idx;
    int idx_next = (idx + 1) % TABLE_SIZE;
    return sin_table[idx] * (1.0f - frac) + sin_table[idx_next] * frac;
}

float fast_cos(float x) { return fast_sin(x + PI / 2); }

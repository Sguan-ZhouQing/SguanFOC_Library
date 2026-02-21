/*
 * @Author: 星必尘Sguan
 * @GitHub: https://github.com/Sguan-ZhouQing
 * @Date: 2026-01-29 15:32:59
 * @LastEditors: 星必尘Sguan|3464647102@qq.com
 * @LastEditTime: 2026-02-13 21:25:25
 * @FilePath: \stm_SguanFOCtest\SguanFOC\Sguan_Calculate.c
 * @Description: SguanFOC库的"浮点转Q31定点运算"实现 - 修复版
 * 
 * Copyright (c) 2026 by $星必尘Sguan, All Rights Reserved. 
 */
#include "Sguan_Calculate.h"
#include <math.h>

#define Q31_MAX     0x7FFFFFFF      // 表示最大值0.9999999995
#define Q31_MIN     0x80000000      // 表示最小值-1.0
#define Q31_HALF    0x40000000      // 表示0.5
#define Q31_ZERO    0x00000000      // 表示0.0

// 浮点常量
#define Q31_MAX_FLOAT  2147483647.0   // Q31_MAX的双精度表示 (2^31-1)
#define Q31_SCALE      (1.0 / 2147483648.0)  // 2^31的倒数

// 预计算64位常量（使用正确的类型）
static const int64_t Q31_MAX_64 = 2147483647LL;
static const int64_t Q31_MIN_64 = -2147483648LL;

// 重写fmodf函数
static float Value_fmodf(float x, float y) {
  if (y == 0.0f) return 0.0f;
  int quotient = (int)(x / y); // 1次除法运算
  return x - quotient * y;     // 1次乘1次减
}

// 重写fabsf函数
float Value_fabsf(float x){
  union {
      float f;
      uint32_t i;
  } u;
  u.f = x;
  u.i &= 0x7FFFFFFF; // 1次位与操作
  return u.f;
}

// 重写isinf函数
int Value_isinf(float x){
    union{
        float f;
        uint32_t i;
    } u = {x};
    
    // 单精度浮点数格式：
    // 符号位(1bit) | 指数位(8bits) | 尾数位(23bits)
    // 无穷大：指数位全1，尾数位全0
    // 去除符号位后检查
    uint32_t exp_mask = 0x7F800000;  // 指数位全1的掩码
    uint32_t mant_mask = 0x007FFFFF;  // 尾数位掩码
    
    // 检查指数位是否全1且尾数位全0
    if ((u.i & exp_mask) == exp_mask && (u.i & mant_mask) == 0) {
        return 1;  // 是无穷大
    }
    return 0;  // 不是无穷大
}

// 重写isnan函数
int Value_isnan(float x){
    union{
        float f;
        uint32_t i;
    } u = {x};
    
    uint32_t exp_mask = 0x7F800000;  // 指数位全1的掩码
    uint32_t mant_mask = 0x007FFFFF;  // 尾数位掩码
    if ((u.i & exp_mask) == exp_mask && (u.i & mant_mask) != 0) {
        return 1;  // 是NaN
    }
    return 0;  // 不是NaN
}

// 重写sqrtf函数
float Value_sqrtf(float x){
    if (x < 0){
        return 0.0f / 0.0f;  // 返回NaN（0/0产生NaN）
    }
    if (x == 0 || x == 1){
        return x;
    }
    float guess = x;
    float epsilon = 0.00001f;  // 精度要求
    // 牛顿迭代公式：guess = (guess + x/guess) / 2
    while (1) {
        float new_guess = (guess + x / guess) * 0.5f;
        if (new_guess > guess){
            if (new_guess - guess < epsilon){
                return new_guess;
            }
        } else {
            if (guess - new_guess < epsilon){
                return new_guess;
            }
        }
        guess = new_guess;
    }
}

// 数值限幅
float Value_Limit(float val, float max, float min) {
    if (val > max) return max;
    if (val < min) return min;
    return val;
}

// 参数取模
float normalize_angle(float angle) {
  float normalized = Value_fmodf(angle, Value_PI*2);
  // 如果结果为负，加上2π使其在[0, 2π)范围内
  if (normalized < 0) {
      normalized += Value_PI*2;
  }
  return normalized;
}

// Q31乘法（带舍入）
q31_t q31_mul(q31_t a, q31_t b){
    int64_t result = (int64_t)a * (int64_t)b;
    result += 1LL << 30;    // 四舍五入：加0.5*2^31=1<<30
    result >>= 31;          // 右移31位
    // 饱和检查
    if (result > Q31_MAX_64) {
        return Q31_MAX;
    }
    if (result < Q31_MIN_64) {
        return Q31_MIN;
    }
    return (q31_t)result;
}

// Q31加法（带饱和检查）
q31_t q31_add(q31_t a, q31_t b){
    int32_t sum = a + b;
    // 检查溢出：正数+正数=负数 表示正溢出
    if ((a > 0) && (b > 0) && (sum <= 0)) {
        return Q31_MAX;
    }
    // 检查溢出：负数+负数=正数 表示负溢出
    if ((a < 0) && (b < 0) && (sum >= 0)) {
        return Q31_MIN;
    }
    return sum;
}

// Q31减法（带饱和检查）
q31_t q31_sub(q31_t a, q31_t b){
    int32_t diff = a - b;
    // 检查溢出：正数-负数=负数 表示正溢出
    if ((a > 0) && (b < 0) && (diff <= 0)) {
        return Q31_MAX;
    }
    // 检查溢出：负数-正数=正数 表示负溢出
    if ((a < 0) && (b > 0) && (diff >= 0)) {
        return Q31_MIN;
    }
    return diff;
}

// Q31除法（带舍入和饱和）
q31_t q31_div(q31_t a, q31_t b){
    if (b == 0){
        // 除零：返回最大值或最小值
        return (a >= 0) ? Q31_MAX : Q31_MIN;
    }
    int64_t numerator = (int64_t)a << 31;
    int64_t result = numerator / b;
    // 计算余数用于四舍五入
    int64_t remainder = numerator % b;
    // 四舍五入
    if (remainder != 0) {
        // 取余数的绝对值
        int64_t rem_abs = (remainder < 0) ? -remainder : remainder;
        int64_t b_abs = (b < 0) ? -(int64_t)b : (int64_t)b;
        // 如果余数的两倍大于等于除数，需要进位
        if (rem_abs * 2 >= b_abs) {
            result += (numerator >= 0) ? 1 : -1;
        }
    }
    // 饱和检查
    if (result > Q31_MAX_64) {
        return Q31_MAX;
    }
    if (result < Q31_MIN_64) {
        return Q31_MIN;
    }
    return (q31_t)result;
}

/**
 * @description: 浮点数转Q31定点(带标幺化)
 */
q31_t q31_from_float(float f, float base_value){
    // 1. 参数验证
    if (base_value <= 0.0f || f != f) {  // f != f 检查NaN
        return Q31_ZERO;
    }
    // 2. 标幺化
    float normalized = f / base_value;
    // 3. 饱和处理
    if (normalized >= 1.0f){
        return Q31_MAX;
    } else if (normalized <= -1.0f){
        return Q31_MIN;
    }
    // 4. 转换为Q31
    // 使用双精度提高精度：normalized * 2^31
    // 注意：Q31格式是值 * 2^31，不是 2^31-1
    double scaled = (double)normalized * 2147483648.0;  // 2^31
    // 5. 四舍五入
    if (scaled >= 0.0){
        scaled += 0.5;
    } else {
        scaled -= 0.5;
    }
    // 6. 转换为整数
    int64_t result = (int64_t)scaled;
    // 7. 范围检查（防止因四舍五入溢出）
    if (result > Q31_MAX_64){
        return Q31_MAX;
    } else if (result < Q31_MIN_64){
        return Q31_MIN;
    }
    return (q31_t)result;
}

/**
 * @description: Q31定点转浮点数(带反标幺化)
 */
float q31_to_float(q31_t q, float base_value){
    // 1. 参数检查
    if (base_value <= 0.0f){
        return 0.0f;
    }
    // 2. Q31转归一化浮点
    // Q31格式：值 = q / 2^31
    double normalized = (double)q * Q31_SCALE;
    // 3. 反标幺化
    return (float)(normalized * base_value);
}


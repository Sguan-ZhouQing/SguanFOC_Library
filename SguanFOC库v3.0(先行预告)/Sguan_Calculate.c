/*
 * @Author: 星必尘Sguan
 * @GitHub: https://github.com/Sguan-ZhouQing
 * @Date: 2026-01-29 15:32:59
 * @LastEditors: 星必尘Sguan|3464647102@qq.com
 * @LastEditTime: 2026-02-04 01:12:11
 * @FilePath: \demo_SguanFOCCode\SguanFOC库\Sguan_Calculate.c
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


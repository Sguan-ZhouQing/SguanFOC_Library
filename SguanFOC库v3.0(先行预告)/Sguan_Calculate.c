/*
 * @Author: 星必尘Sguan
 * @GitHub: https://github.com/Sguan-ZhouQing
 * @Date: 2026-01-29 15:32:59
 * @LastEditors: 星必尘Sguan|3464647102@qq.com
 * @LastEditTime: 2026-02-03 23:35:45
 * @FilePath: \demo_SguanFOCCode\SguanFOC库\Sguan_Calculate.c
 * @Description: SguanFOC库的“浮点转Q31定点运算”实现
 * 
 * Copyright (c) 2026 by $星必尘Sguan, All Rights Reserved. 
 */
#include "Sguan_Calculate.h"

#define Q31_MAX     0x7FFFFFFF      // 表示最大值0.99
#define Q31_MIN     0x80000000      // 表示最小值-1.0
#define Q31_HALF    0x40000000      // 表示0.5
#define Q31_ZERO    0x00000000      // 表示0.0

// 浮点常量（使用double提高精度）
#define Q31_MAX_FLOAT  2147483647.0   // Q31_MAX的双精度表示
#define Q31_SCALE_INV  (1.0 / Q31_MAX_FLOAT)  // 除法优化用

// 预计算64位常量（提高效率）
static const int64_t Q31_MAX_64 = (int64_t)Q31_MAX;
static const int64_t Q31_MIN_64 = (int64_t)Q31_MIN;

// 预计算的倒数，提高效率
static const float inv_Q31_MAX = 1.0f / (float)Q31_MAX_FLOAT;

// Q31乘法（带舍入）
q31_t q31_mul(q31_t a, q31_t b){
    int64_t result = (int64_t)a * (int64_t)b;
    result += 1LL << 30;    // 四舍五入：加0.5*2^31=1<<30
    result >>= 31;          // 右移31位
    // 饱和检查（虽然mul乘法结果应该在范围内）
    if (result > Q31_MAX_64) return Q31_MAX;
    if (result < Q31_MIN_64) return Q31_MIN;
    return (q31_t)result;
}

// Q31加法（饱和）
q31_t q31_add(q31_t a, q31_t b){
    int64_t sum = (int64_t)a + (int64_t)b;
    // 饱和检查（虽然add加法结果应该在范围内）
    if (sum > Q31_MAX_64) return Q31_MAX;
    if (sum < Q31_MIN_64) return Q31_MIN;
    return (q31_t)sum;
}

// Q31减法（饱和）
q31_t q31_sub(q31_t a, q31_t b){
    int64_t diff = (int64_t)a - (int64_t)b;
    // 饱和检查（虽然sub减法结果应该在范围内）
    if (diff > Q31_MAX_64) return Q31_MAX;
    if (diff < Q31_MIN_64) return Q31_MIN;
    return (q31_t)diff;
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
    if (remainder < 0) remainder = -remainder;
    int64_t b_abs = (b < 0) ? -(int64_t)b : b;
    if (remainder * 2 >= b_abs){
        // 需要进位
        result += (numerator >= 0) ? 1 : -1;
    }
    // 饱和检查（虽然div除法结果应该在范围内）
    if (result > Q31_MAX_64) return Q31_MAX;
    if (result < Q31_MIN_64) return Q31_MIN;
    return (q31_t)result;
}

/**
 * @description: 浮点数转Q31定点(带标幺化) - 高精度版
 */
q31_t q31_from_float(float f, float base_value){
    // 1. 参数验证
    if (base_value <= 0.0f || isnan(base_value)){
        return Q31_ZERO;
    }
    // 2. 处理特殊浮点值
    if (isnan(f) || isinf(f)){
        return Q31_ZERO;
    }
    // 3. 标幺化
    float normalized = f / base_value;
    // 4. 饱和处理
    if (normalized > 1.0f){
        normalized = 1.0f;
        // 可以添加溢出警告
    } else if (normalized < -1.0f){
        normalized = -1.0f;
        // 可以添加下溢警告
    }
    // 5. 转换为Q31（使用双精度）
    double scaled = (double)normalized * Q31_MAX_FLOAT;
    // 6. 四舍五入
    if (scaled >= 0.0){
        scaled += 0.5;
    } else{
        scaled -= 0.5;
    }
    // 7. 范围检查（防止因四舍五入溢出）
    if (scaled > (double)Q31_MAX){
        return Q31_MAX;
    } else if (scaled < (double)Q31_MIN){
        return Q31_MIN;
    }
    return (q31_t)scaled;
}

/**
 * @description: Q31定点转浮点数(带反标幺化) - 高精度版
 */
float q31_to_float(q31_t q, float base_value){
    // 1. 参数检查
    if (base_value <= 0.0f){
        return 0.0f;
    }
    // 2. Q31转归一化浮点(双精度提高精度)
    double normalized = (double)q * Q31_SCALE_INV;
    // 3. 反标幺化
    return (float)(normalized * base_value);
}

/*
 * @Author: 星必尘Sguan
 * @GitHub: https://github.com/Sguan-ZhouQing
 * @Date: 2026-02-26 22:37:35
 * @LastEditors: 星必尘Sguan|3464647102@qq.com
 * @LastEditTime: 2026-03-20 22:54:37
 * @FilePath: \SguanFOC_Debug\SguanFOC\Sguan_IQmath.c
 * @Description: SguanFOC库的“IQmath库”实现（Q17.15：1位符号+16位整数+15位小数）
 * 
 * Copyright (c) 2026 by 星必尘Sguan, All Rights Reserved. 
 */
#include "Sguan_IQmath.h"
#include <stdint.h>

// ===================== Q17.15 常量定义 =====================
#define Q15_SHIFT        15                    // 小数位占15位
#define Q15_MULTIPLIER   (1 << Q15_SHIFT)      // 32768（2^15）
#define Q15_INT_MAX      65535                 // 16位整数部分最大值（无符号）
#define Q15_INT_MIN      -65536                // 16位整数部分最小值（含符号）
#define Q15_MAX_FLOAT    (Q15_INT_MAX + (Q15_MULTIPLIER - 1.0f) / Q15_MULTIPLIER) // 65535.9999694824
#define Q15_MIN_FLOAT    (float)Q15_INT_MIN    // -65536.0
#define Q15_MAX          ((int32_t)0x7FFFFFFF) // Q17.15最大值（对应65535.999969）
#define Q15_MIN          ((int32_t)0x80000000) // Q17.15最小值（对应-65536.0）
#define Q15_SCALE        (1.0f / Q15_MULTIPLIER) // 缩放系数（转浮点数用）

// ===================== Q31 常量定义 =====================
#define Q31_MAX         0x7FFFFFFF      // 表示最大值0.9999999995
#define Q31_MIN         0x80000000      // 表示最小值-1.0
#define Q31_HALF        0x40000000      // 表示0.5
#define Q31_ZERO        0x00000000      // 表示0.0

// 浮点常量
#define Q31_MAX_FLOAT   2147483647.0    // Q31_MAX的双精度表示 (2^31-1)
#define Q31_SCALE       (1.0 / 2147483648.0)  // 2^31的倒数

// 预计算64位常量（使用正确的类型）
static const int64_t Q31_MAX_64 = 2147483647LL;
static const int64_t Q31_MIN_64 = -2147483648LL;

// ===================== Q17.15 核心函数实现 =====================
/**
 * @brief  浮点数转Q17.15定点数（1位符号+16位整数+15位小数）
 * @param  f: 待转换浮点数
 * @retval 饱和后的Q17.15定点数
 */
Q15_t IQmath_float_to_Q15(float f){
    if (f > Q15_MAX_FLOAT) return Q15_MAX;
    if (f < Q15_MIN_FLOAT) return Q15_MIN;
    
    float temp = f * Q15_MULTIPLIER;
    temp = (temp >= 0) ? (temp + 0.5f) : (temp - 0.5f);
    
    return (Q15_t)temp;
}

/**
 * @brief  Q17.15定点数转浮点数
 * @param  q: 待转换Q17.15定点数
 * @retval 转换后的浮点数
 */
float IQmath_Q15_to_float(Q15_t q){
    return (float)q * Q15_SCALE;
}

/**
 * @brief  Q17.15定点数加法（带饱和处理）
 * @param  a: 加数1（Q17.15格式）
 * @param  b: 加数2（Q17.15格式）
 * @retval 加法结果（饱和后，Q17.15格式）
 */
Q15_t IQmath_Q15_add(Q15_t a, Q15_t b){
    int64_t sum = (int64_t)a + (int64_t)b;
    
    // 饱和处理
    if (sum > Q15_MAX) return Q15_MAX;
    if (sum < Q15_MIN) return Q15_MIN;
    
    return (Q15_t)sum;
}

/**
 * @brief  Q17.15定点数减法（带饱和处理）
 * @param  a: 被减数（Q17.15格式）
 * @param  b: 减数（Q17.15格式）
 * @retval 减法结果（饱和后，Q17.15格式）
 */
Q15_t IQmath_Q15_sub(Q15_t a, Q15_t b){
    int64_t diff = (int64_t)a - (int64_t)b;
    
    // 饱和处理
    if (diff > Q15_MAX) return Q15_MAX;
    if (diff < Q15_MIN) return Q15_MIN;
    
    return (Q15_t)diff;
}

/**
 * @brief  Q17.15定点数乘法（带饱和处理）
 * @param  a: 乘数1（Q17.15格式）
 * @param  b: 乘数2（Q17.15格式）
 * @retval 乘法结果（饱和后，Q17.15格式）
 */
Q15_t IQmath_Q15_mul(Q15_t a, Q15_t b) {
    int64_t temp = (int64_t)a * (int64_t)b;
    temp = (temp + (1LL << (Q15_SHIFT - 1))) >> Q15_SHIFT;
    
    // 饱和处理
    if (temp > Q15_MAX) return Q15_MAX;
    if (temp < Q15_MIN) return Q15_MIN;
    
    return (Q15_t)temp;
}

/**
 * @brief  Q17.15定点数除法（带饱和处理）
 * @param  a: 被除数（Q17.15格式）
 * @param  b: 除数（Q17.15格式）
 * @retval 除法结果（饱和后，Q17.15格式）
 */
Q15_t IQmath_Q15_div(Q15_t a, Q15_t b) {
    if (b == 0) return (a >= 0) ? Q15_MAX : Q15_MIN;
    
    int64_t temp = ((int64_t)a << Q15_SHIFT) / b;
    
    // 饱和处理
    if (temp > Q15_MAX) return Q15_MAX;
    if (temp < Q15_MIN) return Q15_MIN;
    
    return (Q15_t)temp;
}

// ===================== Q31 核心函数实现 =====================

/**
 * @brief  浮点数转Q31定点(带标幺化)
 * @param  f: 待转换浮点数
 * @param  base_value: 基准值（标幺化分母）
 * @retval Q31定点数
 */
Q31_t IQmath_q31_from_float(float f, float base_value) {
    // 1. 参数验证
    if (base_value <= 0.0f || f != f) {  // f != f 检查NaN
        return Q31_ZERO;
    }
    // 2. 标幺化
    float normalized = f / base_value;
    // 3. 饱和处理
    if (normalized >= 1.0f) {
        return Q31_MAX;
    } else if (normalized <= -1.0f) {
        return Q31_MIN;
    }
    // 4. 转换为Q31
    // 使用双精度提高精度：normalized * 2^31
    double scaled = (double)normalized * 2147483648.0;  // 2^31
    // 5. 四舍五入
    if (scaled >= 0.0) {
        scaled += 0.5;
    } else {
        scaled -= 0.5;
    }
    // 6. 转换为整数
    int64_t result = (int64_t)scaled;
    // 7. 范围检查（防止因四舍五入溢出）
    if (result > Q31_MAX_64) {
        return Q31_MAX;
    } else if (result < Q31_MIN_64) {
        return Q31_MIN;
    }
    return (Q31_t)result;
}

/**
 * @brief  Q31定点转浮点数(带反标幺化)
 * @param  q: Q31定点数
 * @param  base_value: 基准值（反标幺化乘数）
 * @retval 转换后的浮点数
 */
float IQmath_q31_to_float(Q31_t q, float base_value) {
    // 1. 参数检查
    if (base_value <= 0.0f) {
        return 0.0f;
    }
    // 2. Q31转归一化浮点
    double normalized = (double)q * Q31_SCALE;
    // 3. 反标幺化
    return (float)(normalized * base_value);
}

/**
 * @brief  Q31定点数乘法（带舍入和饱和）
 * @param  a: 乘数1（Q1.31格式）
 * @param  b: 乘数2（Q1.31格式）
 * @retval 乘法结果（Q1.31格式）
 */
Q31_t IQmath_q31_mul(Q31_t a, Q31_t b) {
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
    return (Q31_t)result;
}

/**
 * @brief  Q31定点数除法（带舍入和饱和）
 * @param  a: 被除数（Q1.31格式）
 * @param  b: 除数（Q1.31格式）
 * @retval 除法结果（Q1.31格式）
 */
Q31_t IQmath_q31_div(Q31_t a, Q31_t b) {
    if (b == 0) {
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
    return (Q31_t)result;
}

/**
 * @brief  Q31定点数加法（带饱和检查）
 * @param  a: 加数1（Q1.31格式）
 * @param  b: 加数2（Q1.31格式）
 * @retval 加法结果（Q1.31格式）
 */
Q31_t IQmath_q31_add(Q31_t a, Q31_t b) {
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

/**
 * @brief  Q31定点数减法（带饱和检查）
 * @param  a: 被减数（Q1.31格式）
 * @param  b: 减数（Q1.31格式）
 * @retval 减法结果（Q1.31格式）
 */
Q31_t IQmath_q31_sub(Q31_t a, Q31_t b) {
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

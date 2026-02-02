/*
 * @Author: 星必尘Sguan
 * @GitHub: https://github.com/Sguan-ZhouQing
 * @Date: 2026-01-29 15:32:59
 * @LastEditors: 星必尘Sguan|3464647102@qq.com
 * @LastEditTime: 2026-02-02 08:43:46
 * @FilePath: \demo_SguanFOCCode\SguanFOC库\Sguan_Calculate.c
 * @Description: SguanFOC库的“浮点转Q31定点运算”实现
 * 
 * Copyright (c) 2026 by $星必尘Sguan, All Rights Reserved. 
 */
#include "Sguan_Calculate.h"


// Q31乘法（64位中间结果，右移31位）
q31_t q31_mul(q31_t a, q31_t b) {
    int64_t result = (int64_t)a * (int64_t)b;
    return (q31_t)(result >> 31);
}

// Q31加法（饱和处理）
q31_t q31_add(q31_t a, q31_t b) {
    int64_t sum = (int64_t)a + (int64_t)b;
    if (sum > Q31_MAX) return Q31_MAX;
    if (sum < Q31_MIN) return Q31_MIN;
    return (q31_t)sum;
}

// Q31减法（饱和处理）
q31_t q31_sub(q31_t a, q31_t b) {
    int64_t diff = (int64_t)a - (int64_t)b;
    if (diff > Q31_MAX) return Q31_MAX;
    if (diff < Q31_MIN) return Q31_MIN;
    return (q31_t)diff;
}

// Q31除法（备用函数，使用定点除法）
q31_t q31_div(q31_t a, q31_t b) {
    if (b == 0) return (a >= 0) ? Q31_MAX : Q31_MIN;
    // 使用64位中间结果避免溢出
    int64_t result = ((int64_t)a << 31) / b;
    // 饱和处理
    if (result > Q31_MAX) return Q31_MAX;
    if (result < Q31_MIN) return Q31_MIN;
    return (q31_t)result;
}

// 浮点数转Q31
q31_t q31_from_float(float f) {
    if (f >= 1.0f) return Q31_MAX;
    if (f <= -1.0f) return Q31_MIN;
    return (q31_t)(f * (float)Q31_MAX);
}

// Q31转浮点数
float q31_to_float(q31_t q) {
    return (float)q / (float)Q31_MAX;
}

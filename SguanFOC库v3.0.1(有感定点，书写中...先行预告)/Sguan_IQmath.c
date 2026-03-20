/*
 * @Author: 星必尘Sguan
 * @GitHub: https://github.com/Sguan-ZhouQing
 * @Date: 2026-02-26 22:37:35
 * @LastEditors: 星必尘Sguan|3464647102@qq.com
 * @LastEditTime: 2026-03-20 22:54:37
 * @FilePath: \SguanFOC_Debug\SguanFOC\Sguan_IQmath.c
 * @Description: SguanFOC库的“IQmath库”实现
 * 
 * Copyright (c) 2026 by $星必尘Sguan, All Rights Reserved. 
 */
#include "Sguan_IQmath.h"

#define Q_SHIFT         15
#define Q_MULTIPLIER    (1 << Q_SHIFT)  // 32768

// 浮点数转Q16.15
int32_t float_to_q16_15(float f) {
    // 乘以2^15并四舍五入
    return (int32_t)(f * Q_MULTIPLIER + (f >= 0 ? 0.5f : -0.5f));
}

// Q16.15转浮点数
float q16_15_to_float(int32_t q) {
    return (float)q / Q_MULTIPLIER;
}

// Q16.15乘法（保持Q16.15格式）
int32_t q16_15_mul(int32_t a, int32_t b) {
    // 使用64位防止溢出
    int64_t temp = (int64_t)a * (int64_t)b;
    // 右移15位保持Q16.15格式
    return (int32_t)(temp >> Q_SHIFT);
}

// Q16.15除法
int32_t q16_15_div(int32_t a, int32_t b) {
    if (b == 0) return 0;
    // 左移15位再除法，保持精度
    return (int32_t)(((int64_t)a << Q_SHIFT) / b);
}




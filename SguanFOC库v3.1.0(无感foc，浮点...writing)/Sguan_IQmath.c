/*
 * @Author: 星必尘Sguan
 * @GitHub: https://github.com/Sguan-ZhouQing
 * @Date: 2026-04-30 01:49:40
 * @LastEditors: 星必尘Sguan|3464647102@qq.com
 * @LastEditTime: 2026-04-30 01:56:47
 * @FilePath: \SguanFOC_Debug\SguanFOC\Sguan_IQmath.c
 * @Description: SguanFOC库的“IQmath库”实现
 * 
 * Copyright (c) 2026 by $星必尘Sguan, All Rights Reserved. 
 */
#include "Sguan_IQmath.h"

// ============================= Q1.15 常量定义 =============================
#define Q15_FRAC_BITS       15                      // 小数位占15位
#define Q15_SCALE_FACTOR    (1 << Q15_FRAC_BITS)    // 32768 (2^15)
#define Q15_MAX             ((Q15_t)0x7FFF)         // 最大值32767(对应 0.999)
#define Q15_MIN             ((Q15_t)0x8000)         // 最小值-32768(对应 -1.0)
#define Q15_SCALE_FLOAT     (1.0f / 32768.0f)       // 转浮点数的缩放因子

/**
 * @brief  浮点数转Q1.15定点数（带饱和与舍入）
 * @param  f: 待转换浮点数（应在[-1.0, 1.0)范围内）
 * @retval Q1.15格式的int16_t定点数
 */
Q15_t IQmath_Q15_from_float(float f){
    if (f >= 1.0f){
        return Q15_MAX;
    }
    if (f <= -1.0f){
        return Q15_MIN;
    }
    
    float scaled = f*Q15_SCALE_FACTOR;
    int32_t temp;
    
    if (scaled >= 0){
        temp = (int32_t)(scaled + 0.5f);
    } else{
        temp = (int32_t)(scaled - 0.5f);
    }
    
    if (temp > Q15_MAX){
        temp = Q15_MAX;
    }
    if (temp < Q15_MIN){
        temp = Q15_MIN;
    }
    return (Q15_t)temp;
}

/**
 * @brief  Q1.15定点数转浮点数（简洁版）
 * @param  q: Q1.15格式的int16_t定点数
 * @retval 转换后的浮点数
 */
float IQmath_Q15_to_float(Q15_t q){
    return (float)q * Q15_SCALE_FLOAT;
}

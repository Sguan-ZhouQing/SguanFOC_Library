#ifndef __SGUAN_IQMATH_H
#define __SGUAN_IQMATH_H

/* 外部函数声明 */
#include <stdint.h>
#include <stdio.h>
#include "Sguan_math.h"

// 定点化计算格式
typedef int32_t Q15_t;      // Q17.15格式：1位符号+16位整数+15位小数
typedef int32_t Q31_t;      // Q1.31格式：1位符号+0位整数+31位小数

// IQ15的定点化公式计算
Q15_t IQmath_float_to_Q15(float f);
float IQmath_Q15_to_float(Q15_t q);
Q15_t IQmath_Q15_add(Q15_t a, Q15_t b);
Q15_t IQmath_Q15_sub(Q15_t a, Q15_t b);
Q15_t IQmath_Q15_mul(Q15_t a, Q15_t b);
Q15_t IQmath_Q15_div(Q15_t a, Q15_t b);

// Q31的定点化公式计算
Q31_t IQmath_q31_from_float(float f, float base_value);
float IQmath_q31_to_float(Q31_t q, float base_value);
Q31_t IQmath_q31_mul(Q31_t a, Q31_t b);
Q31_t IQmath_q31_div(Q31_t a, Q31_t b);
Q31_t IQmath_q31_add(Q31_t a, Q31_t b);
Q31_t IQmath_q31_sub(Q31_t a, Q31_t b);
Q31_t IQmath_q31_convert_base(Q31_t q, float src_base, float dst_base);

// 快速正余弦求解
#define fast_cos_q31(x) fast_sin_q31(1.5707963f - x);
Q31_t fast_sin_q31(float x);
void fast_sin_cos_q31(float x, Q31_t *sin_x, Q31_t *cos_x);


#endif // SGUAN_IQMATH_H

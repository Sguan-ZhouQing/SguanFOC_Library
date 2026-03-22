#ifndef __SGUAN_IQMATH_H
#define __SGUAN_IQMATH_H

/* 外部函数声明 */
#include "Sguan_Config.h"
#include <stdint.h>

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

#endif // SGUAN_IQMATH_H

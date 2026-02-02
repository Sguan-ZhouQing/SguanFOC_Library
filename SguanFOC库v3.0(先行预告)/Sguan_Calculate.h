#ifndef __SGUAN_CALCULATE_H
#define __SGUAN_CALCULATE_H

/* 外部函数声明 */
#include "UserData_Calculate.h"
#include <stdint.h>

// 常量宏定义
#define Value_PI 3.14159265358979323846f
#define Q31_MAX 0x7FFFFFFF      // Q31最大值 (0.9999999995)
#define Q31_MIN 0x80000000      // Q31最小值 (-1.0)
#define Q31_ONE 0x7FFFFFFF      // Q31的1.0表示
#define Q31_HALF 0x40000000     // Q31的0.5表示
#define Q31_SQRT3 0x6ED9EBA1    // sqrt(3) in Q31
#define Q31_INV_SQRT3 0x49E6F2A4 // 1/sqrt(3) in Q31
#define Q31_RAD60 0x55555555    // 60° in Q31 (π/3)

// Q31运算函数声明
typedef int32_t q31_t;

// Q31基本运算
q31_t q31_mul(q31_t a, q31_t b);
q31_t q31_add(q31_t a, q31_t b);
q31_t q31_sub(q31_t a, q31_t b);
q31_t q31_div(q31_t a, q31_t b);  // 备用除法函数
q31_t q31_from_float(float f);
float q31_to_float(q31_t q);


#endif // SGUAN_CALCULATE_H

#ifndef __SGUAN_CALCULATE_H
#define __SGUAN_CALCULATE_H

/* 外部函数声明 */
#include "UserData_Calculate.h"
#include <stdint.h>

// 常量宏定义
#define Value_PI 3.14159265358979323846f

// Q31运算函数声明
typedef int32_t q31_t;

// Q31基本运算
q31_t q31_mul(q31_t a, q31_t b);
q31_t q31_add(q31_t a, q31_t b);
q31_t q31_sub(q31_t a, q31_t b);
q31_t q31_div(q31_t a, q31_t b);
q31_t q31_from_float(float f, float base_value);
float q31_to_float(q31_t q, float base_value);


#endif // SGUAN_CALCULATE_H

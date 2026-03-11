#ifndef __SGUAN_CALCULATE_H
#define __SGUAN_CALCULATE_H

/* 外部函数声明 */
#include "UserData_Calculate.h"

typedef signed char         int8_t;
typedef signed short int    int16_t;
typedef signed int          int32_t;
typedef signed long long    int64_t;
typedef unsigned char       uint8_t;
typedef unsigned short int  uint16_t;
typedef unsigned int        uint32_t;
typedef unsigned long long  uint64_t;

// 常量宏定义
#define Value_PI 3.14159265358979323846f

// Q31运算函数声明
typedef int32_t q31_t;

// 重写C标准库
float Value_fabsf(float x);
int Value_isinf(float x);
int Value_isnan(float x);
float Value_sqrtf(float x);

// 参数限制函数
float Value_Limit(float val, float max, float min);
float Value_normalize(float angle);

// Q31基本运算
q31_t q31_mul(q31_t a, q31_t b);
q31_t q31_add(q31_t a, q31_t b);
q31_t q31_sub(q31_t a, q31_t b);
q31_t q31_div(q31_t a, q31_t b);
q31_t q31_from_float(float f, float base_value);
float q31_to_float(q31_t q, float base_value);


#endif // SGUAN_CALCULATE_H

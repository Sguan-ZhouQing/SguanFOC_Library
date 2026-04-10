#ifndef __SGUAN_MATH_H
#define __SGUAN_MATH_H

/* 外部函数声明 */
#include <stdint.h>
#include <stdio.h>

// 常量宏定义
#define Value_PI            3.141592653589793f
#define Value_2PI           6.283185307179586f
#define Value_512_2PI       81.487330863050417f

// 重写C标准库
float Value_fabsf(float x);
int Value_isinf(float x);
int Value_isnan(float x);
float Value_sqrtf(float x);

// 参数限制函数
float Value_Limit(float val, float max, float min);
float Value_normalize(float angle);

// 快速正余弦求解float版本
#define fast_cos(x) fast_sin(1.5707963f - x);
float fast_sin(float x);
void fast_sin_cos(float x, float *sin_x, float *cos_x);


#endif // SGUAN_MATH_H

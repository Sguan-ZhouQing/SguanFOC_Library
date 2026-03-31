#ifndef __SGUAN_MATH_H
#define __SGUAN_MATH_H

// /* 外部函数声明 */
// #include "Sguan_Config.h"

// 常量宏定义
#define Value_PI 3.14159265358979323846f
#define Value_2PI 6.2831854f

// 重写C标准库
float Value_fabsf(float x);
int Value_isinf(float x);
int Value_isnan(float x);
float Value_sqrtf(float x);

// 参数限制函数
float Value_Limit(float val, float max, float min);
float Value_normalize(float angle);

// 快速正余弦求解
#define fast_cos(x) fast_sin(1.5707963f - x);
float fast_sin(float x);
void fast_sin_cos(float x, float *sin_x, float *cos_x);


#endif // SGUAN_MATH_H

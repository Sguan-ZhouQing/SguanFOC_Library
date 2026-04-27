#ifndef __SGUAN_MATH_H
#define __SGUAN_MATH_H

/* 外部函数声明 */
#include <stdint.h>
#include <stdio.h>

// 常量宏定义
#define Value_PI            3.141592653589793f      // 圆周率数值
#define Value_PI_2          1.570796326794896f      // 二分之一pi
#define Value_2PI           6.283185307179586f      // 2pi的数值
#define Value_INV_SQRT3     0.5773502691896257f     // 根号三分之壹
#define Value_SQRT3_2       0.8660254037844386f     // 二分之根号三
#define Value_2_SQRT2       2.8284271247461903f     // 根号二分之二
#define Value_N_INF         0xFF800000              // (负无穷数)

// 重写C标准库Value...f
float Value_maxf(float a, float b);
float Value_minf(float a, float b);
float Value_fabsf(float x);
float Value_sqrtf(float x);

// 参数限制函数Value...x
float Value_Limit(float val, float max, float min);
float Value_normalize(float angle);
int8_t Value_set(int8_t val, int8_t max, int8_t min);
float Value_Sign(float value);

// 电机角度生成器
float Value_Rad_Loop(float Rad_s, float T);

// MOTOR公式变换
void clarke(float *i_alpha,float *i_beta,float i_a,float i_b);
void park(float *i_d,float *i_q,float i_alpha,float i_beta,float sine,float cosine);
void ipark(float *u_alpha,float *u_beta,float u_d,float u_q,float sine,float cosine);

// 快速正余弦求解float版本
#define fast_cos(x) fast_sin(Value_PI_2 - x);
float fast_sin(float x);
void fast_sin_cos(float x, float *sin_x, float *cos_x);


#endif // SGUAN_MATH_H

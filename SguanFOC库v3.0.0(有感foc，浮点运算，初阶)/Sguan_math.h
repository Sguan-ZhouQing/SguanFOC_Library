#ifndef __SGUAN_MATH_H
#define __SGUAN_MATH_H

/* 外部函数声明 */
#include "UserData_Calculate.h"

// 常量宏定义
#define Value_PI 3.14159265358979323846f

// 定义嵌入式系统常用类型
typedef signed char         int8_t;
typedef signed short int    int16_t;
typedef signed int          int32_t;
typedef signed long long    int64_t;
typedef unsigned char       uint8_t;
typedef unsigned short int  uint16_t;
typedef unsigned int        uint32_t;
typedef unsigned long long  uint64_t;

// 重写C标准库
float Value_fabsf(float x);
int Value_isinf(float x);
int Value_isnan(float x);
float Value_sqrtf(float x);

// 参数限制函数
float Value_Limit(float val, float max, float min);
float Value_normalize(float angle);

// 快速正余弦求解
float fast_sin(float x);
float fast_cos(float x);
void fast_sin_cos(float x, float *sin_x, float *cos_x);

// 电机相关
void SVPWM(float d, float q, float sin_phi, float cos_phi, float *d_u, float *d_v, float *d_w);
void clarke(float *i_alpha,float *i_beta,float i_a,float i_b);
void park(float *i_d,float *i_q,float i_alpha,float i_beta,float sine,float cosine);
void ipark(float *u_alpha,float *u_beta,float u_d,float u_q,float sine,float cosine);


#endif // SGUAN_MATH_H

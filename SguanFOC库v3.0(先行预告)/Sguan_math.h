#ifndef __SGUAN_MATH_H
#define __SGUAN_MATH_H

/* 外部函数声明 */
#include "Sguan_Calculate.h"
// 重写C标准库
float Sguan_fabsf(float x);

// 参数限制函数
float Value_Limit(float val, float max, float min);
float normalize_angle(float angle);

// 快速正余弦求解
float fast_sin(float x);
float fast_cos(float x);
void fast_sin_cos(float x, float *sin_x, float *cos_x);

// 滤波器
float low_pass_filter(float input, float last_output, float alpha);
float kalman_filter_std(uint8_t Motor_CH, float input, float r, float q);

// 电机相关
void SVPWM(float phi, float d, float q, float *d_u, float *d_v, float *d_w);
void clarke(float *i_alpha,float *i_beta,float i_a,float i_b);
void park(float *i_d,float *i_q,float i_alpha,float i_beta,float sine,float cosine);
void ipark(float *u_alpha,float *u_beta,float u_d,float u_q,float sine,float cosine);


#endif // SGUAN_MATH_H

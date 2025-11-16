#ifndef __SGUAN_MATH_H
#define __SGUAN_MATH_H

#include <stdint.h>
#include <stdbool.h>

// 重写C标准库
float Sguan_fmodf(float x, float y);
float Sguan_fabsf(float x);

// 常量宏定义
#define Value_PI 3.14159265358979323846f
#define Value_rad60 1.047197551196598f
#define Value_INV_SQRT3 0.5773502691896257f
#define Value_SQRT3 1.73205080756887729353f

// 参数限制函数
#define Value_Limit(val, max, min) (((val) > (max)) ? (max) : (((val) < (min)) ? (min) : (val)))
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

#ifndef __SGUAN_SVPWM_H
#define __SGUAN_SVPWM_H

/* 外部函数声明 */
#include "Sguan_Config.h"

// 电机相关
void SVPWM(float d, float q, float sin_phi, float cos_phi, 
          float *d_u, float *d_v, float *d_w);
void clarke(float *i_alpha,float *i_beta,float i_a,float i_b);
void park(float *i_d,float *i_q,float i_alpha,float i_beta,float sine,float cosine);
void ipark(float *u_alpha,float *u_beta,float u_d,float u_q,float sine,float cosine);


#endif // SGUAN_SVPWM_H

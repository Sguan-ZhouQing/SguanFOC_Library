#ifndef __SGUAN_SVPWM_H
#define __SGUAN_SVPWM_H

/* SguanFOC配置文件声明 */
#include "Sguan_Config.h"

// 电机相关
void SVPWM(float u_alpha, float u_beta, 
        float *d_u, float *d_v, float *d_w);
void clarke(float *i_alpha,float *i_beta,float i_a,float i_b);
void park(float *i_d,float *i_q,float i_alpha,float i_beta,float sine,float cosine);
void ipark(float *u_alpha,float *u_beta,float u_d,float u_q,float sine,float cosine);


// ============================ Q31 版本代码 ============================

void SVPWM_q31(Q31_t u_alpha, Q31_t u_beta, 
        Q31_t *d_u, Q31_t *d_v, Q31_t *d_w);
void clarke_q31(Q31_t *i_alpha,Q31_t *i_beta,Q31_t i_a,Q31_t i_b);
void park_q31(Q31_t *i_d,Q31_t *i_q,Q31_t i_alpha,Q31_t i_beta,Q31_t sine,Q31_t cosine);
void ipark_q31(Q31_t *u_alpha,Q31_t *u_beta,Q31_t u_d,Q31_t u_q,Q31_t sine,Q31_t cosine);


#endif // SGUAN_SVPWM_H

#ifndef __SGUAN_SVPWM_H
#define __SGUAN_SVPWM_H

/* SguanFOC配置文件声明 */
#include "Sguan_Config.h"

// 全局变量声明
extern uint8_t sector_SVPWM;

// 电机相关
void SVPWM(Q31_t u_alpha, Q31_t u_beta, 
        Q31_t *d_u, Q31_t *d_v, Q31_t *d_w);
void clarke(Q31_t *i_alpha,Q31_t *i_beta,Q31_t i_a,Q31_t i_b);
void park(Q31_t *i_d,Q31_t *i_q,Q31_t i_alpha,Q31_t i_beta,Q31_t sine,Q31_t cosine);
void ipark(Q31_t *u_alpha,Q31_t *u_beta,Q31_t u_d,Q31_t u_q,Q31_t sine,Q31_t cosine);


#endif // SGUAN_SVPWM_H

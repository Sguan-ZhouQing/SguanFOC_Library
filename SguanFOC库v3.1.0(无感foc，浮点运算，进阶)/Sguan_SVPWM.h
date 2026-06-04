#ifndef __SGUAN_SVPWM_H
#define __SGUAN_SVPWM_H

/* SguanFOC配置文件声明 */
#include "Sguan_Config.h"

// 电机相关
void SVPWM(float u_alpha, float u_beta, 
        float *d_u, float *d_v, float *d_w);


#endif // SGUAN_SVPWM_H

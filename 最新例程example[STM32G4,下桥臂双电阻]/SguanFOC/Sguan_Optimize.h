#ifndef __SGUAN_OPTIMIZE_H
#define __SGUAN_OPTIMIZE_H

/* SguanFOC配置文件声明 */
#include "Sguan_Config.h"

// 最大转矩电流比
void MTPA_Loop(float *Target_id,float flux,float Ld,float Lq,float iq);
void MTPA_Loop_q31(Q31_t *Target_id,Q31_t flux,Q31_t Ld,Q31_t Lq,Q31_t iq);

// 弱磁控制(待写)
// 三次谐波注入(待写)


#endif // SGUAN_OPTIMIZE_H

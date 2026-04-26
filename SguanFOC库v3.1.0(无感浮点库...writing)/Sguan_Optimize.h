#ifndef __SGUAN_OPTIMIZE_H
#define __SGUAN_OPTIMIZE_H

/* SguanFOC配置文件声明 */
#include "Sguan_Config.h"

// 最大转矩电流比
void MTPA_Loop(float *Target_id,float flux,float Ld,float Lq,float iq);

// 弱磁控制(待写)
void FW_Loop(void);


#endif // SGUAN_OPTIMIZE_H

#ifndef __SGUAN_OPTIMIZE_H
#define __SGUAN_OPTIMIZE_H

/* SguanFOC配置文件声明 */
#include "Sguan_Config.h"

void MTPA_Loop(float *Target_id, 
            float flux, 
            float Ld, 
            float Lq, 
            float iq);
float FW_Loop(void *fw, 
            float Ud, 
            float Uq, 
            float Percentage, 
            float Vbus);
void DeadZone_Loop(float *Ua_duty, 
            float *Ub_duty, 
            float *Uc_duty, 
            float Ia, 
            float Ib, 
            float Ic, 
            float Dead_Time);


#endif // SGUAN_OPTIMIZE_H

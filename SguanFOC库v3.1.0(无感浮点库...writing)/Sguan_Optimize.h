#ifndef __SGUAN_OPTIMIZE_H
#define __SGUAN_OPTIMIZE_H

/* SguanFOC配置文件声明 */
#include "Sguan_Config.h"

void MTPA_Loop(float *Target_id, 
            float flux, 
            float Ld, 
            float Lq, 
            float iq);
void FW_Loop(void);
void DeadZone_Loop(float *Ua_error, 
                float *Ub_error, 
                float *Uc_error, 
                float Ia, 
                float Ib, 
                float Ic, 
                float VUBS,
                float Dead_Time);


#endif // SGUAN_OPTIMIZE_H

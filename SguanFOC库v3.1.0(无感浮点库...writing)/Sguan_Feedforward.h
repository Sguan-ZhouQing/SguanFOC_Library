#ifndef __SGUAN_FEEDFORWARD_H
#define __SGUAN_FEEDFORWARD_H

/* SguanFOC配置文件声明 */
#include "Sguan_Config.h"

float Feedforward_CurrentD(float We,float Lq,float Iq);
float Feedforward_CurrentQ(float We,float Ld,float Id,float Flux);
float Feedforward_Velocity(float Speed,float Ba);


#endif // SGUAN_FEEDFORWARD_H

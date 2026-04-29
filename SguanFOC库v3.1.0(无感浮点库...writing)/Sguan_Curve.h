#ifndef __SGUAN_CURVE_H
#define __SGUAN_CURVE_H

/* SguanFOC配置文件声明 */
#include "Sguan_Config.h"

typedef struct{
    float Last_in;
    float Gain;
    float Multiple;

    float Input;
    float Output;
}SET_STRUCT;

typedef struct{
    SET_STRUCT set;

    float T;
    float K_max;
    float K_min;
}CURVE_STRUCT;

void CURVE_Loop(CURVE_STRUCT *curve);


#endif // SGUAN_CURVE_H

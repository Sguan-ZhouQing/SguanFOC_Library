#ifndef __SGUAN_CURVE_H
#define __SGUAN_CURVE_H

/* SguanFOC配置文件声明 */
#include "Sguan_Config.h"

typedef struct{
    float Last_in;
    float Gain;
    float K;

    float Input;
    float Output;
}CURVE_GO_STRUCT;

typedef struct{
    CURVE_GO_STRUCT go;

    float T;
    float Cycle;
    float K_max;
}CURVE_STRUCT;

void Curve_Init(CURVE_STRUCT *curve);
void Curve_Loop(CURVE_STRUCT *curve);


#endif // SGUAN_CURVE_H

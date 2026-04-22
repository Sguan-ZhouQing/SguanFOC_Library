#ifndef __SGUAN_IDENTIFY_H
#define __SGUAN_IDENTIFY_H

/* SguanFOC配置文件声明 */
#include "Sguan_Config.h"

typedef struct{
    float Rs;                           // (电机实体参数)相线电阻
    float Ld;                           // (电机实体参数)D轴电感
    float Lq;                           // (电机实体参数)Q轴电感
    int8_t Encoder_Dir;                 // (有感实体参数)编码器方向

    float Flux;                         // (电机实体参数)电机磁链
    float B;                            // (电机实体参数)粘性阻尼
    float J;                            // (电机实体参数)转动惯量
}IDENTIFY_STRUCT;



#endif // SGUAN_IDENTIFY_H

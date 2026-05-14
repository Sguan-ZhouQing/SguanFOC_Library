#ifndef __SGUAN_LTD_H
#define __SGUAN_LTD_H

/* 外部函数声明 */
#include "Sguan_Config.h"

typedef struct{
    float Input;            // (输入数据)阶跃输入的数值
    float Output;           // (输出数据)输出平滑的数值
}LTD_GO_STRUCT;

typedef struct{
    LTD_GO_STRUCT go;       // (结构体)LTD运输数据

    float T;                // (系统时钟)T离散周期
    float Wc;               // (参数设计)Wc巴特沃斯截止频率
}LTD_STRUCT;

void LTD_Init(LTD_STRUCT *ltd);
void LTD_Loop(LTD_STRUCT *ltd);


#endif // SGUAN_LTD_H

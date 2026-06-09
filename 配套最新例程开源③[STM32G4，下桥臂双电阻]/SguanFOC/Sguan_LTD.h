#ifndef __SGUAN_LTD_H
#define __SGUAN_LTD_H

/* 外部函数声明 */
#include "Sguan_Config.h"

typedef struct{
    float num;              // (中间量)传递函数的分子系数
    float den;              // (中间量)传递函数的分母系数
    float Last_i;           // (数据)历史的输入数值

    float Input;            // (输入数据)阶跃输入的数值
    float Output;           // (输出数据)输出平滑的数值
}LTD_GO_STRUCT;

typedef struct{
    LTD_GO_STRUCT go;       // (结构体)LTD运输数据

    float T;                // (系统时钟)T离散运行的时间周期
    float r;                // (参数设计)r速度因子_跟踪微分器
}LTD_STRUCT;

void LTD_Init(LTD_STRUCT *ltd);
void LTD_Loop(LTD_STRUCT *ltd);


#endif // SGUAN_LTD_H

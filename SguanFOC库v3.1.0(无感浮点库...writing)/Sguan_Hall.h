#ifndef __SGUAN_HALL_H
#define __SGUAN_HALL_H

/* SguanFOC配置文件声明 */
#include "Sguan_Config.h"

typedef struct{
    uint8_t Input_Ga;       // (输入数据)U相输入0或1
    uint8_t Input_Gb;       // (输入数据)V相输入0或1
    uint8_t Input_Gc;       // (输入数据)W相输入0或1
    float Output_Rad;       // (输出数据)输出不连续角度值

    float Hall_A;           // (中间量)三相历史数据
    float Hall_B;           // (中间量)三相历史数据
    float Hall_C;           // (中间量)三相历史数据

    float Gain;             // (中间量)低通滤波输入项系数
    float Normalized_Gain;  // (中间量)低通滤波输出项系数
}RAD_STRUCT;

typedef struct{
    RAD_STRUCT rad;         // (结构体)霍尔传感器运算数据

    float Wc;               // (参数设计)Wc微分环节一阶低通滤波
    float T;                // (参数设计)T离散运行周期

    float Hall_High;        // (参数设计)霍尔信号上边界
    float Hall_Low;         // (参数设计)霍尔信号下边界
}HALL_STRUCT;

void Hall_Init(HALL_STRUCT *hall);
void Hall_Loop(HALL_STRUCT *hall);


#endif // SGUAN_HALL_H

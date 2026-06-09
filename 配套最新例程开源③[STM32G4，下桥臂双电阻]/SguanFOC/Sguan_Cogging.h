#ifndef __SGUAN_COGGING_H
#define __SGUAN_COGGING_H

/* SguanFOC配置文件声明 */
#include "Sguan_Config.h"

// 离线标定的1800点位抗齿槽电流表
#define Cogging_N   1800

typedef struct{
    float Input_Init_Iq;    // (输入数据)Init运行输入的Q轴电流
    float Output_Init_Rad;  // (输出数据)Init运行输出的电机位置

    float Input_Loop_Rad;   // (输入数据)Loop运行输入的电机位置
    float Output_Loop_Iq;   // (输出数据)Loop运行输出的抗齿槽电流
}COGGING_GO_STRUCT;

typedef struct{
    COGGING_GO_STRUCT go;   // (结构体)抗齿槽算法运输数据

    float T;                // (系统时钟)T离散运行的时间周期

    uint8_t Count;          // (参数设计)Count循环平均测的次数
    float Cycle;            // (参数设计)Cycle总周期->单位为秒
}COGGING_STRUCT;

void Cogging_SetIq_Init(COGGING_STRUCT *cogging);
void Cogging_ReadIq_Loop(COGGING_STRUCT *cogging);
int16_t Cogging_ReadIq_Tick(uint16_t K);


#endif // SGUAN_COGGING_H

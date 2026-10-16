#ifndef __SGUAN_PID_H
#define __SGUAN_PID_H

/* SguanFOC配置文件声明 */
#include "Sguan_Config.h"

typedef struct{
    Q31_t Input;             // (数据)数据历史输入值
    Q31_t Io;               // (数据)积分历史输出值

    Q31_t Ref;              // (输入数据)Target期望数值
    Q31_t Fbk;              // (输出数据)Real真实反馈数据
    Q31_t Output;           // (输出数据)Output输出

    Q31_t T;                // (Q31)T离散周期

    Q31_t Kp;               // (Q31)Kp比例项增益
    Q31_t Ki;               // (Q31)Ki积分项增益

    Q31_t OutMax;           // (Q31)输出上限限幅
    Q31_t OutMin;           // (Q31)输出下限限幅

    Q31_t IntMax;           // (Q31)积分项上限
    Q31_t IntMin;           // (Q31)积分项下限

    uint8_t IntegralFrozen_flag; // (中间量)积分抗饱和
}__PID_GO_STRUCT;

typedef struct{
    __PID_GO_STRUCT go;     // (结构体)PID运算结构体

    uint8_t id;             // (判据)此PI控制器用于转速环或电流环

    float T;                // (系统时钟)T离散周期

    float Kp;               // (参数设计)Kp比例项增益
    float Ki;               // (参数设计)Ki积分项增益

    float OutMax;           // (参数设计)输出上限限幅
    float OutMin;           // (参数设计)输出下限限幅

    float IntMax;           // (参数设计)积分项上限
    float IntMin;           // (参数设计)积分项下限
}PID_STRUCT;

void PID_Init(PID_STRUCT *pid);
void PID_Loop(PID_STRUCT *pid);


#endif // SGUAN_PID_H

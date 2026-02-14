#ifndef __SGUAN_PID_H
#define __SGUAN_PID_H

/* 外部函数声明 */
#include "Sguan_Calculate.h"

typedef struct{
    float i[2];     // (数据)数据历史输入值
    float Io[2];    // (数据)积分历史输出值
    float Do[2];    // (数据)微分历史输出值

    float Ref;      // (数据)Target期望数值
    float Fbk;      // (数据)Real真实反馈数据
    float Output;   // (数据)Output输出

    float I_num[2]; // (中间量)积分传递函数分子系数
    float I_den[2]; // (中间量)积分传递函数分母系数
    float D_num[2]; // (中间量)微分传递函数分子系数
    float D_den[2]; // (中间量)微分传递函数分母系数
}RUN_STRUCT;

typedef struct{
    RUN_STRUCT run; // (结构体)PID运算结构体

    double Wc;      // (参数设计)Wc微分环节一阶低通滤波
    double T;       // (参数设计)T周期

    float Kp;       // (参数设计)Kp比例项增益
    double Ki;      // (参数设计)Ki积分项增益
    double Kd;      // (参数设计)Kd微分项增益

    float OutMax;   // (参数设计)输出上限限幅
    float OutMin;   // (参数设计)输出下限限幅

    float IntMax;   // (参数设计)积分项上限
    float IntMin;   // (参数设计)积分项下限
}PID_STRUCT;

void PID_Init(PID_STRUCT *pid);
void PID_Loop(PID_STRUCT *pid);


#endif // SGUAN_PID_H

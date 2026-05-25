#ifndef __SGUAN_LADRC_H
#define __SGUAN_LADRC_H

/* 外部函数声明 */
#include "Sguan_Config.h"

typedef struct{
    float I_num;            // (中间量)积分项传递函数分子系数

    float v1_i;             // (中间量)积分历史输入值
    float v2_i;             // (中间量)积分历史输入值
    float z1_i;             // (中间量)积分历史输入值
    float z2_i;             // (中间量)积分历史输入值
    float z3_i;             // (中间量)积分历史输入值

    float v1;               // (数据)跟踪微分器输出v1(跟踪信号)
    float v2;               // (数据)跟踪微分器输出v2(微分信号)
    
    float z1;               // (数据)观测器输出z1(跟踪反馈信号)
    float z2;               // (数据)观测器输出z2(跟踪反馈微分)
    float z3;               // (数据)观测器输出z3(总扰动估计)
    
    float Ref;              // (输入数据)期望输入值
    float Fbk;              // (输入数据)真实反馈值
    float Output;           // (输出数据)限幅后的控制器输出
    
    float e;                // (中间量)观测器误差 e = z1 - Fbk
    float e1;               // (中间量)跟踪误差 e1 = v1 - z1
    float e2;               // (中间量)微分误差 e2 = v2 - z2
    float u0;               // (中间量)线性控制率输出
    float fh;               // (中间量)最速控制综合函数
}LADRC_RUN_STRUCT;

typedef struct{
    float w0;               // (参数数据~)观测器带宽w0 = 4Wc
    
    float beta1;            // (参数数据~)观测器系数1 = 3*w0
    float beta2;            // (参数数据~)观测器系数2 = 3*w0^2
    float beta3;            // (参数数据~)观测器系数3 = w0^3
    
    float Kp;               // (参数数据~)比例系数 = Wc^2
    float Kd;               // (参数数据~)微分系数 = 2*Wc
}LADRC_DATA_STRUCT;

typedef struct{
    LADRC_RUN_STRUCT run;   // (结构体)LADRC运算数据的结构体
    LADRC_DATA_STRUCT data; // (结构体)控制器自动计算的参数
        
    float T;                // (系统时钟)积分步长_采样周期
    
    float r;                // (参数设计)速度因子_跟踪微分器
    float b0;               // (参数设计)系统参数_控制量增益
    float Wc;               // (参数设计)控制器带宽常设计为Wc = 10/T
    
    float OutMax;           // (参数设计)输出上限
    float OutMin;           // (参数设计)输出下限
}LADRC_STRUCT;

void Ladrc_Init(LADRC_STRUCT *ladrc);
void Ladrc_Loop(LADRC_STRUCT *ladrc);


#endif // SGUAN_LADRC_H

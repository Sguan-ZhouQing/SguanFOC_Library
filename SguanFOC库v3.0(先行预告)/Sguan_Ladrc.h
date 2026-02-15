#ifndef __SGUAN_LADRC_H
#define __SGUAN_LADRC_H

/* 外部函数声明 */
#include "Sguan_Calculate.h"

typedef struct {
    /* 跟踪微分器(TD)变量 */
    float v1;           // (数据)跟踪微分器输出v1(跟踪信号)
    float v2;           // (数据)跟踪微分器输出v2(微分信号)
    
    /* 扩张状态观测器(ESO)变量 */
    float z1;           // (数据)观测器输出z1(跟踪反馈信号)
    float z2;           // (数据)观测器输出z2(跟踪反馈微分)
    float z3;           // (数据)观测器输出z3(总扰动估计)
    
    /* 输入输出变量 */
    float Ref;          // (数据)期望输入值
    float Fbk;          // (数据)真实反馈值
    float u;            // (数据)控制器输出
    float u_sat;        // (数据)限幅后的控制器输出
    
    /* 中间计算结果 */
    float e;            // (中间量)观测器误差 e = z1 - Fbk
    float e1;           // (中间量)跟踪误差 e1 = v1 - z1
    float e2;           // (中间量)微分误差 e2 = v2 - z2
    float u0;           // (中间量)线性控制率输出
    float fh;           // (中间量)最速控制综合函数
} LINEAR_STRUCT;

typedef struct {
    LINEAR_STRUCT linear;  // (结构体)LADRC运算结构体
    
    /* 离散化参数 */
    float h;            // (参数设计)积分步长_采样周期
    float r;            // (参数设计)速度因子(跟踪微分器)
    
    /* 控制器参数 */
    float wc;           // (参数设计)控制器带宽
    float w0;           // (参数设计)观测器带宽
    float b0;           // (参数设计)系统参数_控制量增益
    
    /* 观测器系数(自动计算) */
    float beta01;       // (参数设计)观测器系数1 = 3*w0
    float beta02;       // (参数设计)观测器系数2 = 3*w0^2
    float beta03;       // (参数设计)观测器系数3 = w0^3
    
    /* 控制器系数(自动计算) */
    float Kp;           // (参数设计)比例系数 = wc^2
    float Kd;           // (参数设计)微分系数 = 2*wc
    
    /* 输出限幅 */
    float OutMax;       // (参数设计)输出上限
    float OutMin;       // (参数设计)输出下限
} LADRC_STRUCT;

void Ladrc_Init(LADRC_STRUCT *ladrc);
void Ladrc_Loop(LADRC_STRUCT *ladrc);


#endif // SGUAN_LADRC_H

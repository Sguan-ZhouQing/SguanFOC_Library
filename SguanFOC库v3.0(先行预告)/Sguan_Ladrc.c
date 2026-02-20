/*
 * @Author: 星必尘Sguan
 * @GitHub: https://github.com/Sguan-ZhouQing
 * @Date: 2026-02-16  00:27:53
 * @LastEditors: 星必尘Sguan|3464647102@qq.com
 * @LastEditTime: 2026-02-20 23:18:28
 * @FilePath: \stm_SguanFOCtest\SguanFOC\Sguan_Ladrc.c
 * @Description: SguanFOC库的“典型二阶线性自抗扰控制(LADRC)算法”实现
 * 
 * Copyright (c) 2026 by 星必尘Sguan, All Rights Reserved. 
 */
#include "Sguan_Ladrc.h"

// 静态函数声明 - LADRC核心算法的三个步骤
static void Ladrc_LTD(LADRC_STRUCT *ladrc);
static void Ladrc_LESO(LADRC_STRUCT *ladrc);
static void Ladrc_LinearControlRate(LADRC_STRUCT *ladrc);

// 线性跟踪微分器(LTD) - 安排过渡过程
static void Ladrc_LTD(LADRC_STRUCT *ladrc){
    /* 最速控制综合函数 fh = -r^2*(v1-Ref) - 2*r*v2 */
    ladrc->linear.fh = -ladrc->r * ladrc->r * (ladrc->linear.v1 - ladrc->linear.Ref) 
                    - 2.0f * ladrc->r * ladrc->linear.v2;
    
    /* 离散化更新 v1 = v1 + h*v2, v2 = v2 + h*fh */
    ladrc->linear.v1 += ladrc->linear.v2 * ladrc->T;
    ladrc->linear.v2 += ladrc->linear.fh * ladrc->T;
}

// 线性扩张状态观测器(LESO) - 估计系统状态和总扰动
static void Ladrc_LESO(LADRC_STRUCT *ladrc){
    /* 观测器误差 e = z1 - Fbk */
    ladrc->linear.e = ladrc->linear.z1 - ladrc->linear.Fbk;
    
    /* 扩张状态观测器离散方程
     * z1 += (z2 - beta01*e) * h
     * z2 += (z3 - beta02*e + b0*u_sat) * h
     * z3 += (-beta03*e) * h
     */
    ladrc->linear.z1 += (ladrc->linear.z2 - ladrc->beta01 * ladrc->linear.e) * ladrc->T;
    ladrc->linear.z2 += (ladrc->linear.z3 - ladrc->beta02 * ladrc->linear.e 
                     + ladrc->b0 * ladrc->linear.u_sat) * ladrc->T;
    ladrc->linear.z3 += (-ladrc->beta03 * ladrc->linear.e) * ladrc->T;
}

// 线性控制率(LSEF/LF) - 计算控制量
static void Ladrc_LinearControlRate(LADRC_STRUCT *ladrc){
    /* 误差计算 e1 = v1 - z1, e2 = v2 - z2 */
    ladrc->linear.e1 = ladrc->linear.v1 - ladrc->linear.z1;
    ladrc->linear.e2 = ladrc->linear.v2 - ladrc->linear.z2;
    
    /* 线性反馈控制率 u0 = Kp*e1 + Kd*e2 */
    ladrc->linear.u0 = ladrc->Kp * ladrc->linear.e1 + ladrc->Kd * ladrc->linear.e2;
    
    /* 扰动补偿 u = (u0 - z3) / b0 */
    ladrc->linear.u_sat = (ladrc->linear.u0 - ladrc->linear.z3) / ladrc->b0;
    
    /* 输出限幅 */
    ladrc->linear.u_sat = Value_Limit(ladrc->linear.u_sat, ladrc->OutMax, ladrc->OutMin);
}

/**
 * @description: LADRC核心参数初始化
 * @param {LADRC_STRUCT} *ladrc LADRC结构体指针
 * @return {void}
 */
void Ladrc_Init(LADRC_STRUCT *ladrc){
    /* 控制器参数设置 */
    ladrc->w0 = 4.0f * ladrc->wc;     // 观测器带宽
    
    /* 计算观测器系数 */
    ladrc->beta01 = 3.0f * ladrc->w0;
    ladrc->beta02 = 3.0f * ladrc->w0 * ladrc->w0;
    ladrc->beta03 = ladrc->w0 * ladrc->w0 * ladrc->w0;
    
    /* 计算控制器系数 */
    ladrc->Kp = ladrc->wc * ladrc->wc;
    ladrc->Kd = 2.0f * ladrc->wc;
    
    /* 默认输出限幅 */
    ladrc->OutMax = 2000.0f;
    ladrc->OutMin = -2000.0f;
    
    /* 初始化所有运行变量为零 */
    ladrc->linear.v1 = 0.0f;
    ladrc->linear.v2 = 0.0f;
    ladrc->linear.z1 = 0.0f;
    ladrc->linear.z2 = 0.0f;
    ladrc->linear.z3 = 0.0f;
    ladrc->linear.Ref = 0.0f;
    ladrc->linear.Fbk = 0.0f;
    ladrc->linear.u_sat = 0.0f;
    ladrc->linear.e = 0.0f;
    ladrc->linear.e1 = 0.0f;
    ladrc->linear.e2 = 0.0f;
    ladrc->linear.u0 = 0.0f;
    ladrc->linear.fh = 0.0f;
}

/**
 * @description: LADRC主循环函数(定时器中断中调用)
 * @param {LADRC_STRUCT} *ladrc LADRC结构体指针
 * @return {void}
 */
void Ladrc_Loop(LADRC_STRUCT *ladrc){
    /* 跟踪微分器 - 安排过渡过程 */
    Ladrc_LTD(ladrc);
    
    /* 扩张状态观测器 - 估计状态和扰动 */
    Ladrc_LESO(ladrc);
    
    /* 线性控制率 - 计算控制量 */
    Ladrc_LinearControlRate(ladrc);
}


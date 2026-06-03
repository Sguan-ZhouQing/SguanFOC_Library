/*
 * @Author: 星必尘Sguan
 * @GitHub: https://github.com/Sguan-ZhouQing
 * @Date: 2026-02-16  00:27:53
 * @LastEditors: 星必尘Sguan|3464647102@qq.com
 * @LastEditTime: 2026-04-22 15:25:03
 * @FilePath: \SguanFOC_Debug\SguanFOC\Sguan_Ladrc.c
 * @Description: SguanFOC库的“二阶线性自适应抗干扰控制(LADRC)算法”实现
 * 
 * Copyright (c) 2026 by 星必尘Sguan, All Rights Reserved. 
 */
#include "Sguan_Ladrc.h"

// 静态函数声明->LADRC核心算法的三个步骤
static void Ladrc_LTD(LADRC_STRUCT *ladrc);
static void Ladrc_LESO(LADRC_STRUCT *ladrc);
static void Ladrc_LinearControlRate(LADRC_STRUCT *ladrc);

// 线性跟踪微分器(LTD)->安排过渡过程
static void Ladrc_LTD(LADRC_STRUCT *ladrc){
    // 1.最速控制综合函数 fh = -r^2*(v1-Ref) - 2*r*v2
    ladrc->run.fh = -ladrc->r*ladrc->r*(ladrc->run.v1 - ladrc->run.Ref) 
                    -2.0f*ladrc->r*ladrc->run.v2;
    
    // 2.离散化更新 v2 = v2 + h*fh, v1 = v1 + h*v2
    ladrc->run.v2 += ladrc->run.I_num*(ladrc->run.fh + ladrc->run.v2_i);
    ladrc->run.v1 += ladrc->run.I_num*(ladrc->run.v2 + ladrc->run.v1_i);

    // 3.更新历史输入值
    ladrc->run.v1_i = ladrc->run.v2;
    ladrc->run.v2_i = ladrc->run.fh;
}

// 线性扩张状态观测器(LESO)->估计系统状态和总扰动
static void Ladrc_LESO(LADRC_STRUCT *ladrc){
    // 1.观测器误差 e = z1 - Fbk
    ladrc->run.e = ladrc->run.z1 - ladrc->run.Fbk;
    
    /* 2.扩张状态观测器离散方程
     * z3 += (-beta03*e) * h
     * z2 += (z3 - beta02*e + b0*u_sat) * h
     * z1 += (z2 - beta01*e) * h
     */
    float temp_3 = -ladrc->data.beta3*ladrc->run.e;
    float temp_2 = ladrc->run.z3 - 
                ladrc->data.beta2*ladrc->run.e + 
                ladrc->b0*ladrc->run.Output;
    float temp_1 = ladrc->run.z2 - 
                ladrc->data.beta1*ladrc->run.e;

    ladrc->run.z3 += ladrc->run.I_num*(temp_3 + ladrc->run.z3_i);
    ladrc->run.z2 += ladrc->run.I_num*(temp_2 + ladrc->run.z2_i);
    ladrc->run.z1 += ladrc->run.I_num*(temp_1 + ladrc->run.z1_i);

    // 3.更新历史输入值
    ladrc->run.z3_i = temp_3;
    ladrc->run.z2_i = temp_2;
    ladrc->run.z1_i = temp_1;
}

// 线性控制率(LSEF/LF)->计算控制量
static void Ladrc_LinearControlRate(LADRC_STRUCT *ladrc){
    // 1.误差计算 e1 = v1 - z1, e2 = v2 - z2
    ladrc->run.e1 = ladrc->run.v1 - ladrc->run.z1;
    ladrc->run.e2 = ladrc->run.v2 - ladrc->run.z2;
    
    // 2.线性反馈控制率 u0 = Kp*e1 + Kd*e2
    ladrc->run.u0 = ladrc->data.Kp*ladrc->run.e1 + 
                        ladrc->data.Kd*ladrc->run.e2;
    
    // 3.扰动补偿 u = (u0 - z3) / b0
    ladrc->run.Output = (ladrc->run.u0 - ladrc->run.z3)/ladrc->b0;
}

/**
 * @description: LADRC核心参数初始化
 * @reminder: (初始化相关系数float->double->float)
 * @reminder: (单浮点转double运算，提高系数精度)
 * @param {LADRC_STRUCT} *ladrc
 * @return {*}
 */
void Ladrc_Init(LADRC_STRUCT *ladrc){
    // 1.控制器参数设置
    ladrc->data.w0 = (float)(4.0*((double)ladrc->Wc));
    ladrc->run.I_num = (float)(((double)ladrc->T)/2.0);
    
    // 2.计算观测器系数和控制器系数
    ladrc->data.beta1 = (float)(3.0*((double)ladrc->data.w0));
    ladrc->data.beta2 = (float)(3.0*((double)ladrc->data.w0)*
                    ((double)ladrc->data.w0));
    ladrc->data.beta3 = (float)(((double)ladrc->data.w0)*
                    ((double)ladrc->data.w0)*
                    ((double)ladrc->data.w0));

    ladrc->data.Kp = (float)(((double)ladrc->Wc)*
                    ((double)ladrc->Wc));
    ladrc->data.Kd = (float)(2.0*((double)ladrc->Wc));
    
    // 3.初始化令所有运行变量为零
    ladrc->run.v1 = 0.0f;
    ladrc->run.v2 = 0.0f;
    ladrc->run.z1 = 0.0f;
    ladrc->run.z2 = 0.0f;
    ladrc->run.z3 = 0.0f;
    ladrc->run.e = 0.0f;
    ladrc->run.e1 = 0.0f;
    ladrc->run.e2 = 0.0f;
    ladrc->run.u0 = 0.0f;
    ladrc->run.fh = 0.0f;

    ladrc->run.v1_i = 0.0f;
    ladrc->run.v2_i = 0.0f;
    ladrc->run.z1_i = 0.0f;
    ladrc->run.z2_i = 0.0f;
    ladrc->run.z3_i = 0.0f;

    ladrc->run.Ref = 0.0f;
    ladrc->run.Fbk = 0.0f;
    ladrc->run.Output = 0.0f;
}

/**
 * @description: LADRC离散周期运行函数
 * @reminder: https://github.com/Sguan-ZhouQing/SguanFOC_Library/blob/main/%E9%85%8D%E5%A5%97Simulink%E6%A8%A1%E5%9E%8B%E5%BC%80%E6%BA%90%E2%91%A1%5B%E7%AE%97%E6%B3%95%E5%8E%9F%E7%90%86%E5%9B%BE%5D/Sguan_Ladrc.png
 * @reminder: (上方链接是此Sguan_Ladrc模块Simulink原理仿真图)
 * @param {LADRC_STRUCT} *ladrc
 * @return {*}
 */
void Ladrc_Loop(LADRC_STRUCT *ladrc){
    // 1.跟踪微分器_安排过渡过程
    Ladrc_LTD(ladrc);
    
    // 2.扩张状态观测器_估计状态和扰动
    Ladrc_LESO(ladrc);
    
    // 3.线性控制率_计算控制量
    Ladrc_LinearControlRate(ladrc);

    // 4.输出限幅
    Value_Limit(&ladrc->run.Output, 
                ladrc->OutMax, 
                ladrc->OutMin);
}


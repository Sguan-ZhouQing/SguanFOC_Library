/*
 * @Author: 星必尘Sguan
 * @GitHub: https://github.com/Sguan-ZhouQing
 * @Date: 2026-05-23 16:43:32
 * @LastEditors: 星必尘Sguan|3464647102@qq.com
 * @LastEditTime: 2026-05-23 18:34:41
 * @FilePath: \SguanFOC_Debug\SguanFOC\Sguan_NLFO.c
 * @Description: SguanFOC库的“NLFO非线性磁链观测器”实现
 * 
 * Copyright (c) 2026 by $星必尘Sguan, All Rights Reserved. 
 */
#include "Sguan_NLFO.h"

/**
 * @description: NLFO非线性磁链观测器的参数初始化
 * @reminder: (初始化相关系数float->double->float)
 * @reminder: (单浮点转double运算，提高系数精度)
 * @param {NLFO_STRUCT} *nlfo
 * @return {*}
 */
void NLFO_Init(NLFO_STRUCT *nlfo){
    // 1.自动计算好需要的参数数值
    nlfo->go.I_num = (float)(((double)nlfo->T)/2.0);
    
    nlfo->go.Flux_pow = (float)(((double)nlfo->Flux)*((double)nlfo->Flux));
    nlfo->go.Flux_inv = (float)(1.0/((double)nlfo->Flux));
    nlfo->go.Ls = (float)(((double)nlfo->Ld)+((double)(nlfo->Lq))/2.0);

    // 2.初始化为零
    nlfo->go.alpha_i = 0.0f;
    nlfo->go.beta_i = 0.0f;
    nlfo->go.alpha_o = 0.0f;
    nlfo->go.beta_o = 0.0f;
    nlfo->go.Input_Ialpha = 0.0f;
    nlfo->go.Input_Ibeta = 0.0f;
    nlfo->go.Input_Ualpha = 0.0f;
    nlfo->go.Input_Ubeta = 0.0f;

    nlfo->go.Output_Sine = 1.0f;
    nlfo->go.Output_Cosine = 0.0f;
}

/**
 * @description: 非线性磁链观测器的离散运行函数
 * @reminder: https://github.com/Sguan-ZhouQing/SguanFOC_Library/blob/main/%E9%85%8D%E5%A5%97Simulink%E6%A8%A1%E5%9E%8B%E5%BC%80%E6%BA%90%E2%91%A1%5B%E7%AE%97%E6%B3%95%E5%8E%9F%E7%90%86%E5%9B%BE%5D/Sguan_NLFO.png
 * @reminder: (上方链接是此Sguan_NLFO模块Simulink原理仿真图)
 * @param {NLFO_STRUCT} *nlfo
 * @return {*}
 */
void NLFO_Loop(NLFO_STRUCT *nlfo){
    // 1.定义可使用到的全局变量
    float num0,num1,gain0,gain1,flux_error;

    // 2.运算非线性磁链观测器
    num0 = nlfo->go.Input_Ialpha*nlfo->go.Ls;
    num1 = nlfo->go.Input_Ibeta*nlfo->go.Ls;

    gain0 = nlfo->go.alpha_o - num0;
    gain1 = nlfo->go.beta_o - num1;
    flux_error = nlfo->go.Flux_pow - 
                gain0*gain0 - 
                gain1*gain1;

    float alpha = nlfo->Gain*gain0*flux_error + 
                nlfo->go.Input_Ualpha - 
                nlfo->go.Input_Ialpha*nlfo->Rs;
    float beta = nlfo->Gain*gain1*flux_error + 
                nlfo->go.Input_Ubeta - 
                nlfo->go.Input_Ibeta*nlfo->Rs;

    // 3.计算积分器
    float Out0 = nlfo->go.I_num*(alpha + nlfo->go.alpha_i) + 
                nlfo->go.alpha_o;
    float Out1 = nlfo->go.I_num*(beta + nlfo->go.beta_i) + 
                nlfo->go.beta_o;

    // 4.更新历史输入和输出数值
    nlfo->go.alpha_i = alpha;
    nlfo->go.beta_i = beta;
    nlfo->go.alpha_o = Out0;
    nlfo->go.beta_o = Out1;

    // 5.输出角度信息sin和cos
    nlfo->go.Output_Cosine = (Out0 - num0)*nlfo->go.Flux_inv;
    nlfo->go.Output_Sine = (Out1 - num1)*nlfo->go.Flux_inv;
}


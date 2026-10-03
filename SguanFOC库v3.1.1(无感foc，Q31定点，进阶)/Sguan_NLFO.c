/*
 * @Author: 星必尘Sguan
 * @GitHub: https://github.com/Sguan-ZhouQing
 * @Date: 2026-05-23 16:43:32
 * @LastEditors: 星必尘Sguan|3464647102@qq.com
 * @LastEditTime: 2026-06-05 03:48:05
 * @FilePath: \SguanFOC_Debug\SguanFOC\Sguan_NLFO.c
 * @Description: SguanFOC库的“NLFO非线性磁链观测器”实现
 * 
 * Copyright (c) 2026 by $星必尘Sguan, All Rights Reserved. 
 */
#include "Sguan_NLFO.h"

// NLFO非线性磁链观测器的参数初始化
void NLFO_Init(NLFO_STRUCT *nlfo){
    // 1.自动计算好需要的参数数值
    nlfo->go.T = PMSM_RUN_T_q31;

    nlfo->go.Rs = iqmath_from_float(nlfo->Rs, BASE_Resistor);
    nlfo->go.Ls = iqmath_from_float(nlfo->Ls, BASE_Inductor);

    float Flux_pow = (float)(((double)nlfo->Flux)*((double)nlfo->Flux));
    float Flux_inv = (float)(1.0/((double)nlfo->Flux));
    nlfo->go.Flux_pow = iqmath_from_float(Flux_pow, (BASE_Flux*BASE_Flux));
    nlfo->go.Flux_inv = iqmath_from_float(Flux_inv, ((1.0f/BASE_Flux)*BASE_SUB));

    nlfo->go.Gain = iqmath_from_float(
        nlfo->Gain, 
        ((1.0f/(BASE_Time*BASE_Time*BASE_Time*BASE_Voltage*BASE_Voltage))*BASE_SUB));

    // 2.初始化为零
    nlfo->go.alpha_o = 0;
    nlfo->go.beta_o = 0;
    nlfo->go.Input_Ialpha = 0;
    nlfo->go.Input_Ibeta = 0;
    nlfo->go.Input_Ualpha = 0;
    nlfo->go.Input_Ubeta = 0;

    nlfo->go.Output_Sine = 0;
    nlfo->go.Output_Cosine = Q31_MAX;
}

// 非线性磁链观测器的离散运行函数
void NLFO_Loop(NLFO_STRUCT *nlfo){
    // 1.定义可使用到的全局变量
    Q31_t num0,num1,gain0,gain1,flux_error0,flux_error_end;

    // 2.运算非线性磁链观测器
    num0 = iqmath_mul(nlfo->go.Input_Ialpha, nlfo->go.Ls);
    num1 = iqmath_mul(nlfo->go.Input_Ibeta, nlfo->go.Ls);

    gain0 = iqmath_sub(nlfo->go.alpha_o, num0);
    gain1 = iqmath_sub(nlfo->go.beta_o, num1);
    flux_error0 = iqmath_add(
        iqmath_mul(gain0, gain0), 
        iqmath_mul(gain1, gain1));
    flux_error_end = iqmath_sub(
        nlfo->go.Flux_pow, 
        flux_error0);

    float alpha = nlfo->Gain*gain0*flux_error_end + 
                nlfo->go.Input_Ualpha - 
                nlfo->go.Input_Ialpha*nlfo->Rs;
    float beta = nlfo->Gain*gain1*flux_error_end + 
                nlfo->go.Input_Ubeta - 
                nlfo->go.Input_Ibeta*nlfo->Rs;

    Q31_t alpha_temp = iqmath_mul(
        iqmath_mul(nlfo->go.Gain, gain0), 
        flux_error_end);
    Q31_t alpha_0 = iqmath_shift5_left(alpha_temp);
    Q31_t alpha_1 = iqmath_mul(
        nlfo->go.Input_Ialpha, 
        nlfo->go.Rs);
    Q31_t alpha = iqmath_add(
        iqmath_sub(alpha_0, alpha_1), 
        nlfo->go.Input_Ualpha);

    Q31_t beta_temp = iqmath_mul(
        iqmath_mul(nlfo->go.Gain, gain1), 
        flux_error_end);
    Q31_t beta_0 = iqmath_shift5_left(beta_temp);
    Q31_t beta_1 = iqmath_mul(
        nlfo->go.Input_Ibeta, 
        nlfo->go.Rs);
    Q31_t beta = iqmath_add(
        iqmath_sub(beta_0, beta_1), 
        nlfo->go.Input_Ubeta);

    // 3.计算积分器
    nlfo->go.alpha_o = iqmath_add(
        iqmath_mul(nlfo->go.T, alpha), 
        nlfo->go.alpha_o);

    nlfo->go.beta_o = iqmath_add(
        iqmath_mul(nlfo->go.T, beta), 
        nlfo->go.beta_o);

    // 4.输出角度信息sin和cos
    Q31_t Cosine_temp = iqmath_mul(
        iqmath_sub(nlfo->go.alpha_o, num0), 
        nlfo->go.Flux_inv);
    Q31_t Sine_temp = iqmath_mul(
        iqmath_sub(nlfo->go.beta_o, num1), 
        nlfo->go.Flux_inv);

    nlfo->go.Output_Cosine = iqmath_shift5_left(Cosine_temp);
    nlfo->go.Output_Sine = iqmath_shift5_left(Sine_temp);
}



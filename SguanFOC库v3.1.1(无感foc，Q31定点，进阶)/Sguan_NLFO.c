/* ===================================================================
 * 文件:     Sguan_NLFO.c
 * 项目:     SguanFOC_v3.1.1 —— 开源 FOC 电机控制框架（无感FOC · Q31定点 · 进阶）
 * 链接:     https://github.com/Sguan-ZhouQing/SguanFOC_Library
 * 版权:     Copyright 2026 星必尘Sguan <3464647102@qq.com>
 * 许可证:   MIT License
 * SPDX:     SPDX-License-Identifier: MIT
 *
 * 许可说明:
 *   [EN] Permission is hereby granted, free of charge, to use, copy,
 *        modify, merge, publish, distribute, sublicense, and/or sell
 *        copies of this software, provided this notice is retained.
 *   [中文] 本软件依据 MIT 许可证授权：可免费使用、复制、修改、合并、
 *        出版、分发、再授权及销售，仅须保留本版权与许可声明。
 *   [DE] Hiermit wird jeder Person kostenlos erlaubt, diese Software zu
 *        nutzen, zu kopieren, zu veraendern, zu veroeffentlichen, zu
 *        vertreiben, zu unterlizenzieren und/oder zu verkaufen, sofern
 *        dieser Hinweis erhalten bleibt.
 *   [JA] 本ソフトウェアは MIT ライセンスに基づき、無償で使用・複製・
 *        改変・公開・配布・再許諾・販売が許可されます。ただし本表示を
 *        保持する必要があります。
 * 完整许可文本: 参见仓库根目录下的 LICENSE-MIT 文件。
 * =================================================================== */
#include "Sguan_NLFO.h"

// NLFO非线性磁链观测器的参数初始化
void NLFO_Init(NLFO_STRUCT *nlfo){
    // 1.自动计算好需要的参数数值
    nlfo->go.T = iqmath_from_float(nlfo->T, BASE_Time);

    nlfo->go.Rs = iqmath_from_float(nlfo->Rs, BASE_Resistor);
    nlfo->go.Ls = iqmath_from_float(nlfo->Ls, BASE_Inductor);

    float Flux_pow = (float)(((double)nlfo->Flux)*((double)nlfo->Flux));
    float Flux_inv = (float)(1.0/((double)nlfo->Flux));
    nlfo->go.Flux_pow = iqmath_from_float(Flux_pow, (BASE_Flux*BASE_Flux));
    nlfo->go.Flux_inv = iqmath_from_float(Flux_inv, ((1.0f/BASE_Flux)*64.0f));

    nlfo->go.Gain = iqmath_from_float(
        nlfo->Gain, 
        ((1.0f/(BASE_Time*BASE_Time*BASE_Time*BASE_Voltage*BASE_Voltage))*32.0f));

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

    Q31_t alpha_temp = iqmath_mul(
        iqmath_mul(nlfo->go.Gain, gain0), 
        flux_error_end);
    Q31_t alpha_0 = (Q31_t)((uint32_t)alpha_temp << 5);
    Q31_t alpha_1 = iqmath_mul(
        nlfo->go.Input_Ialpha, 
        nlfo->go.Rs);
    Q31_t alpha = iqmath_add(
        iqmath_sub(alpha_0, alpha_1), 
        nlfo->go.Input_Ualpha);

    Q31_t beta_temp = iqmath_mul(
        iqmath_mul(nlfo->go.Gain, gain1), 
        flux_error_end);
    Q31_t beta_0 = (Q31_t)((uint32_t)beta_temp << 5);
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
    nlfo->go.Output_Cosine = iqmath_mul(
        iqmath_sub(nlfo->go.alpha_o, num0), 
        nlfo->go.Flux_inv);
    nlfo->go.Output_Sine = iqmath_mul(
        iqmath_sub(nlfo->go.beta_o, num1), 
        nlfo->go.Flux_inv);
}


/* ===================================================================
 * 文件:     Sguan_NLFO.h
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
#ifndef __SGUAN_NLFO_H
#define __SGUAN_NLFO_H

/* SguanFOC配置文件声明 */
#include "Sguan_Config.h"

typedef struct{
    Q31_t alpha_o;          // (数据)alpha轴积分历史输出值
    Q31_t beta_o;           // (数据)beta轴积分历史输出值

    Q31_t Flux_pow;         // (中间量)电机磁链平方
    Q31_t Flux_inv;         // (中间量)电机磁链倒数

    Q31_t Input_Ialpha;     // (输入数据)alpha轴电流
    Q31_t Input_Ibeta;      // (输入数据)beta轴电流
    Q31_t Input_Ualpha;     // (输入数据)alpha轴电压
    Q31_t Input_Ubeta;      // (输入数据)beta轴电压
    Q31_t Output_Sine;      // (输出数据)Sine角度信息
    Q31_t Output_Cosine;    // (输出数据)Cosine角度信息

    Q31_t T;                // (Q31)离散周期
    Q31_t Rs;               // (Q31)电机相电阻
    Q31_t Ls;               // (Q31)电机平均电感

    Q31_t Gain;             // (Q31)磁链观测解调增益
}__NLFO_GO_STRUCT;

typedef struct{
    __NLFO_GO_STRUCT go;      // (结构体)非线性磁链运算数据

    float T;                // (系统时钟)离散周期
    
    float Flux;             // (电机参数)磁链
    float Rs;               // (电机参数)电机相电阻
    float Ls;               // (中间量)电机平均电感

    float Gain;             // (参数设计)磁链观测解调增益
}NLFO_STRUCT;

void NLFO_Init(NLFO_STRUCT *nlfo);
void NLFO_Loop(NLFO_STRUCT *nlfo);


#endif // SGUAN_NLFO_H

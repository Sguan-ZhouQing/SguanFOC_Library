/* ===================================================================
 * 文件:     Sguan_Filter.h
 * 项目:     SguanFOC_v3.1.0 —— 开源 FOC 电机控制框架（无感FOC · 浮点运算 · 进阶）
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
#ifndef __SGUAN_FILTER_H
#define __SGUAN_FILTER_H

/* SguanFOC配置文件声明 */
#include "Sguan_Config.h"

// =========================== LPF低通滤波器 ========================
typedef struct{
    float i[2];             // (数据)历史输入值
    float o[2];             // (数据)历史输出值

    float Input;            // (输入数据)Input输入
    float Output;           // (输出数据)Output输出

    float num[2];           // (中间量)传递函数分子系数
    float den[2];           // (中间量)传递函数分母系数
}LPF_GO_STRUCT;

typedef struct{
    LPF_GO_STRUCT filter;   // (结构体)滤波器运算数据

    float T;                // (系统时钟)T离散周期
    float Wc;               // (参数设计)Wc截止频率
}LPF_STRUCT;

void LPF_Init(LPF_STRUCT *lpf);
void LPF_Loop(LPF_STRUCT *lpf);


// ======================== TPNF三参数陷波滤波器 =======================
typedef struct{
    float i[2];             // (数据)历史输入值
    float o[2];             // (数据)历史输出值

    float Input;            // (输入数据)Input输入
    float Output;           // (输出数据)Output输出

    float Gain0;            // (中间量)增益系数
    float Gain1;            // (中间量)增益系数
    float Gain2;            // (中间量)增益系数
}TPNF_GO_STRUCT;

typedef struct{
    TPNF_GO_STRUCT filter;  // (结构体)滤波器运算数据

    float T;                // (系统时钟)T离散周期

    float Wo;               // (参数设计)Wo陷波频率
    float K1;               // (参数设计)K1分母阻尼比
    float K2;               // (参数设计)K2分子阻尼比
}TPNF_STRUCT;

void TPNF_Init(TPNF_STRUCT *tpnf);
void TPNF_Loop(TPNF_STRUCT *tpnf);


#endif // SGUAN_FILTER_H

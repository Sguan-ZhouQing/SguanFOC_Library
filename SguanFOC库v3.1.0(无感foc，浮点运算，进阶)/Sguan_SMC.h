/* ===================================================================
 * 文件:     Sguan_SMC.h
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
#ifndef __SGUAN_SMC_H
#define __SGUAN_SMC_H

/* SguanFOC配置文件声明 */
#include "Sguan_Config.h"

typedef struct{
    float I_i;              // (数据)积分项历史输入
    float D_i;              // (数据)微分项历史输入

    float I_num;            // (中间量)积分的传递函数分子系数
    float D_num;            // (中间量)微分的传递函数分子系数
    float D_den;            // (中间量)微分的传递函数分母系数

    float Ref;              // (输入数据)Target期望数值
    float Fbk;              // (输出数据)Real真实反馈数据
    float Output;           // (输出数据)Output输出

    uint8_t IntegralFrozen_flag; // (中间量)积分抗饱和
}SMC_RUN_STRUCT;

typedef struct{
    SMC_RUN_STRUCT run;   // (结构体)传统滑模控制
    
    float T;                // (系统时钟)离散周期
    
    float miu;              // (参数设计)不连续控制增益
    float q;                // (参数设计)切换项增益
    float C;                // (参数设计)动态响应增益
    float Gain;             // (参数设计)电机输入增益

    float IntMax;           // (参数设计)积分项上限,即输出限幅
    float IntMin;           // (参数设计)积分项下限,即输出限幅
}SMC_STRUCT;

void SMC_Init(SMC_STRUCT *smc);
void SMC_Loop(SMC_STRUCT *smc);


#endif // SGUAN_SMC_H

/* ===================================================================
 * 文件:     Sguan_PLL.h
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
#ifndef __SGUAN_PLL_H
#define __SGUAN_PLL_H

/* SguanFOC配置文件声明 */
#include "Sguan_Config.h"

typedef struct{
    float We_i;             // (数据)数据历史输入值
    float Re_i;             // (数据)数据历史输入值
    float X_num[2];         // (中间量)PI传递函数分子系数
    float Y_num;            // (中间量)积分项分母系数

    float Error;            // (输入数据)Error真实反馈数据
    float OutWe;            // (输出数据)OutWe电角速度输出
    float OutRe;            // (输出数据)OutRe电角度输出
}PLL_GO_STRUCT;

typedef struct{
    PLL_GO_STRUCT go;       // (结构体)PID运算结构体

    float T;                // (系统时钟)T运算离散周期
    
    float Kp;               // (参数设计)Kp比例项增益
    float Ki;               // (参数设计)Ki积分项增益

    uint8_t is_position_mode; // (参数设计)位置环模式标志位
}PLL_STRUCT;

void PLL_Init(PLL_STRUCT *pll);
void PLL_Loop(PLL_STRUCT *pll);


#endif // SGUAN_PLL_H

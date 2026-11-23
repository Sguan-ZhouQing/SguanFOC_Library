/* ===================================================================
 * 文件:     Sguan_LTD.h
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
#ifndef __SGUAN_LTD_H
#define __SGUAN_LTD_H

/* 外部函数声明 */
#include "Sguan_Config.h"

typedef struct{
    float num;              // (中间量)传递函数的分子系数
    float den;              // (中间量)传递函数的分母系数
    float Last_i;           // (数据)历史的输入数值

    float Input;            // (输入数据)阶跃输入的数值
    float Output;           // (输出数据)输出平滑的数值
}LTD_GO_STRUCT;

typedef struct{
    LTD_GO_STRUCT go;       // (结构体)LTD运输数据

    float T;                // (系统时钟)T离散运行的时间周期
    float r;                // (参数设计)r速度因子_跟踪微分器
}LTD_STRUCT;

void LTD_Init(LTD_STRUCT *ltd);
void LTD_Loop(LTD_STRUCT *ltd);


#endif // SGUAN_LTD_H

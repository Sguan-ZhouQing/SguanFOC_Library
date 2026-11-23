/* ===================================================================
 * 文件:     Sguan_Hall.h
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
#ifndef __SGUAN_HALL_H
#define __SGUAN_HALL_H

/* SguanFOC配置文件声明 */
#include "Sguan_Config.h"

typedef struct{
    uint8_t Input_Ga;       // (输入数据)U相输入0或1
    uint8_t Input_Gb;       // (输入数据)V相输入0或1
    uint8_t Input_Gc;       // (输入数据)W相输入0或1
    float Output_Rad;       // (输出数据)输出不连续角度值

    float Hall_A;           // (中间量)三相历史数据
    float Hall_B;           // (中间量)三相历史数据
    float Hall_C;           // (中间量)三相历史数据

    float Gain;             // (中间量)低通滤波输入项系数
    float Normalized_Gain;  // (中间量)低通滤波输出项系数
}HALL_GO_STRUCT;

typedef struct{
    HALL_GO_STRUCT go;      // (结构体)霍尔传感器运算数据

    float T;                // (系统时钟)T离散运行周期

    float Wc;               // (参数设计)Wc微分环节一阶低通滤波
    float Hall_High;        // (参数设计)霍尔信号上边界
    float Hall_Low;         // (参数设计)霍尔信号下边界
}HALL_STRUCT;

void Hall_Init(HALL_STRUCT *hall);
void Hall_Loop(HALL_STRUCT *hall);


#endif // SGUAN_HALL_H

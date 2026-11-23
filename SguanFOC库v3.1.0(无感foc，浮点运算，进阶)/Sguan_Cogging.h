/* ===================================================================
 * 文件:     Sguan_Cogging.h
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
#ifndef __SGUAN_COGGING_H
#define __SGUAN_COGGING_H

/* SguanFOC配置文件声明 */
#include "Sguan_Config.h"

// 离线标定的1800点位抗齿槽电流表
#define Cogging_N   1800

typedef struct{
    float Input_Init_Iq;    // (输入数据)Init运行输入的Q轴电流
    float Output_Init_Rad;  // (输出数据)Init运行输出的电机位置

    float Input_Loop_Rad;   // (输入数据)Loop运行输入的电机位置
    float Output_Loop_Iq;   // (输出数据)Loop运行输出的抗齿槽电流
}COGGING_GO_STRUCT;

typedef struct{
    COGGING_GO_STRUCT go;   // (结构体)抗齿槽算法运输数据

    float T;                // (系统时钟)T离散运行的时间周期

    uint8_t Count;          // (参数设计)Count循环平均测的次数
    float Cycle;            // (参数设计)Cycle总周期->单位为秒
}COGGING_STRUCT;

void Cogging_SetIq_Init(COGGING_STRUCT *cogging);
void Cogging_ReadIq_Loop(COGGING_STRUCT *cogging);
int16_t Cogging_ReadIq_Tick(uint16_t K);


#endif // SGUAN_COGGING_H

/* ===================================================================
 * 文件:     Sguan_NSD.h
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
#ifndef __SGUAN_NSD_H
#define __SGUAN_NSD_H

/* 外部函数声明 */
#include "Sguan_Config.h"

typedef struct{
    float p_num;            // (中间量)带通滤波器->传递函数分子系数
    float p_den[2];         // (中间量)带通滤波器->传递函数分母系数
    float p_i[2];           // (数据)带通滤波器->历史输入值
    float p_o[2];           // (数据)带通滤波器->历史输出值

    float Input_D;          // (输入数据)电机D轴原始电流信号
    float Output_Ud;        // (输出数据)输出D轴的电压偏置
    uint8_t Output_Flag;    // (输出数据)输出D轴高频积分信号

    float Integration0;      // (数据)前向欧拉的积分值
    float Integration1;      // (数据)前向欧拉的积分值
    uint8_t Step;           // (数据)电机转子极性辨识标志位
    // (1245分别表示正偏置开始，正偏置结束，负偏置开始，负偏置结束)
    // (积分只会在Step为1开始正积分，然后在Step为3开始负积分)
}NSD_GO_STRUCT;

typedef struct{
    NSD_GO_STRUCT go;       // (结构体)NSD运输数据

    float T;                // (系统时钟)T离散运行的时间周期

    float Wo;               // (参数设计)Wo高频信号的角频率
    float zeta;             // (参数设计)zeta阻尼比
    float Ud;               // (参数设计)偏置D轴电压幅值
    float Cycle;            // (参数设计)辨识的总周期->单位为秒
}NSD_STRUCT;

void NSD_Init(NSD_STRUCT *nsd);
void NSD_SetUd_Init(NSD_STRUCT *nsd);
uint8_t NSD_ReadOffset_Loop(NSD_STRUCT *nsd);


#endif // SGUAN_NSD_H

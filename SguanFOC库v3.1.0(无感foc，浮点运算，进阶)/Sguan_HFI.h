/* ===================================================================
 * 文件:     Sguan_HFI.h
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
#ifndef __SGUAN_HFI_H
#define __SGUAN_HFI_H

/* SguanFOC配置文件声明 */
#include "Sguan_Config.h"

typedef struct{
    float Angle;            // (数据)注入的角度数值变化
    float Sine[2];             // (数据)历史注入的正弦值
    float High;             // (数据)仅经BPF滤波后的高频信号

    float Input_Id;         // (输入数据)原始D轴电流信号
    float Input_Iq;         // (输入数据)原始Q轴电流信号
    float Output_Id;        // (输出数据)基频D轴电流
    float Output_Iq;        // (输出数据)基频Q轴电流

    float Input_Q;          // (输入数据)原始Q轴电流信号
    float Output_Q;         // (输出数据)高频Q轴电流
    float Output_Uin;       // (输出数据)高频正弦电压D轴叠加量

    float s1_num[3];        // (中间量)陷波滤波器->传递函数分子系数
    float s1_den;           // (中间量)陷波滤波器->传递函数分母系数
    float s2_num[3];        // (中间量)陷波滤波器->传递函数分子系数
    float s2_den;           // (中间量)陷波滤波器->传递函数分母系数

    float p_num;            // (中间量)带通滤波器->传递函数分子系数
    float p_den[2];         // (中间量)带通滤波器->传递函数分母系数
}HFI_GO_STRUCT;

typedef struct{
    float x_i[2];           // (数据)x的滤波器->历史输入值
    float x_o[2];           // (数据)x的滤波器->历史输出值

    float y_i[2];           // (数据)y的滤波器->历史输入值
    float y_o[2];           // (数据)y的滤波器->历史输出值
}HFI_DATA_STRUCT;

typedef struct{
    HFI_GO_STRUCT go;       // (结构体)高频正弦波注入运算
    HFI_DATA_STRUCT data0;  // (结构体)运算中间量
    HFI_DATA_STRUCT data1;  // (结构体)运算中将量

    float T;                // (系统时钟)T离散周期

    float Wo;               // (参赛设计)注入正弦波频率
    float h;                // (参赛设计)高频解调增益
    float Uh;               // (参数设计)注入电压幅值

    float K1;               // (参数设计)三参数陷波滤波器->K1分母阻尼比
    float K2;               // (参数设计)三参数陷波滤波器->K2分子阻尼比
    float zeta;             // (参数设计)带通滤波越小越好->zeta阻尼比
}HFI_STRUCT;

void HFI_Init(HFI_STRUCT *hfi);
void HFI_ReadRad_Loop(HFI_STRUCT *hfi);
void HFI_Current_Loop(HFI_STRUCT *hfi);


#endif // SGUAN_HFI_H

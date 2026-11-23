/* ===================================================================
 * 文件:     Sguan_DOB.c
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
#ifndef __SGUAN_DOB_H
#define __SGUAN_DOB_H

/* SguanFOC配置文件声明 */
#include "Sguan_Config.h"

typedef struct{
    float I_num;            // (中间量)积分项传递函数分子系数
    float O_num;            // (中间量)输出项传递函数分子系数
    float O_den;            // (中间量)输出项传递函数分母系数

    float Fd_i;             // (数值)历史的Fd输入值
    float Wm_i;             // (数据)历史的Wm输入值

    float Gain0;            // (中间量)数据增益
    float Gain1;            // (中间量)数据增益

    float Input_Iq;         // (输入数据)输真实的Q轴电流
    float Input_Wm;         // (输入数据)真实的机械角速度
    float Output_Fd;        // (输出数据)预估扰动转矩
    float Output_Wm;        // (输出数据)预估的机械角速度
}SMDO_STRUCT;

typedef struct{
    SMDO_STRUCT smdo;       // (结构体)超螺旋滑模DOB
    
    float T;                // (系统时钟)离散周期
    
    uint8_t Pn;             // (电机参数)极对数
    float Flux;             // (电机参数)磁链
    float B;                // (电机参数)粘性阻尼
    float J;                // (电机参数)转动惯量
    
    float K1;               // (参数设计)比例项增益
    float K2;               // (参数设计)积分项增益

    float OutMax_Fd;        // (参数设计)输出上限限幅
    float OutMin_Fd;        // (参数设计)输出下限限幅
    float OutMax_Wm;        // (参数设计)输出下限限幅
    float OutMin_Wm;        // (参数设计)输出下限限幅
}DOB_STRUCT;

void DOB_Init(DOB_STRUCT *dob);
void DOB_Loop(DOB_STRUCT *dob);


#endif // SGUAN_DOB_H

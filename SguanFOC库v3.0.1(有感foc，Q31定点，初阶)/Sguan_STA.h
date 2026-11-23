/* ===================================================================
 * 文件:     Sguan_STA.h
 * 项目:     SguanFOC_v3.0.1 —— 开源 FOC 电机控制框架（有感FOC · Q31定点 · 初阶）
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
#ifndef __SGUAN_STA_H
#define __SGUAN_STA_H

/* SguanFOC配置文件声明 */
#include "Sguan_Config.h"

typedef struct{
    float s;                // (数据)滑模面当前值
    float integral;         // (数据)积分项累积值
    float output;           // (输出数据)控制器输出
    
    float Ref;              // (输入数据)期望输入值
    float Fbk;              // (输入数据)真实反馈值

    uint8_t IntegralFrozen_flag; // (中间量)积分抗饱和
}STA_RUN_STRUCT;

typedef struct{
    STA_RUN_STRUCT sta;     // (结构体)STA运算结构体
    
    float k1;               // (参数设计)非线性增益
    float k2;               // (参数设计)积分增益
    float T;                // (参数设计)积分步长_采样周期
    
    float boundary;         // (参数设计)边界层厚度(用于饱和函数)
    
    float OutMax;           // (参数设计)输出上限限幅
    float OutMin;           // (参数设计)输出下限限幅
    
    float IntMax;           // (参数设计)积分项上限限幅
    float IntMin;           // (参数设计)积分项下限限幅
}STA_STRUCT;


// ============================ Q31 版本代码 ============================

typedef struct{
    Q31_t s;                // (数据)滑模面当前值
    Q31_t integral;         // (数据)积分项累积值
    Q31_t output;           // (输出数据)控制器输出

    Q31_t Ref;              // (输入数据)期望输入值
    Q31_t Fbk;              // (输入数据)真实反馈值

    uint8_t IntegralFrozen_flag; // (中间量)积分抗饱和
    
    Q31_t k1;               // (参数设计)非线性增益
    Q31_t k2;               // (参数设计)积分增益
    Q31_t T;                // (参数设计)积分步长_采样周期

    Q31_t boundary;         // (中间量)边界层厚度(用于饱和函数)

    Q31_t OutMax;           // (数据)输出上限限幅
    Q31_t OutMin;           // (数据)输出下限限幅

    Q31_t IntMax;           // (数据)积分项上限
    Q31_t IntMin;           // (数据)积分项下限
}STA_RUN_STRUCT_q31;

typedef struct{
    STA_RUN_STRUCT_q31 sta; // (结构体)STA运算结构体
    
    float k1;               // (参数设计)非线性增益
    float k2;               // (参数设计)积分增益
    float T;                // (参数设计)积分步长_采样周期
    
    float boundary;         // (参数设计)边界层厚度(用于饱和函数)
    
    float OutMax;           // (参数设计)输出上限限幅
    float OutMin;           // (参数设计)输出下限限幅
    
    float IntMax;           // (参数设计)积分项上限限幅
    float IntMin;           // (参数设计)积分项下限限幅
}STA_STRUCT_q31;

void STA_Init(STA_STRUCT *sta);
void STA_Loop(STA_STRUCT *sta);

uint8_t STA_Init_q31(STA_STRUCT_q31 *sta);
void STA_Loop_q31(STA_STRUCT_q31 *sta);


#endif // SGUAN_STA_H

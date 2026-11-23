/* ===================================================================
 * 文件:     Sguan_Config.h
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
#ifndef __SGUAN_CONFIG_H
#define __SGUAN_CONFIG_H

#include "Sguan_IQmath.h"

// ============================ 系统配置 宏定义 ============================
#define CONFIG_PI           Open_PI_Control
#define CONFIG_MTPA         Open_MTPA_Calculate
#define CONFIG_FW           Open_FW_Calculate
#define CONFIG_Q31          Open_Q31_Calculate
#define CONFIG_Float        Open_float_Calculate
#define CONFIG_Debug        Open_Printf_Debug


// ============================ Q31标幺化 宏定义 ===========================
// 标幺化基准值(Config)
#define BASE_Time           Q_Time
#define BASE_Rad            Q_Rad
#define BASE_Current        Q_Current
#define BASE_Voltage        Q_Voltage
#define BASE_Speed          Q_Speed
#define BASE_Hz             Q_Hz
#define BASE_Inductor       Q_Inductor
#define BASE_Flux           Q_Flux
#define BASE_Resistor       Q_Resistor

// 默认数值缩放倍率(Sguan_Fiter)
#define BASE_Filter_Num     16.0f
#define SHIFT_Filter_Num    4

// 默认数值缩放倍率(Sguan_PLL)
#define BASE_PLL_Num        16.0f
#define INT32_PLL_Num_16    134217728
#define SHIFT_PLL_Num       4
// #define BASE_Num         16.0f
// #define INT32_Num_16     IQmath_Q31_from_float(1.0f,BASE_Num)
// #define SHIFT_Num        4


// ======================== 控制系统离散周期 宏定义 =========================
// 离散控制周期大小
#define PMSM_RUN_T          TIM_T


#endif // SGUAN_CONFIG_H

/* ===================================================================
 * 文件:     Sguan_Config.h
 * 项目:     SguanFOC_v3.1.1 —— 开源 FOC 电机控制框架（无感FOC · Q31定点 · 进阶）
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

/* SguanFOC配置文件声明 */
#include "Sguan_IQmath.h"

// ============================ 系统配置 宏定义 ============================
#define CONFIG_MODE         Define_Run_Mode
#define CONFIG_CUR          Current_Offset_Open
#define CONFIG_Float        Q31_Float_Open
#define CONFIG_Observation  Read_Observation_Open

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

// ======================== 控制系统离散周期 宏定义 =========================
// 离散控制周期大小
// #define PMSM_RUN_T_q31   iqmath_from_float(TIM_T,BASE_Time)
// #define Dead_T_q31       iqmath_from_float(Dead_Time,BASE_Time)
// #define ADC_T_q31        iqmath_from_float(ADC_Time,BASE_Time)
// (这些Q31的数值定义，都放在了Sguan_Value.h中)
#define PMSM_RUN_T          TIM_T



#endif // SGUAN_CONFIG_H

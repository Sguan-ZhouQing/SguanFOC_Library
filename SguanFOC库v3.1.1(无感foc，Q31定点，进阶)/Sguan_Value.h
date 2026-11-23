/* ===================================================================
 * 文件:     Sguan_Value.h
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
#ifndef __SGUAN_VALUE_H
#define __SGUAN_VALUE_H

/* SguanFOC配置文件声明 */
#include <stdint.h>
#include "UserData_Config.h"

// 浮点数
#define Value_2PI           6.283185307179586f      // 2pi的数值
#define Value_PI_2          1.570796326794896f      // 二分之pi
#define Value_2_SQRT2       2.8284271247461903f     // 二倍根号二

// Q31三角函数计算中间值(8.0基值，Rad)
#define SIN_K_q31           652         // round(8.0 * 512 / (2π))

// Q31定点数(8.0基值，Rad)
// .............................................................
// #define Value_2PI_q31    iqmath_from_float(Value_2PI, Q_Rad)
// #define Value_PI_2_q31   iqmath_from_float(Value_PI_2, Q_Rad)
#define Value_2PI_q31       1686629713
#define Value_PI_2_q31      421657428
// .............................................................
// 离散控制周期大小
// #define PMSM_RUN_T_q31   iqmath_from_float(TIM_T,BASE_Time)
// #define Dead_T_q31       iqmath_from_float(Dead_Time,BASE_Time)
// #define ADC_T_q31        iqmath_from_float(ADC_Time,BASE_Time)
// (5K)
// #define PMSM_RUN_T_q31   439804651
// (4.5K)
// #define PMSM_RUN_T_q31   274877907
// (4K)
// #define PMSM_RUN_T_q31   274877907
// (3.5K)
#define PMSM_RUN_T_q31      314146179
// (2K)
// #define PMSM_RUN_T_q31   549755814
// ....
// Current_Gain_q31 = (Current_Gain/Q_Current)*2^62 / Current_RAW_SCALE
// Current_Gain = 0.0081382 (5V基准，0.015R采样电阻，4096采样精度，10倍)
// Q_Current        = 64.0
// 2^62             = 4611686018427387904
#define Current_RAW_SCALE   500000
// #define Current_Gain_q31 1172806000
#define Current_Gain_q31    1776978798


#endif // SGUAN_VALUE_H

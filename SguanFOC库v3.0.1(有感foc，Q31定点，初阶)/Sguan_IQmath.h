/* ===================================================================
 * 文件:     Sguan_IQmath.h
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
#ifndef __SGUAN_IQMATH_H
#define __SGUAN_IQMATH_H

/* 外部函数声明 */
#include <stdint.h>
#include <stdio.h>

// 外部标幺化基准值引入(非解耦量,解耦需自定义Q_Rad值)
#include "UserData_Config.h"

// ===================== Q17.15 常量定义 =====================
#define Q15_INT_MAX      65535                 // 16位整数部分最大值（无符号）
#define Q15_INT_MIN      -65536                // 16位整数部分最小值（含符号）
// ===================== Q31 常量定义 =====================
#define Q31_MAX         0x7FFFFFFF      // 表示最大值0.9999999995
#define Q31_MIN         0x80000000      // 表示最小值-1.0
#define Q31_HALF        0x40000000      // 表示0.5

// 定点化计算格式
typedef int32_t Q15_t;      // Q17.15格式：1位符号+16位整数+15位小数
typedef int32_t Q31_t;      // Q1.31格式：1位符号+0位整数+31位小数

// 常量宏定义
#define Value_PI            3.141592653589793f
#define Value_2PI           6.283185307179586f
#define Value_512_2PI       81.487330863050417f


// ============================ float 版本代码 ============================

// 重写C标准库
float Value_fabsf(float x);
int Value_isinf(float x);
int Value_isnan(float x);
float Value_sqrtf(float x);

// 参数限制函数
float Value_Limit(float val, float max, float min);
float Value_normalize(float angle);

// 快速正余弦求解float版本
#define fast_cos(x) fast_sin(1.5707963f - x);
float fast_sin(float x);
void fast_sin_cos(float x, float *sin_x, float *cos_x);


// ============================ Q31 版本代码 =============================

// Q31版本C标准库及参数限制函数
Q31_t Value_ads_q31(Q31_t x);
Q31_t Value_sqrt_q31(Q31_t input);
Q31_t Value_Limit_q31(Q31_t val, Q31_t max, Q31_t min);
Q31_t Value_normalize_q31(Q31_t angle_q31);

// IQ15的定点化公式计算
Q15_t IQmath_float_to_Q15(float f);
float IQmath_Q15_to_float(Q15_t q);
Q15_t IQmath_Q15_add(Q15_t a, Q15_t b);
Q15_t IQmath_Q15_sub(Q15_t a, Q15_t b);
Q15_t IQmath_Q15_mul(Q15_t a, Q15_t b);
Q15_t IQmath_Q15_div(Q15_t a, Q15_t b);

// Q31的定点化公式计算
Q31_t IQmath_Q31_Limit(Q31_t value);
Q31_t IQmath_Q31_from_float(float f, float base_value);
float IQmath_Q31_to_float(Q31_t q, float base_value);
Q31_t IQmath_Q31_mul(Q31_t a, Q31_t b);
Q31_t IQmath_Q31_div(Q31_t a, Q31_t b);
Q31_t IQmath_Q31_add(Q31_t a, Q31_t b);
Q31_t IQmath_Q31_sub(Q31_t a, Q31_t b);
Q31_t IQmath_Q31_convert_base(Q31_t q, float src_base, float dst_base);

#define Value_PI_q31 IQmath_Q31_from_float(Value_PI,Q_Rad)
#define Value_2PI_q31 IQmath_Q31_from_float(Value_2PI,Q_Rad)
#define Value_1_Rad_q31 IQmath_Q31_from_float(1.0f,1.0f/Q_Rad)

#define IQmath_Q31_Shift_Right(x, n)    ((Q31_t)((int64_t)(x) >> (n)))
#define IQmath_Q31_Shift_Left(x, n)     ((Q31_t)((int64_t)(x) << (n)))

// 快速正余弦求解Q31版本
#define fast_cos_q31(x) fast_sin_q31(1.5707963f - x);
Q31_t fast_sin_q31(float x);
void fast_sin_cos_q31(float x, Q31_t *sin_x, Q31_t *cos_x);


// ============================ 定点初始化 协议层 ============================

// 读写uint8_t寄存器数值
uint8_t IQmath_ReadBit(uint8_t reg, uint8_t n);
void IQmath_SetBit(uint8_t *reg, uint8_t n, uint8_t Bit);

/* 
1.协议(Sguan.IQmath_Error)
#define Error_Filter_Current    0x01 // 0.Filter“电流滤波”定点初始化失败
#define Error_Filter_Encoder    0x02 // 1.Filter“编码器滤波”定点初始化失败
#define Error_PID_Current       0x04 // 2.PID“电流环”定点初始化失败
#define Error_PID_Velocity      0x08 // 3.PID“速度环”定点初始化失败
#define Error_STA_Speed         0x10 // 4.STA“速度环”定点初始化失败
#define Error_PID_Position      0x20 // 5.PID“位置环”定点初始化失败
#define Error_PLL_Encoder       0x40 // 6.PLL“位置环”定点初始化失败
*/




#endif // SGUAN_IQMATH_H

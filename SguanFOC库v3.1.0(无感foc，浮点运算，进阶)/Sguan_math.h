/* ===================================================================
 * 文件:     Sguan_math.h
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
#ifndef __SGUAN_MATH_H
#define __SGUAN_MATH_H

/* 外部函数声明 */
#include <stdint.h>
#include <stdio.h>

// 常量宏定义
#define Value_PI            3.141592653589793f      // 圆周率数值
#define Value_PI_2          1.570796326794896f      // 二分之一pi
#define Value_2PI           6.283185307179586f      // 2pi的数值
#define Value_INV_SQRT3     0.5773502691896257f     // 根号三分之壹
#define Value_SQRT3         1.7320508075688772f     // 根号三
#define Value_SQRT3_2       0.8660254037844386f     // 二分之根号三
#define Value_2_SQRT2       2.8284271247461903f     // 二倍根号二分
#define Value_INV_ln20      0.3338082006953340f     // ln(20)分之壹
#define Value_N_INF         0xFF800000              // (负无穷数)

// 重写C标准库Value...f
float Value_maxf(float a, float b);
float Value_minf(float a, float b);
float Value_fabsf(float x);
float Value_sqrtf(float x);

// 参数限制函数Value...x
void Value_Limit(float *val, float max, float min);
float Value_Gain_Get(float *gain, 
                float real_speed,
                float abs_max,
                float abs_min);
float Value_normalize(float angle);
void Value_Correct(float *angle, float error);
int8_t Value_set(int8_t val, int8_t max, int8_t min);
float Value_Sign(float value);

// 电机角度生成器和单位转换函数
void Value_Rad_Loop(float *Output, 
                float Input, 
                float Last_in, 
                float T);
float Value_Rad_from_Hz(float hz);
float Value_Rad_to_Hz(float rad_s);

// MOTOR公式变换
void clarke(float *i_alpha, 
        float *i_beta, 
        float i_a, 
        float i_b);
void park(float *i_d, 
        float *i_q, 
        float i_alpha, 
        float i_beta, 
        float sine, 
        float cosine);
void ipark(float *u_alpha, 
        float *u_beta, 
        float u_d, 
        float u_q, 
        float sine, 
        float cosine);

// 快速正余弦求解float版本
#define fast_cos(x) fast_sin(Value_PI_2 - x);
float fast_sin(float x);
void fast_sin_cos(float x, float *sin_x, float *cos_x);


#endif // SGUAN_MATH_H

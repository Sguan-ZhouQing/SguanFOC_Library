/* ===================================================================
 * 文件:     Sguan_IQmath.h
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
#ifndef __SGUAN_IQMATH_H
#define __SGUAN_IQMATH_H

/* SguanFOC配置文件声明 */
#include "Sguan_Value.h"

// 定点化计算格式
typedef int32_t Q31_t;      // 1位符号+0位整数+31位小数

// ===================== Q31 常量定义 =====================
#define Q31_MAX         0x7FFFFFFF      // 表示最大值0.9999999995
#define Q31_MIN         0x80000000      // 表示最小值-1.0
#define Q31_HALF        0x40000000      // 表示0.5(特殊场景会用到)

// Q的定点化公式计算
Q31_t iqmath_from_float(float f, float base_value);
float iqmath_to_float(Q31_t q, float base_value);
Q31_t iqmath_add(Q31_t a, Q31_t b);
Q31_t iqmath_sub(Q31_t a, Q31_t b);
Q31_t iqmath_mul(Q31_t a, Q31_t b);
Q31_t iqmath_div(Q31_t a, Q31_t b);
Q31_t iqmath_convert_base(Q31_t q, float old_base, float new_base);
// ============================================================
Q31_t iqmath_abs(Q31_t x);
void iqmath_limit(Q31_t *val, Q31_t max, Q31_t min);
void iqmath_u16_limit(uint16_t *val, uint16_t max, uint16_t min);
Q31_t iqmath_normalize(Q31_t angle);
Q31_t iqmath_current_raw_to_q31(int16_t raw);
void iqmath_rad_loop(Q31_t *Rad, Q31_t Speed, Q31_t T);
Q31_t iqmath_speed_div5_fast(Q31_t x);
// ============================================================
Q31_t fast_sin(Q31_t x);
Q31_t fast_cos(Q31_t x);
void fast_sin_cos(Q31_t x, Q31_t *sin_x, Q31_t *cos_x);



#endif // SGUAN_IQMATH_H

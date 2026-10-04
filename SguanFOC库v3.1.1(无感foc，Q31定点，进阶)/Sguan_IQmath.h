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
/* 左移 5 位（×32）饱和判断阈值：base=32.0f → base=1.0f 专用 */
#define Q31_MAX_SHIFT5  0x03FFFFFF      /*  67108863  = Q31_MAX >> 5 */
#define Q31_MIN_SHIFT5  0xFC000000      /* -67108864  = Q31_MIN >> 5 */

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
Q31_t iqmath_normalize(Q31_t angle);
Q31_t iqmath_current_raw_to_q31(uint16_t raw);
void iqmath_rad_loop(Q31_t *Rad, Q31_t Speed, Q31_t T);
Q31_t iqmath_speed_div5_fast(Q31_t x);
Q31_t iqmath_shift5_left(Q31_t q);
// ============================================================
Q31_t fast_sin(Q31_t x);
Q31_t fast_cos(Q31_t x);
void fast_sin_cos(Q31_t x, Q31_t *sin_x, Q31_t *cos_x);



#endif // SGUAN_IQMATH_H

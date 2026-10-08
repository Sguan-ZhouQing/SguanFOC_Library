#ifndef __SGUAN_IQMATH_H
#define __SGUAN_IQMATH_H

/* SguanFOC配置文件声明 */
#include "Sguan_Value.h"

// ===================== Q31 常量定义 =====================
#define Q31_MAX         0x7FFFFFFF      // 表示最大值0.9999999995
#define Q31_MIN         0x80000000      // 表示最小值-1.0
#define Q31_HALF        0x40000000      // 表示0.5(特殊场景会用到)
// ===================== Q15 常量定义 =====================
#define Q15_MAX         0x7FFF          // 表示最大值0.9999694824
#define Q15_MIN         0x8000          // 表示最小值-1.0
#define Q15_HALF        0x4000          // 表示0.5(特殊场景会用到)

// Q的定点化公式计算
SguanQ iqmath_from_float(SguanF f, SguanF base_value);
SguanF iqmath_to_float(SguanQ q, SguanF base_value);
SguanQ iqmath_add(SguanQ a, SguanQ b);
SguanQ iqmath_sub(SguanQ a, SguanQ b);
SguanQ iqmath_mul(SguanQ a, SguanQ b);
SguanQ iqmath_div(SguanQ a, SguanQ b);
SguanQ iqmath_convert_base(SguanQ q, SguanF old_base, SguanF new_base);
SguanQ iqmath_abs(SguanQ x);
SguanQ iqmath_zero(void);

#endif // SGUAN_IQMATH_H

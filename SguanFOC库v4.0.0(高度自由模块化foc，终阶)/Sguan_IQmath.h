#ifndef __SGUAN_IQMATH_H
#define __SGUAN_IQMATH_H

/* SguanFOC配置文件声明 */
#include "Sguan_Value.h"
#include "UserData_Config.h"


#define CONFIG_IQMATH       DATA_DEFINE_IQMATH

// 定点化计算格式
typedef int32_t Q31;      // 1位符号+0位整数+31位小数
typedef int16_t Q15;      // 1位符号+0位整数+15位小数

#if DATA_DEFINE_IQMATH==0x01
typedef Q31                 SguanQ;
#elif DATA_DEFINE_IQMATH==0x02
typedef Q15                 SguanQ;
#else // CONFIG_IQMATH
typedef float               SguanQ;
#endif // CONFIG_IQMATH

// ===================== Q31 常量定义 =====================
#define Q31_MAX         0x7FFFFFFF      // 表示最大值0.9999999995
#define Q31_MIN         0x80000000      // 表示最小值-1.0
#define Q31_HALF        0x40000000      // 表示0.5(特殊场景会用到)
// ===================== Q15 常量定义 =====================
#define Q15_MAX         0x7FFF          // 表示最大值0.9999694824
#define Q15_MIN         0x8000          // 表示最小值-1.0
#define Q15_HALF        0x4000          // 表示0.5(特殊场景会用到)

// Q的定点化公式计算
SguanQ IQmath_Q_from_float(float f, float base_value);
float IQmath_Q_to_float(SguanQ q, float base_value);
SguanQ IQmath_Q_add(SguanQ a, SguanQ b);
SguanQ IQmath_Q_sub(SguanQ a, SguanQ b);
SguanQ IQmath_Q_mul(SguanQ a, SguanQ b);
SguanQ IQmath_Q_div(SguanQ a, SguanQ b);
SguanQ IQmath_Q_convert_base(SguanQ q, float old_base, float new_base);


#endif // SGUAN_IQMATH_H

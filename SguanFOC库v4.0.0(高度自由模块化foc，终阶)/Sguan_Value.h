#ifndef ___SGUAN_VALUE_H
#define __SGUAN_VALUE_H

/* SguanFOC配置文件声明 */
#include <stdint.h>
#include "UnitLib_Config.h"
#include "UserData_Config.h"

// 定点化计算格式
typedef int32_t Q31;        // 1位符号+0位整数+31位小数
typedef int16_t Q15;        // 1位符号+0位整数+15位小数

// 数据格式宏定义
#define VALUE_MATH          UNITLIB_CONFIG_MATH
#define VALUE_FLOAT         0x00
#define VALUE_DOUBLE        0x01
#define VALUE_Q15           0x02
#define VALUE_Q31           0x03

// Value数值分类(若是定点，Rad例程代码按照8.0归一化)
// (SguanF必然是浮点数)
// (只出现在Sguan_IQmath.c中)
// (SguanQ看格式，一般情况是浮点数，定点代码是整数)
// (全代码都出现)
#if VALUE_MATH==VALUE_DOUBLE
typedef double              SguanQ;
typedef double              SguanF;
#define VALUE_2PI           6.283185307179586f      // 2pi的数值
#define VALUE_PI_2          1.570796326794896f      // 二分之pi
#define VALUE_2_SQRT2       2.8284271247461903f     // 二倍根号二
#elif VALUE_MATH==VALUE_Q15
typedef Q15                 SguanQ;
typedef float               SguanF;
#define VALUE_2PI           6.283185307179586f      // 2pi的数值
#define VALUE_PI_2          1.570796326794896f      // 二分之pi
#define VALUE_2_SQRT2       2.8284271247461903f     // 二倍根号二
#elif VALUE_MATH==VALUE_Q31
typedef Q31                 SguanQ;
typedef float               SguanF;
#define VALUE_2PI           6.283185307179586f      // 2pi的数值
#define VALUE_PI_2          1.570796326794896f      // 二分之pi
#define VALUE_2_SQRT2       2.8284271247461903f     // 二倍根号二
#else // CONFIG_IQMATH->VALUE_FLOAT
typedef float               SguanQ;
typedef float               SguanF;
#define VALUE_2PI           6.283185307179586f      // 2pi的数值
#define VALUE_PI_2          1.570796326794896f      // 二分之pi
#define VALUE_2_SQRT2       2.8284271247461903f     // 二倍根号二
#endif // CONFIG_IQMATH


#endif // SGUAN_VALUE_H

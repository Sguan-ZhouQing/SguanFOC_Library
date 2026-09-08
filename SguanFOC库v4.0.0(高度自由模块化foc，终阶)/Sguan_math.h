#ifndef __SGUAN_MATH_H
#define __SGUAN_MATH_H

#include <stdint.h>

#define MATH_Value_2PI           6.283185307179586f      // 2pi的数值
#define MATH_Value_PI_2          1.570796326794896f      // 二分之pi
#define MATH_Value_2_SQRT2       2.8284271247461903f     // 二倍根号二


// 快速正余弦求解float版本
float Math_sin(float x);
#define Math_cos(x) Math_sin(MATH_Value_PI_2 - x);
void Math_sin_cos(float x, float *sin_x, float *cos_x);
float Math_tan(float x);
float Math_atan(float x);



#endif // SGUAN_MATH_H

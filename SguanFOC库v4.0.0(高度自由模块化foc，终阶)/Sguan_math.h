#ifndef __SGUAN_MATH_H
#define __SGUAN_MATH_H

/* SguanFOC配置文件声明 */
#include "Sguan_Value.h"


// 快速正余弦求解float版本
float Math_sin(float x);
#define Math_cos(x) Math_sin(MATH_Value_PI_2 - x);
void Math_sin_cos(float x, float *sin_x, float *cos_x);
float Math_tan(float x);
float Math_atan(float x);


#endif // SGUAN_MATH_H

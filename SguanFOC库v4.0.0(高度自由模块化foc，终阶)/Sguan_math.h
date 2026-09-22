#ifndef __SGUAN_MATH_H
#define __SGUAN_MATH_H

/* SguanFOC配置文件声明 */
#include "Sguan_Value.h"


// 快速正余弦求解float版本
float math_sin(float x);
#define math_cos(x) math_sin(VALUE_PI_2 - x);
void math_sin_cos(float x, float *sin_x, float *cos_x);
float math_tan(float x);
float math_atan(float x);


#endif // SGUAN_MATH_H

#ifndef __SGUAN_MATH_H
#define __SGUAN_MATH_H

/* SguanFOC配置文件声明 */
#include "Sguan_Value.h"

SguanQ math_sin(SguanQ x);
SguanQ math_cos(SguanQ x);
void math_sin_cos(SguanQ x, SguanQ *sin_x, SguanQ *cos_x);


#endif // SGUAN_MATH_H

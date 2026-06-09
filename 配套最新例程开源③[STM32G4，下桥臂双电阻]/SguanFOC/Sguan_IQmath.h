#ifndef __SGUAN_IQMATH_H
#define __SGUAN_IQMATH_H

/* 外部函数声明 */
#include <stdint.h>

#define Q15_t int16_t

Q15_t IQmath_Q15_from_float(float f);
float IQmath_Q15_to_float(Q15_t q);


#endif // SGUAN_IQMATH_H

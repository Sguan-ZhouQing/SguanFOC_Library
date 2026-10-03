#ifndef ___SGUAN_VALUE_H
#define __SGUAN_VALUE_H

/* SguanFOC配置文件声明 */
#include <stdint.h>
#include "UserData_Config.h"

// 浮点数
#define Value_2PI           6.283185307179586f      // 2pi的数值
#define Value_PI_2          1.570796326794896f      // 二分之pi
#define Value_2_SQRT2       2.8284271247461903f     // 二倍根号二

// Q31三角函数计算中间值(8.0基值，Rad)
#define SIN_K_q31           652         // round(8.0 * 512 / (2π))

// Q31定点数(8.0基值，Rad)
// .............................................................
// #define Value_2PI_q31    iqmath_from_float(Value_2PI, Q_Rad)
// #define Value_PI_2_q31   iqmath_from_float(Value_PI_2, Q_Rad)
#define Value_2PI_q31       1686629713
#define Value_PI_2_q31      421657428
// .............................................................
// 离散控制周期大小
// #define PMSM_RUN_T_q31   iqmath_from_float(TIM_T,BASE_Time)
// #define Dead_T_q31       iqmath_from_float(Dead_Time,BASE_Time)
// #define ADC_T_q31        iqmath_from_float(ADC_Time,BASE_Time)
#define PMSM_RUN_T_q31      54975581
#define Dead_T_q31          4617949
#define ADC_T_q31           549756
// ....
// Current_Gain_q31 = (Current_Gain/Q_Current)*2^62 / Current_RAW_SCALE
// Current_Gain = 0.0101725264
// Q_Current        = 64.0
// 2^62             = 4611686018427387904
#define Current_RAW_SCALE   500000
#define Current_Gain_q31    1465804000


#endif // SGUAN_VALUE_H

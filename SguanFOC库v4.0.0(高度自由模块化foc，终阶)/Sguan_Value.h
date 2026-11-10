#ifndef ___SGUAN_VALUE_H
#define __SGUAN_VALUE_H

/* SguanFOC配置文件声明 */
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "UnitLib_Define0.h"    // (应用库UnitLib)
#include "UserMotor_Define1.h"  // (电参库UserMotor)
#include "Zone_Define2.h"       // (用户驱动层Zone)

// ================================================================================
// 1.局部定点化计算格式
typedef int32_t Q31;            // 1位符号+0位整数+31位小数
typedef int16_t Q15;            // 1位符号+0位整数+15位小数

// ================================================================================
// 2.数据格式宏定义
#define VALUE_MODE_FLOAT        0x00
#define VALUE_MODE_DOUBLE       0x01
#define VALUE_MODE_Q15          0x02
#define VALUE_MODE_Q31          0x03

#ifndef DEFINE0_MOTOR_MATH
#define DEFINE0_MOTOR_MATH       VALUE_MODE_FLOAT
#endif

#if !((DEFINE0_MOTOR_MATH>=VALUE_MODE_FLOAT) && (DEFINE0_MOTOR_MATH<=VALUE_MODE_Q31))
#define VALUE_MODE_MATH         VALUE_MODE_FLOAT
#else // DEFINE0_MOTOR_MATH
#define VALUE_MODE_MATH         DEFINE0_MOTOR_MATH
#endif // DEFINE0_MOTOR_MATH

// ================================================================================
// 3.Value数值分类
// (SguanF必然是浮点数)
// (只出现在Sguan_IQmath.c和少数其他函数中)
// (SguanQ看格式，一般情况是浮点数，定点代码是整数)
// (全代码都出现)
#if VALUE_MODE_MATH==VALUE_MODE_DOUBLE
typedef double                  SguanQ;
typedef double                  SguanF;
#elif VALUE_MODE_MATH==VALUE_MODE_Q15
typedef Q15                     SguanQ;
typedef float                   SguanF;
#elif VALUE_MODE_MATH==VALUE_MODE_Q31
typedef Q31                     SguanQ;
typedef float                   SguanF;
#else // VALUE_MODE_MATH->VALUE_MODE_FLOAT
typedef float                   SguanQ;
typedef float                   SguanF;
#endif // CONFIG_IQMATH

#define SGUAN_Q(x) ((SguanQ)(x))
#define SGUAN_F(x) ((SguanF)(x))

// ================================================================================
// 4.常用数值宏定义
// (若是定点，Rad例程代码按照8.0归一化)
/* 以下定义分别是浮点的：
(2pi的数值) (pi的数值) (二分之pi)
(二倍根号二) (二分之根号三)
(数值壹) (数值半) (数值零)
定点Q15的数值：
(2pi的数值) (pi的数值) (二分之pi)
定点Q31的数值：
(2pi的数值) (pi的数值) (二分之pi)*/
#define VALUE_F_2PI             (SguanF)6.283185307179586477
#define VALUE_F_PI              (SguanF)3.141592653589793239
#define VALUE_F_PI_2            (SguanF)1.570796326794896619
#define VALUE_F_2_SQRT2         (SguanF)2.828427124746190098
#define VALUE_F_SQRT3           (SguanF)1.732050807568877293
#define VALUE_F_SQRT3_2         (SguanF)0.866025403784438646
#define VALUE_F_INV_SQRT3       (SguanF)0.577350269189625764
#define VALUE_F_ONE             (SguanF)1.0     // 数值壹
#define VALUE_F_HALF            (SguanF)0.5     // 数值半

#define VALUE_Q15_2PI           (Q15)25736      // BASE=8.0
#define VALUE_Q15_PI            (Q15)12868      // BASE=8.0
#define VALUE_Q15_PI_2          (Q15)6434       // BASE=8.0
#define VALUE_Q15_INV_SQRT3     (Q15)18919      // BASE=1.0
#define VALUE_Q15_SQRT3_2       (Q15)28378      // BASE=1.0
#define VALUE_Q15_MAX           0x7FFF          // 表示最大值0.9999694824
#define VALUE_Q15_MIN           0x8000          // 表示最小值-1.0
#define VALUE_Q15_HALF          0x4000          // 表示0.5(特殊场景会用到)

#define VALUE_Q31_2PI           (Q31)1686629713 // BASE=8.0
#define VALUE_Q31_PI            (Q31)843314857  // BASE=8.0
#define VALUE_Q31_PI_2          (Q31)421657428  // BASE=8.0
#define VALUE_Q31_INV_SQRT3     (Q31)1239850262 // BASE=1.0
#define VALUE_Q31_SQRT3_2       (Q31)1859775393 // BASE=1.0
#define VALUE_Q31_MAX           0x7FFFFFFF      // 表示最大值0.9999999995
#define VALUE_Q31_MIN           0x80000000      // 表示最小值-1.0
#define VALUE_Q31_HALF          0x40000000      // 表示0.5(特殊场景会用到)

// ================================================================================
// 5.基值设计Q15或者Q31
// (前面时间、弧度角度、电压和电流自变量)
// (后面频率、角速度、电阻、电感和磁链是因变量)
// VALUE_BASE_RAD这个固定设定为8.0
// RAD=8.0覆盖所有速度控制场景
// 所有用到定点的场景都是速度控制
// 位置控制精度要求高，不用定点
#ifdef DEFINE0_BASE_TIME
#define VALUE_BASE_TIME         (float)DEFINE0_BASE_TIME
#else // DEFINE0_BASE_TIME
#define VALUE_BASE_TIME         0.001953125f
#endif // DEFINE0_BASE_TIME

#ifdef DEFINE0_BASE_VOLTAGE
#define VALUE_BASE_VOLTAGE      (float)DEFINE0_BASE_VOLTAGE
#else // DEFINE0_BASE_VOLTAGE
#define VALUE_BASE_VOLTAGE      1024.0f
#endif // DEFINE0_BASE_VOLTAGE

#ifdef DEFINE0_BASE_CURRENT
#define VALUE_BASE_CURRENT      (float)DEFINE0_BASE_CURRENT
#else // DEFINE0_BASE_CURRENT
#define VALUE_BASE_CURRENT      64.0f
#endif // DEFINE0_BASE_CURRENT

#ifdef DEFINE0_BASE_OTHER
#define VALUE_BASE_OTHER        (float)DEFINE0_BASE_OTHER
#else // DEFINE0_BASE_OTHER
#define VALUE_BASE_OTHER        512.0f
#endif // DEFINE0_BASE_OTHER

#define VALUE_BASE_RAD          8.0f    // 固定BASE=8.0基值设计
#define VALUE_BASE_HZ           1.0f/VALUE_BASE_TIME
#define VALUE_BASE_SPEED        VALUE_BASE_RAD/VALUE_BASE_TIME
#define VALUE_BSAE_RESISTOR     VALUE_BASE_VOLTAGE/VALUE_BASE_CURRENT
#define VALUE_BASE_INDUCTOR     VALUE_BSAE_RESISTOR*VALUE_BASE_TIME
#define VALUE_BASE_FLUX         VALUE_BASE_VOLTAGE*VALUE_BASE_TIME


#endif // SGUAN_VALUE_H

#ifndef ___SGUAN_VALUE_H
#define __SGUAN_VALUE_H

/* SguanFOC配置文件声明 */
#include <stdint.h>             // (C标准库......)
#include <stdio.h>              // (C标准库......)
#include <string.h>             // (C标准库......)
#include "UnitLib_Define0.h"    // (应用库UnitLib)
#include "UnitLib_Define1.h"    // (应用库UnitLib)
#include "UserMotor_Define2.h"  // (电参库UserMotor)
#include "Zone_Define3.h"       // (用户驱动层Zone)

// ================================================================================
// 1.C和C++配置文件声明
#ifdef __cplusplus
#define CPLUSPLUS_C_BEGIN extern "C" {
#define CPLUSPLUS_C_END   }
#else
#define CPLUSPLUS_C_BEGIN
#define CPLUSPLUS_C_END
#endif

// ================================================================================
// 2.局部定点化计算格式
typedef int32_t Q31;            // 1位符号+0位整数+31位小数
typedef int16_t Q15;            // 1位符号+0位整数+15位小数

// ================================================================================
// 3.数据格式宏定义
#define VALUE_MODE_FLOAT        0x00 // 全局运算单精度浮点
#define VALUE_MODE_DOUBLE       0x01 // 全局运算双精度浮点
#define VALUE_MODE_Q15          0x02 // 全局运算Q15定点
#define VALUE_MODE_Q31          0x03 // 全局运算Q31定点

#ifndef DEFINE0_MOTOR_MATH
#define DEFINE0_MOTOR_MATH      VALUE_MODE_FLOAT
#endif // DEFINE0_MOTOR_MATH

#if !((DEFINE0_MOTOR_MATH>=VALUE_MODE_FLOAT) && (DEFINE0_MOTOR_MATH<=VALUE_MODE_Q31))
#define VALUE_MODE_MATH         VALUE_MODE_FLOAT
#else // DEFINE0_MOTOR_MATH
#define VALUE_MODE_MATH         DEFINE0_MOTOR_MATH
#endif // DEFINE0_MOTOR_MATH

// ================================================================================
// 4.Value数值分类
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
// 5.常用数值宏定义
// (若是定点，Rad例程代码按照8.0归一化)
/* 以下定义分别是浮点的：
(2pi的数值) (pi的数值) (二分之pi)
(二倍根号二) (二分之根号三)
(数值壹) (数值半) (数值零)等
定点Q15的数值：
(2pi的数值) (pi的数值) (二分之pi)等
定点Q31的数值：
(2pi的数值) (pi的数值) (二分之pi)等*/
#define VALUE_F_2PI             (SguanF)6.283185307179586477
#define VALUE_F_PI              (SguanF)3.141592653589793239
#define VALUE_F_PI_2            (SguanF)1.570796326794896619
#define VALUE_F_2_SQRT2         (SguanF)2.828427124746190098
#define VALUE_F_SQRT3           (SguanF)1.732050807568877293
#define VALUE_F_SQRT3_2         (SguanF)0.866025403784438646
#define VALUE_F_INV_SQRT3       (SguanF)0.577350269189625764
#define VALUE_F_LN2             (SguanF)0.693147180559945309
#define VALUE_F_INV_LN2         (SguanF)1.442695040888963407
#define VALUE_F_TWO             (SguanF)2.0         // 单纯数值贰
#define VALUE_F_ONE             (SguanF)1.0         // 单纯数值壹
#define VALUE_F_HALF            (SguanF)0.5         // 单纯数值半

#define VALUE_Q15_2PI           (Q15)25736          // BASE=8.0
#define VALUE_Q15_PI            (Q15)12868          // BASE=8.0
#define VALUE_Q15_PI_2          (Q15)6434           // BASE=8.0
#define VALUE_Q15_INV_SQRT3     (Q15)18919          // BASE=1.0
#define VALUE_Q15_SQRT3_2       (Q15)28378          // BASE=1.0
#define VALUE_Q15_MAX           (Q15)32767          // 表示最大值0.9999694824
#define VALUE_Q15_MIN           (Q15)(-32768)       // 表示最小值-1.0
#define VALUE_Q15_HALF          (Q15)16384          // 表示0.5(特殊场景会用到)

#define VALUE_Q31_2PI           (Q31)1686629713     // BASE=8.0
#define VALUE_Q31_PI            (Q31)843314857      // BASE=8.0
#define VALUE_Q31_PI_2          (Q31)421657428      // BASE=8.0
#define VALUE_Q31_INV_SQRT3     (Q31)1239850262     // BASE=1.0
#define VALUE_Q31_SQRT3_2       (Q31)1859775393     // BASE=1.0
#define VALUE_Q31_MAX           (Q31)2147483647     // 表示最大值0.9999999995
#define VALUE_Q31_MIN           (Q31)(-2147483648)  // 表示最小值-1.0
#define VALUE_Q31_HALF          (Q31)1073741824     // 表示0.5(特殊场景会用到)

// ================================================================================
// 6.基值设计Q15或者Q31
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
#define VALUE_BASE_CURRENT      128.0f
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

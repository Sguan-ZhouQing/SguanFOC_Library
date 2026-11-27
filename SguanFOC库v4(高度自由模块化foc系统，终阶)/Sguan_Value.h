/* ===================================================================
 * 文件:     Sguan_Value.h
 * 项目:     SguanFOC_v4 —— 开源 FOC 电机控制框架
 * 说明:     1.Sguan固件库_数值宏
 * 链接:     https://github.com/Sguan-ZhouQing/SguanFOC_Library
 * 版权:     Copyright 2026 星必尘Sguan <3464647102@qq.com>
 * 许可证:   Apache License, Version 2.0
 * SPDX:     SPDX-License-Identifier: Apache-2.0
 *
 * 许可说明:
 *   [EN] Licensed under the Apache License, Version 2.0. You may use,
 *        copy, modify, and distribute this software, including for
 *        commercial purposes, provided you retain this copyright
 *        notice and the terms of the License.
 *   [中文] 本软件依据 Apache License 2.0 授权：可自由使用、复制、修改、
 *        分发（含商业用途），但必须保留本版权声明与许可证条款。
 *   [DE] Lizenziert unter der Apache License, Version 2.0. Nutzung,
 *        Vervielfaeltigung, Aenderung und Weitergabe (auch kommerziell)
 *        sind erlaubt, sofern dieser Urheberrechtshinweis und die
 *        Lizenzbedingungen erhalten bleiben.
 *   [JA] Apache License, Version 2.0 に基づき許諾されます。使用・複製・
 *        改変・再配布（商用を含む）が可能ですが、本著作権表示と
 *        ライセンス条項を保持する必要があります。
 * 完整许可文本: 参见仓库根目录下的 LICENSE-APACHE 文件。
 * =================================================================== */
#ifndef ___SGUAN_VALUE_H
#define __SGUAN_VALUE_H

/* SguanFOC配置文件声明 */
#include <stdint.h>                                 // (C标准库......)
#include <stdio.h>                                  // (C标准库......)
#include <string.h>                                 // (C标准库......)
#include "UnitLib_Define0.h"                        // (应用库UnitLib)
#include "UnitLib_Define1.h"                        // (应用库UnitLib)
#include "UserMotor_Define2.h"                      // (电参库UserMotor)
#include "Zone_Define3.h"                           // (用户驱动层Zone)

// =================================================|==============================
// 1.C和C++配置文件声明
#ifdef __cplusplus
#define CPLUSPLUS_C_BEGIN extern "C" {              // (C和C++配置,可在C++中运行)
#define CPLUSPLUS_C_END   }                         // (C和C++配置,可在C++中运行)
#else
#define CPLUSPLUS_C_BEGIN
#define CPLUSPLUS_C_END
#endif

// =================================================|==============================
// 2.局部定点化计算格式
typedef int32_t Q31;                                // 1位符号+0位整数+31位小数
typedef int16_t Q15;                                // 1位符号+0位整数+15位小数

// =================================================|==============================
// 3.数据格式宏定义
#define VALUE_MODE_FLOAT        0x00                // 全局运算单精度浮点
#define VALUE_MODE_DOUBLE       0x01                // 全局运算双精度浮点
#define VALUE_MODE_Q15          0x02                // 全局运算Q15定点
#define VALUE_MODE_Q31          0x03                // 全局运算Q31定点

#ifndef DEFINE0_MOTOR_MATH
#define DEFINE0_MOTOR_MATH      VALUE_MODE_FLOAT
#endif // DEFINE0_MOTOR_MATH

#if !((DEFINE0_MOTOR_MATH>=VALUE_MODE_FLOAT) && (DEFINE0_MOTOR_MATH<=VALUE_MODE_Q31))
#define VALUE_MODE_MATH         VALUE_MODE_FLOAT
#else // DEFINE0_MOTOR_MATH
#define VALUE_MODE_MATH         DEFINE0_MOTOR_MATH
#endif // DEFINE0_MOTOR_MATH

// =================================================|==============================
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

// =================================================|==============================
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
#define VALUE_F_INV_SQRT3       (SguanF)0.577350269189625764
#define VALUE_F_SQRT3_2         (SguanF)0.866025403784438646
#define VALUE_F_2_3             (SguanF)0.666666666666666666
#define VALUE_F_LN2             (SguanF)0.693147180559945309
#define VALUE_F_INV_LN2         (SguanF)1.442695040888963407
#define VALUE_F_3H_TWENTY       (SguanF)320.0       // 纯数值320
#define VALUE_F_1H_SIXTY        (SguanF)160.0       // 纯数值160
#define VALUE_F_NINETY_SIX      (SguanF)96.0        // 单纯数值96
#define VALUE_F_SIXTY_FOUR      (SguanF)64.0        // 单纯数值64
#define VALUE_F_FORTY_EIGHT     (SguanF)48.0        // 单纯数值48
#define VALUE_F_THIRTY_TWO      (SguanF)32.0        // 单纯数值32
#define VALUE_F_TWENTY_FOUR     (SguanF)24.0        // 单纯数值24
#define VALUE_F_SIXTEEN         (SguanF)16.0        // 单纯数值16
#define VALUE_F_TEN             (SguanF)10.0        // 单纯数值10
#define VALUE_F_EIGHT           (SguanF)8.0         // 单纯数值捌
#define VALUE_F_SIX             (SguanF)6.0         // 单纯数值陆
#define VALUE_F_FIVE            (SguanF)5.0         // 单纯数值伍
#define VALUE_F_FOUR            (SguanF)4.0         // 单纯数值肆
#define VALUE_F_THREE           (SguanF)3.0         // 单纯数值叁
#define VALUE_F_TWO             (SguanF)2.0         // 单纯数值贰
#define VALUE_F_ONE             (SguanF)1.0         // 单纯数值壹
#define VALUE_F_HALF            (SguanF)0.5         // 单纯数值半

#define VALUE_Q15_2PI           (Q15)25736          // BASE=8.0
#define VALUE_Q15_PI            (Q15)12868          // BASE=8.0
#define VALUE_Q15_PI_2          (Q15)6434           // BASE=8.0
#define VALUE_Q15_INV_SQRT3     (Q15)18919          // BASE=1.0
#define VALUE_Q15_SQRT3_2       (Q15)28378          // BASE=1.0
#define VALUE_Q15_2_3           (Q15)21845          // BASE=1.0
#define VALUE_Q15_MAX           (Q15)32767          // 表示最大值0.9999694824
#define VALUE_Q15_MIN           (Q15)(-32768)       // 表示最小值-1.0
#define VALUE_Q15_HALF          (Q15)16384          // 表示0.5(特殊场景会用到)

#define VALUE_Q31_2PI           (Q31)1686629713     // BASE=8.0
#define VALUE_Q31_PI            (Q31)843314857      // BASE=8.0
#define VALUE_Q31_PI_2          (Q31)421657428      // BASE=8.0
#define VALUE_Q31_INV_SQRT3     (Q31)1239850262     // BASE=1.0
#define VALUE_Q31_SQRT3_2       (Q31)1859775393     // BASE=1.0
#define VALUE_Q31_2_3           (Q31)1431655765     // BASE=1.0
#define VALUE_Q31_MAX           (Q31)2147483647     // 表示最大值0.9999999995
#define VALUE_Q31_MIN           (Q31)(-2147483648)  // 表示最小值-1.0
#define VALUE_Q31_HALF          (Q31)1073741824     // 表示0.5(特殊场景会用到)

// =================================================|==============================
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

/* ===================================================================
 * 文件:     Sguan_Config.h
 * 项目:     SguanFOC_v4 —— 开源 FOC 电机控制框架
 * 说明:     1.Sguan固件库_配置文件
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


#ifndef __SGUAN_CONFIG_H
#define __SGUAN_CONFIG_H

/* SguanFOC配置文件声明 */
#include "Sguan_Math.h"

// ================================================================================
// 1.Transfer模块使用的宏定义
#ifdef DEFINE0_NUM0_TRANSFER1
#define CONFIG_TRANSFER1    DEFINE0_NUM0_TRANSFER1
#else // DEFINE0_NUM0_TRANSFER1
#define CONFIG_TRANSFER1    0
#endif // DEFINE0_NUM0_TRANSFER1

#ifdef DEFINE0_NUM0_TRANSFER2
#define CONFIG_TRANSFER2    DEFINE0_NUM0_TRANSFER2
#else // DEFINE0_NUM0_TRANSFER2
#define CONFIG_TRANSFER2    0
#endif // DEFINE0_NUM0_TRANSFER2

#ifdef DEFINE0_NUM0_TRANSFER3
#define CONFIG_TRANSFER3    DEFINE0_NUM0_TRANSFER3
#else // DEFINE0_NUM0_TRANSFER3
#define CONFIG_TRANSFER3    0
#endif // DEFINE0_NUM0_TRANSFER3

#ifdef DEFINE0_NUM0_TRANSFER4
#define CONFIG_TRANSFER4    DEFINE0_NUM0_TRANSFER4
#else // DEFINE0_NUM0_TRANSFER4
#define CONFIG_TRANSFER4    0
#endif // DEFINE0_NUM0_TRANSFER4

#ifdef DEFINE0_NUM0_TRANSFER5
#define CONFIG_TRANSFER5    DEFINE0_NUM0_TRANSFER5
#else // DEFINE0_NUM0_TRANSFER5
#define CONFIG_TRANSFER5    0
#endif // DEFINE0_NUM0_TRANSFER5

#ifdef DEFINE0_NUM0_INTEGRATOR
#define CONFIG_INTEGRATOR   DEFINE0_NUM0_INTEGRATOR
#else // DEFINE0_NUM0_INTEGRATOR
#define CONFIG_INTEGRATOR   0
#endif // DEFINE0_NUM0_INTEGRATOR

#ifdef DEFINE0_NUM0_DERIVATIVE
#define CONFIG_DERIVATIVE   DEFINE0_NUM0_DERIVATIVE
#else // DEFINE0_NUM0_DERIVATIVE
#define CONFIG_DERIVATIVE   0
#endif // DEFINE0_NUM0_DERIVATIVE

#ifdef DEFINE0_NUM0_DELAY1
#define CONFIG_DELAY1       DEFINE0_NUM0_DELAY1
#else // DEFINE0_NUM0_DELAY1
#define CONFIG_DELAY1       0
#endif // DEFINE0_NUM0_DELAY1

#ifdef DEFINE0_NUM0_DELAY2
#define CONFIG_DELAY2       DEFINE0_NUM0_DELAY2
#else // DEFINE0_NUM0_DELAY2
#define CONFIG_DELAY2       0
#endif // DEFINE0_NUM0_DELAY2

#ifdef DEFINE0_NUM0_DELAY3
#define CONFIG_DELAY3       DEFINE0_NUM0_DELAY3
#else // DEFINE0_NUM0_DELAY3
#define CONFIG_DELAY3       0
#endif // DEFINE0_NUM0_DELAY3

#ifdef DEFINE0_NUM0_HALL
#define CONFIG_HALL         DEFINE0_NUM0_HALL
#else // DEFINE0_NUM0_HALL
#define CONFIG_HALL         0
#endif // DEFINE0_NUM0_HALL

#ifdef DEFINE0_NUM0_LADRC1
#define CONFIG_LADRC1       DEFINE0_NUM0_LADRC1
#else // DEFINE0_NUM0_LADRC1
#define CONFIG_LADRC1       0
#endif // DEFINE0_NUM0_LADRC1

#ifdef DEFINE0_NUM0_LADRC2
#define CONFIG_LADRC2       DEFINE0_NUM0_LADRC2
#else // DEFINE0_NUM0_LADRC2
#define CONFIG_LADRC2       0
#endif // DEFINE0_NUM0_LADRC2

#ifdef DEFINE0_NUM0_SMC
#define CONFIG_SMC          DEFINE0_NUM0_SMC
#else // DEFINE0_NUM0_SMC
#define CONFIG_SMC          0
#endif // DEFINE0_NUM0_SMC

#ifdef DEFINE0_NUM0_DPCC
#define CONFIG_DPCC         DEFINE0_NUM0_DPCC
#else // DEFINE0_NUM0_DPCC
#define CONFIG_DPCC         0
#endif // DEFINE0_NUM0_DPCC

#ifdef DEFINE0_NUM0_PIR
#define CONFIG_PIR          DEFINE0_NUM0_PIR
#else // DEFINE0_NUM0_PIR
#define CONFIG_PIR          0
#endif // DEFINE0_NUM0_PIR

#ifdef DEFINE0_NUM0_PID
#define CONFIG_PID          DEFINE0_NUM0_PID
#else // DEFINE0_NUM0_PID
#define CONFIG_PID          0
#endif // DEFINE0_NUM0_PID

#ifdef DEFINE0_NUM0_PLL
#define CONFIG_PLL          DEFINE0_NUM0_PLL
#else // DEFINE0_NUM0_PLL
#define CONFIG_PLL          0
#endif // DEFINE0_NUM0_PLL

#ifdef DEFINE0_NUM0_LPF1
#define CONFIG_LPF1         DEFINE0_NUM0_LPF1
#else // DEFINE0_NUM0_LPF1
#define CONFIG_LPF1         0
#endif // DEFINE0_NUM0_LPF1

#ifdef DEFINE0_NUM0_LPF2
#define CONFIG_LPF2         DEFINE0_NUM0_LPF2
#else // DEFINE0_NUM0_LPF2
#define CONFIG_LPF2         0
#endif // DEFINE0_NUM0_LPF2

#ifdef DEFINE0_NUM0_HPF1
#define CONFIG_HPF1         DEFINE0_NUM0_HPF1
#else // DEFINE0_NUM0_HPF1
#define CONFIG_HPF1         0
#endif // DEFINE0_NUM0_HPF1

#ifdef DEFINE0_NUM0_HPF2
#define CONFIG_HPF2         DEFINE0_NUM0_HPF2
#else // DEFINE0_NUM0_HPF2
#define CONFIG_HPF2         0
#endif // DEFINE0_NUM0_HPF2

#ifdef DEFINE0_NUM0_BPF1
#define CONFIG_BPF1         DEFINE0_NUM0_BPF1
#else // DEFINE0_NUM0_BPF1
#define CONFIG_BPF1         0
#endif // DEFINE0_NUM0_BPF1

#ifdef DEFINE0_NUM0_BPF2
#define CONFIG_BPF2         DEFINE0_NUM0_BPF2
#else // DEFINE0_NUM0_BPF2
#define CONFIG_BPF2         0
#endif // DEFINE0_NUM0_BPF2

#ifdef DEFINE0_NUM0_SOGI
#define CONFIG_SOGI         DEFINE0_NUM0_SOGI
#else // DEFINE0_NUM0_SOGI
#define CONFIG_SOGI         0
#endif // DEFINE0_NUM0_SOGI

#ifdef DEFINE0_NUM0_NF
#define CONFIG_NF           DEFINE0_NUM0_NF
#else // DEFINE0_NUM0_NF
#define CONFIG_NF           0
#endif // DEFINE0_NUM0_NF

#ifdef DEFINE0_NUM0_TPNF
#define CONFIG_TPNF         DEFINE0_NUM0_TPNF
#else // DEFINE0_NUM0_TPNF
#define CONFIG_TPNF         0
#endif // DEFINE0_NUM0_TPNF

#ifdef DEFINE0_NUM0_DOB
#define CONFIG_DOB          DEFINE0_NUM0_DOB
#else // DEFINE0_NUM0_DOB
#define CONFIG_DOB          0
#endif // DEFINE0_NUM0_DOB

#ifdef DEFINE0_NUM0_RLS
#define CONFIG_RLS          DEFINE0_NUM0_RLS
#else // DEFINE0_NUM0_RLS
#define CONFIG_RLS          0
#endif // DEFINE0_NUM0_RLS

#ifdef DEFINE0_NUM0_SMO
#define CONFIG_SMO          DEFINE0_NUM0_SMO
#else // DEFINE0_NUM0_SMO
#define CONFIG_SMO          0
#endif // DEFINE0_NUM0_SMO

#ifdef DEFINE0_NUM0_NFO
#define CONFIG_NFO          DEFINE0_NUM0_NFO
#else // DEFINE0_NUM0_NLFO
#define CONFIG_NFO          0
#endif // DEFINE0_NUM0_NLFO

#ifdef DEFINE0_NUM0_AFO
#define CONFIG_AFO          DEFINE0_NUM0_AFO
#else // DEFINE0_NUM0_VCFO
#define CONFIG_AFO          0
#endif // DEFINE0_NUM0_VCFO

#ifdef DEFINE0_NUM0_HFI
#define CONFIG_HFI          DEFINE0_NUM0_HFI
#else // DEFINE0_NUM0_HFI
#define CONFIG_HFI          0
#endif // DEFINE0_NUM0_HFI

#ifdef DEFINE0_NUM0_ROLO
#define CONFIG_ROLO         DEFINE0_NUM0_ROLO
#else // DEFINE0_NUM0_ROLO
#define CONFIG_ROLO         0
#endif // DEFINE0_NUM0_ROLO

#ifdef DEFINE0_NUM0_MARS
#define CONFIG_MARS         DEFINE0_NUM0_MARS
#else // DEFINE0_NUM0_MARS
#define CONFIG_MARS         0
#endif // DEFINE0_NUM0_MARS

#ifdef DEFINE0_NUM0_EKF
#define CONFIG_EKF          DEFINE0_NUM0_EKF
#else // DEFINE0_NUM0_EKF
#define CONFIG_EKF          0
#endif // DEFINE0_NUM0_EKF

// ================================================================================
// 2.Method模块使用的宏定义
#ifdef DEFINE0_NUM1_SGUANQ
#define CONFIG_SGUANQ       DEFINE0_NUM1_SGUANQ
#else // DEFINE0_NUM1_SGUANQ
#define CONFIG_SGUANQ       0
#endif // DEFINE0_NUM1_SGUANQ

#ifdef DEFINE0_NUM1_INT8
#define CONFIG_INT8         DEFINE0_NUM1_INT8
#else // DEFINE0_NUM1_INT8
#define CONFIG_INT8         0
#endif // DEFINE0_NUM1_INT8

#ifdef DEFINE0_NUM1_UINT8
#define CONFIG_UINT8        DEFINE0_NUM1_UINT8
#else // DEFINE0_NUM1_UINT8
#define CONFIG_UINT8        0
#endif // DEFINE0_NUM1_UINT8

#ifdef DEFINE0_NUM1_INT16
#define CONFIG_INT16        DEFINE0_NUM1_INT16
#else // DEFINE0_NUM1_INT16
#define CONFIG_INT16        0
#endif // DEFINE0_NUM1_INT16

#ifdef DEFINE0_NUM1_UINT16
#define CONFIG_UINT16       DEFINE0_NUM1_UINT16
#else // DEFINE0_NUM1_UINT16
#define CONFIG_UINT16       0
#endif // DEFINE0_NUM1_UINT16

#ifdef DEFINE0_NUM1_INT32
#define CONFIG_INT32        DEFINE0_NUM1_INT32
#else // DEFINE0_NUM1_INT32
#define CONFIG_INT32        0
#endif // DEFINE0_NUM1_INT32

#ifdef DEFINE0_NUM1_UINT32
#define CONFIG_UINT32       DEFINE0_NUM1_UINT32
#else // DEFINE0_NUM1_UINT32
#define CONFIG_UINT32       0
#endif // DEFINE0_NUM1_UINT32

// ================================================================================
// 3.MOTOR电机使用数量的宏定义
#ifndef DEFINE0_MOTOR_COUNT
#define DEFINE0_MOTOR_COUNT 1
#endif

#if !(DEFINE0_MOTOR_COUNT >= 1 && DEFINE0_MOTOR_COUNT <= 6)
#define CONFIG_MOTOR        1
#else // DEFINE0_MOTOR_COUNT
#define CONFIG_MOTOR        DEFINE0_MOTOR_COUNT
#endif // DEFINE0_MOTOR_COUNT

// ================================================================================
// 4.MOTOR电机运行模式的宏定义
#ifndef DEFINE0_MOTOR_MODE
#define DEFINE0_MOTOR_MODE  0
#endif

#if !(DEFINE0_MOTOR_MODE >= 0)
#define CONFIG_MODE         0
#else // DEFINE0_MOTOR_MODE
#define CONFIG_MODE         DEFINE0_MOTOR_MODE
#endif // DEFINE0_MOTOR_MODE

// ================================================================================
// 5.控制系统离散周期的宏定义
#define PMSM_RUN_T          math_from_float((SguanF)DEFINE3_TIM_T,VALUE_BASE_TIME)


#endif // SGUAN_CONFIG_H

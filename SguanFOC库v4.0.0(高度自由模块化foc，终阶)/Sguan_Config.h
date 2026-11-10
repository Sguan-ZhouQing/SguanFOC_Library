#ifndef __SGUAN_CONFIG_H
#define __SGUAN_CONFIG_H

/* SguanFOC配置文件声明 */
#include "Sguan_Math.h"

// ================================================================================
// 1.Transfer模块使用的宏定义
#ifdef DEFINE0_NUM_TRANSFER1
#define CONFIG_TRANSFER1    DEFINE0_NUM_TRANSFER1
#else // DEFINE0_NUM_TRANSFER1
#define CONFIG_TRANSFER1    0
#endif // DEFINE0_NUM_TRANSFER1

#ifdef DEFINE0_NUM_TRANSFER2
#define CONFIG_TRANSFER2    DEFINE0_NUM_TRANSFER2
#else // DEFINE0_NUM_TRANSFER2
#define CONFIG_TRANSFER2    0
#endif // DEFINE0_NUM_TRANSFER2

#ifdef DEFINE0_NUM_TRANSFER3
#define CONFIG_TRANSFER3    DEFINE0_NUM_TRANSFER3
#else // DEFINE0_NUM_TRANSFER3
#define CONFIG_TRANSFER3    0
#endif // DEFINE0_NUM_TRANSFER3

#ifdef DEFINE0_NUM_TRANSFER4
#define CONFIG_TRANSFER4    DEFINE0_NUM_TRANSFER4
#else // DEFINE0_NUM_TRANSFER4
#define CONFIG_TRANSFER4    0
#endif // DEFINE0_NUM_TRANSFER4

#ifdef DEFINE0_NUM_TRANSFER5
#define CONFIG_TRANSFER5    DEFINE0_NUM_TRANSFER5
#else // DEFINE0_NUM_TRANSFER5
#define CONFIG_TRANSFER5    0
#endif // DEFINE0_NUM_TRANSFER5

#ifdef DEFINE0_NUM_INTEGRATOR
#define CONFIG_INTEGRATOR   DEFINE0_NUM_INTEGRATOR
#else // DEFINE0_NUM_INTEGRATOR
#define CONFIG_INTEGRATOR   0
#endif // DEFINE0_NUM_INTEGRATOR

#ifdef DEFINE0_NUM_DERIVATIVE
#define CONFIG_DERIVATIVE   DEFINE0_NUM_DERIVATIVE
#else // DEFINE0_NUM_DERIVATIVE
#define CONFIG_DERIVATIVE   0
#endif // DEFINE0_NUM_DERIVATIVE

#ifdef DEFINE0_NUM_CURVE
#define CONFIG_CURVE        DEFINE0_NUM_CURVE
#else // DEFINE0_NUM_CURVE
#define CONFIG_CURVE        0
#endif // DEFINE0_NUM_CURVE

#ifdef DEFINE0_NUM_DFT
#define CONFIG_DFT          DEFINE0_NUM_DFT
#else // DEFINE0_NUM_DFT
#define CONFIG_DFT          0
#endif // DEFINE0_NUM_DFT

#ifdef DEFINE0_NUM_HALL
#define CONFIG_HALL         DEFINE0_NUM_HALL
#else // DEFINE0_NUM_HALL
#define CONFIG_HALL         0
#endif // DEFINE0_NUM_HALL

#ifdef DEFINE0_NUM_LADRC1
#define CONFIG_LADRC1       DEFINE0_NUM_LADRC1
#else // DEFINE0_NUM_LADRC1
#define CONFIG_LADRC1       0
#endif // DEFINE0_NUM_LADRC1

#ifdef DEFINE0_NUM_LADRC2
#define CONFIG_LADRC2       DEFINE0_NUM_LADRC2
#else // DEFINE0_NUM_LADRC2
#define CONFIG_LADRC2       0
#endif // DEFINE0_NUM_LADRC2

#ifdef DEFINE0_NUM_SMC
#define CONFIG_SMC          DEFINE0_NUM_SMC
#else // DEFINE0_NUM_SMC
#define CONFIG_SMC          0
#endif // DEFINE0_NUM_SMC

#ifdef DEFINE0_NUM_DPCC
#define CONFIG_DPCC         DEFINE0_NUM_DPCC
#else // DEFINE0_NUM_DPCC
#define CONFIG_DPCC         0
#endif // DEFINE0_NUM_DPCC

#ifdef DEFINE0_NUM_PIR
#define CONFIG_PIR          DEFINE0_NUM_PIR
#else // DEFINE0_NUM_PIR
#define CONFIG_PIR          0
#endif // DEFINE0_NUM_PIR

#ifdef DEFINE0_NUM_PID
#define CONFIG_PID          DEFINE0_NUM_PID
#else // DEFINE0_NUM_PID
#define CONFIG_PID          0
#endif // DEFINE0_NUM_PID

#ifdef DEFINE0_NUM_PLL
#define CONFIG_PLL          DEFINE0_NUM_PLL
#else // DEFINE0_NUM_PLL
#define CONFIG_PLL          0
#endif // DEFINE0_NUM_PLL

#ifdef DEFINE0_NUM_LPF1
#define CONFIG_LPF1         DEFINE0_NUM_LPF1
#else // DEFINE0_NUM_LPF1
#define CONFIG_LPF1         0
#endif // DEFINE0_NUM_LPF1

#ifdef DEFINE0_NUM_LPF2
#define CONFIG_LPF2         DEFINE0_NUM_LPF2
#else // DEFINE0_NUM_LPF2
#define CONFIG_LPF2         0
#endif // DEFINE0_NUM_LPF2

#ifdef DEFINE0_NUM_HPF1
#define CONFIG_HPF1         DEFINE0_NUM_HPF1
#else // DEFINE0_NUM_HPF1
#define CONFIG_HPF1         0
#endif // DEFINE0_NUM_HPF1

#ifdef DEFINE0_NUM_HPF2
#define CONFIG_HPF2         DEFINE0_NUM_HPF2
#else // DEFINE0_NUM_HPF2
#define CONFIG_HPF2         0
#endif // DEFINE0_NUM_HPF2

#ifdef DEFINE0_NUM_BPF1
#define CONFIG_BPF1         DEFINE0_NUM_BPF1
#else // DEFINE0_NUM_BPF1
#define CONFIG_BPF1         0
#endif // DEFINE0_NUM_BPF1

#ifdef DEFINE0_NUM_BPF2
#define CONFIG_BPF2         DEFINE0_NUM_BPF2
#else // DEFINE0_NUM_BPF2
#define CONFIG_BPF2         0
#endif // DEFINE0_NUM_BPF2

#ifdef DEFINE0_NUM_SOGI
#define CONFIG_SOGI         DEFINE0_NUM_SOGI
#else // DEFINE0_NUM_SOGI
#define CONFIG_SOGI         0
#endif // DEFINE0_NUM_SOGI

#ifdef DEFINE0_NUM_NF
#define CONFIG_NF           DEFINE0_NUM_NF
#else // DEFINE0_NUM_NF
#define CONFIG_NF           0
#endif // DEFINE0_NUM_NF

#ifdef DEFINE0_NUM_TPNF
#define CONFIG_TPNF         DEFINE0_NUM_TPNF
#else // DEFINE0_NUM_TPNF
#define CONFIG_TPNF         0
#endif // DEFINE0_NUM_TPNF

#ifdef DEFINE0_NUM_DOB
#define CONFIG_DOB          DEFINE0_NUM_DOB
#else // DEFINE0_NUM_DOB
#define CONFIG_DOB          0
#endif // DEFINE0_NUM_DOB

#ifdef DEFINE0_NUM_RLS
#define CONFIG_RLS          DEFINE0_NUM_RLS
#else // DEFINE0_NUM_RLS
#define CONFIG_RLS          0
#endif // DEFINE0_NUM_RLS

#ifdef DEFINE0_NUM_SMO
#define CONFIG_SMO          DEFINE0_NUM_SMO
#else // DEFINE0_NUM_SMO
#define CONFIG_SMO          0
#endif // DEFINE0_NUM_SMO

#ifdef DEFINE0_NUM_NFO
#define CONFIG_NFO          DEFINE0_NUM_NFO
#else // DEFINE0_NUM_NLFO
#define CONFIG_NFO          0
#endif // DEFINE0_NUM_NLFO

#ifdef DEFINE0_NUM_AFO
#define CONFIG_AFO          DEFINE0_NUM_AFO
#else // DEFINE0_NUM_VCFO
#define CONFIG_AFO          0
#endif // DEFINE0_NUM_VCFO

#ifdef DEFINE0_NUM_HFI
#define CONFIG_HFI          DEFINE0_NUM_HFI
#else // DEFINE0_NUM_HFI
#define CONFIG_HFI          0
#endif // DEFINE0_NUM_HFI

#ifdef DEFINE0_NUM_ROLO
#define CONFIG_ROLO         DEFINE0_NUM_ROLO
#else // DEFINE0_NUM_ROLO
#define CONFIG_ROLO         0
#endif // DEFINE0_NUM_ROLO

#ifdef DEFINE0_NUM_MARS
#define CONFIG_MARS         DEFINE0_NUM_MARS
#else // DEFINE0_NUM_MARS
#define CONFIG_MARS         0
#endif // DEFINE0_NUM_MARS

#ifdef DEFINE0_NUM_EKF
#define CONFIG_EKF          DEFINE0_NUM_EKF
#else // DEFINE0_NUM_EKF
#define CONFIG_EKF          0
#endif // DEFINE0_NUM_EKF

#ifdef DEFINE0_NUM_DELAY1
#define CONFIG_DELAY1       DEFINE0_NUM_DELAY1
#else // DEFINE0_NUM_DELAY1
#define CONFIG_DELAY1       0
#endif // DEFINE0_NUM_DELAY1

#ifdef DEFINE0_NUM_DELAY2
#define CONFIG_DELAY2       DEFINE0_NUM_DELAY2
#else // DEFINE0_NUM_DELAY2
#define CONFIG_DELAY2       0
#endif // DEFINE0_NUM_DELAY2

#ifdef DEFINE0_NUM_DELAY3
#define CONFIG_DELAY3       DEFINE0_NUM_DELAY3
#else // DEFINE0_NUM_DELAY3
#define CONFIG_DELAY3       0
#endif // DEFINE0_NUM_DELAY3

// ================================================================================
// 2.Method模块使用的宏定义
#ifdef DEFINE0_NUM_FLOAT
#define CONFIG_Float        DEFINE0_NUM_FLOAT
#else // DEFINE0_NUM_FLOAT
#define CONFIG_Float        0
#endif // DEFINE0_NUM_FLOAT

#ifdef DEFINE0_NUM_INT8
#define CONFIG_Int8         DEFINE0_NUM_INT8
#else // DEFINE0_NUM_INT8
#define CONFIG_Int8         0
#endif // DEFINE0_NUM_INT8

#ifdef DEFINE0_NUM_UINT8
#define CONFIG_Uint8        DEFINE0_NUM_UINT8
#else // DEFINE0_NUM_UINT8
#define CONFIG_Uint8        0
#endif // DEFINE0_NUM_UINT8

#ifdef DEFINE0_NUM_INT16
#define CONFIG_Int16        DEFINE0_NUM_INT16
#else // DEFINE0_NUM_INT16
#define CONFIG_Int16        0
#endif // DEFINE0_NUM_INT16

#ifdef DEFINE0_NUM_UINT16
#define CONFIG_Uint16       DEFINE0_NUM_UINT16
#else // DEFINE0_NUM_UINT16
#define CONFIG_Uint16       0
#endif // DEFINE0_NUM_UINT16

#ifdef DEFINE0_NUM_INT32
#define CONFIG_Int32        DEFINE0_NUM_INT32
#else // DEFINE0_NUM_INT32
#define CONFIG_Int32        0
#endif // DEFINE0_NUM_INT32

#ifdef DEFINE0_NUM_UINT32
#define CONFIG_Uint32       DEFINE0_NUM_UINT32
#else // DEFINE0_NUM_UINT32
#define CONFIG_Uint32       0
#endif // DEFINE0_NUM_UINT32

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
#define PMSM_RUN_T          math_from_float((SguanF)DEFINE2_TIM_T,VALUE_BASE_TIME)


#endif // SGUAN_CONFIG_H

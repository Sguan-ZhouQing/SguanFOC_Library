#ifndef __SGUAN_CONFIG_H
#define __SGUAN_CONFIG_H

/* SguanFOC配置文件声明 */
#include "Sguan_Math.h"

// ================================================================================
// 1.Transfer模块使用的宏定义
#ifdef DESIGN_NUM_TRANSFER1
#define CONFIG_TRANSFER1    DESIGN_NUM_TRANSFER1
#else // DESIGN_NUM_TRANSFER1
#define CONFIG_TRANSFER1    0
#endif // DESIGN_NUM_TRANSFER1

#ifdef DESIGN_NUM_TRANSFER2
#define CONFIG_TRANSFER2    DESIGN_NUM_TRANSFER2
#else // DESIGN_NUM_TRANSFER2
#define CONFIG_TRANSFER2    0
#endif // DESIGN_NUM_TRANSFER2

#ifdef DESIGN_NUM_TRANSFER3
#define CONFIG_TRANSFER3    DESIGN_NUM_TRANSFER3
#else // DESIGN_NUM_TRANSFER3
#define CONFIG_TRANSFER3    0
#endif // DESIGN_NUM_TRANSFER3

#ifdef DESIGN_NUM_TRANSFER4
#define CONFIG_TRANSFER4    DESIGN_NUM_TRANSFER4
#else // DESIGN_NUM_TRANSFER4
#define CONFIG_TRANSFER4    0
#endif // DESIGN_NUM_TRANSFER4

#ifdef DESIGN_NUM_TRANSFER5
#define CONFIG_TRANSFER5    DESIGN_NUM_TRANSFER5
#else // DESIGN_NUM_TRANSFER5
#define CONFIG_TRANSFER5    0
#endif // DESIGN_NUM_TRANSFER5

#ifdef DESIGN_NUM_INTEGRATOR
#define CONFIG_INTEGRATOR   DESIGN_NUM_INTEGRATOR
#else // DESIGN_NUM_INTEGRATOR
#define CONFIG_INTEGRATOR   0
#endif // DESIGN_NUM_INTEGRATOR

#ifdef DESIGN_NUM_DERIVATIVE
#define CONFIG_DERIVATIVE   DESIGN_NUM_DERIVATIVE
#else // DESIGN_NUM_DERIVATIVE
#define CONFIG_DERIVATIVE   0
#endif // DESIGN_NUM_DERIVATIVE

#ifdef DESIGN_NUM_CURVE
#define CONFIG_CURVE        DESIGN_NUM_CURVE
#else // DESIGN_NUM_CURVE
#define CONFIG_CURVE        0
#endif // DESIGN_NUM_CURVE

#ifdef DESIGN_NUM_DFT
#define CONFIG_DFT          DESIGN_NUM_DFT
#else // DESIGN_NUM_DFT
#define CONFIG_DFT          0
#endif // DESIGN_NUM_DFT

#ifdef DESIGN_NUM_HALL
#define CONFIG_HALL         DESIGN_NUM_HALL
#else // DESIGN_NUM_HALL
#define CONFIG_HALL         0
#endif // DESIGN_NUM_HALL

#ifdef DESIGN_NUM_LADRC1
#define CONFIG_LADRC1       DESIGN_NUM_LADRC1
#else // DESIGN_NUM_LADRC1
#define CONFIG_LADRC1       0
#endif // DESIGN_NUM_LADRC1

#ifdef DESIGN_NUM_LADRC2
#define CONFIG_LADRC2       DESIGN_NUM_LADRC2
#else // DESIGN_NUM_LADRC2
#define CONFIG_LADRC2       0
#endif // DESIGN_NUM_LADRC2

#ifdef DESIGN_NUM_SMC
#define CONFIG_SMC          DESIGN_NUM_SMC
#else // DESIGN_NUM_SMC
#define CONFIG_SMC          0
#endif // DESIGN_NUM_SMC

#ifdef DESIGN_NUM_DPCC
#define CONFIG_DPCC         DESIGN_NUM_DPCC
#else // DESIGN_NUM_DPCC
#define CONFIG_DPCC         0
#endif // DESIGN_NUM_DPCC

#ifdef DESIGN_NUM_PIR
#define CONFIG_PIR          DESIGN_NUM_PIR
#else // DESIGN_NUM_PIR
#define CONFIG_PIR          0
#endif // DESIGN_NUM_PIR

#ifdef DESIGN_NUM_PID
#define CONFIG_PID          DESIGN_NUM_PID
#else // DESIGN_NUM_PID
#define CONFIG_PID          0
#endif // DESIGN_NUM_PID

#ifdef DESIGN_NUM_PLL
#define CONFIG_PLL          DESIGN_NUM_PLL
#else // DESIGN_NUM_PLL
#define CONFIG_PLL          0
#endif // DESIGN_NUM_PLL

#ifdef DESIGN_NUM_LPF1
#define CONFIG_LPF1         DESIGN_NUM_LPF1
#else // DESIGN_NUM_LPF1
#define CONFIG_LPF1         0
#endif // DESIGN_NUM_LPF1

#ifdef DESIGN_NUM_LPF2
#define CONFIG_LPF2         DESIGN_NUM_LPF2
#else // DESIGN_NUM_LPF2
#define CONFIG_LPF2         0
#endif // DESIGN_NUM_LPF2

#ifdef DESIGN_NUM_HPF1
#define CONFIG_HPF1         DESIGN_NUM_HPF1
#else // DESIGN_NUM_HPF1
#define CONFIG_HPF1         0
#endif // DESIGN_NUM_HPF1

#ifdef DESIGN_NUM_HPF2
#define CONFIG_HPF2         DESIGN_NUM_HPF2
#else // DESIGN_NUM_HPF2
#define CONFIG_HPF2         0
#endif // DESIGN_NUM_HPF2

#ifdef DESIGN_NUM_BPF1
#define CONFIG_BPF1         DESIGN_NUM_BPF1
#else // DESIGN_NUM_BPF1
#define CONFIG_BPF1         0
#endif // DESIGN_NUM_BPF1

#ifdef DESIGN_NUM_BPF2
#define CONFIG_BPF2         DESIGN_NUM_BPF2
#else // DESIGN_NUM_BPF2
#define CONFIG_BPF2         0
#endif // DESIGN_NUM_BPF2

#ifdef DESIGN_NUM_SOGI
#define CONFIG_SOGI         DESIGN_NUM_SOGI
#else // DESIGN_NUM_SOGI
#define CONFIG_SOGI         0
#endif // DESIGN_NUM_SOGI

#ifdef DESIGN_NUM_NF
#define CONFIG_NF           DESIGN_NUM_NF
#else // DESIGN_NUM_NF
#define CONFIG_NF           0
#endif // DESIGN_NUM_NF

#ifdef DESIGN_NUM_TPNF
#define CONFIG_TPNF         DESIGN_NUM_TPNF
#else // DESIGN_NUM_TPNF
#define CONFIG_TPNF         0
#endif // DESIGN_NUM_TPNF

#ifdef DESIGN_NUM_DOB
#define CONFIG_DOB          DESIGN_NUM_DOB
#else // DESIGN_NUM_DOB
#define CONFIG_DOB          0
#endif // DESIGN_NUM_DOB

#ifdef DESIGN_NUM_RLS
#define CONFIG_RLS          DESIGN_NUM_RLS
#else // DESIGN_NUM_RLS
#define CONFIG_RLS          0
#endif // DESIGN_NUM_RLS

#ifdef DESIGN_NUM_SMO
#define CONFIG_SMO          DESIGN_NUM_SMO
#else // DESIGN_NUM_SMO
#define CONFIG_SMO          0
#endif // DESIGN_NUM_SMO

#ifdef DESIGN_NUM_NLFO
#define CONFIG_NLFO         DESIGN_NUM_NLFO
#else // DESIGN_NUM_NLFO
#define CONFIG_NLFO         0
#endif // DESIGN_NUM_NLFO

#ifdef DESIGN_NUM_VCFO
#define CONFIG_VCFO         DESIGN_NUM_VCFO
#else // DESIGN_NUM_VCFO
#define CONFIG_VCFO         0
#endif // DESIGN_NUM_VCFO

#ifdef DESIGN_NUM_HFI
#define CONFIG_HFI          DESIGN_NUM_HFI
#else // DESIGN_NUM_HFI
#define CONFIG_HFI          0
#endif // DESIGN_NUM_HFI

#ifdef DESIGN_NUM_ROLO
#define CONFIG_ROLO         DESIGN_NUM_ROLO
#else // DESIGN_NUM_ROLO
#define CONFIG_ROLO         0
#endif // DESIGN_NUM_ROLO

#ifdef DESIGN_NUM_MARS
#define CONFIG_MARS         DESIGN_NUM_MARS
#else // DESIGN_NUM_MARS
#define CONFIG_MARS         0
#endif // DESIGN_NUM_MARS

#ifdef DESIGN_NUM_EKF
#define CONFIG_EKF          DESIGN_NUM_EKF
#else // DESIGN_NUM_EKF
#define CONFIG_EKF          0
#endif // DESIGN_NUM_EKF

#ifdef DESIGN_NUM_DELAY1
#define CONFIG_DELAY1       DESIGN_NUM_DELAY1
#else // DESIGN_NUM_DELAY1
#define CONFIG_DELAY1       0
#endif // DESIGN_NUM_DELAY1

#ifdef DESIGN_NUM_DELAY2
#define CONFIG_DELAY2       DESIGN_NUM_DELAY2
#else // DESIGN_NUM_DELAY2
#define CONFIG_DELAY2       0
#endif // DESIGN_NUM_DELAY2

#ifdef DESIGN_NUM_DELAY3
#define CONFIG_DELAY3       DESIGN_NUM_DELAY3
#else // DESIGN_NUM_DELAY3
#define CONFIG_DELAY3       0
#endif // DESIGN_NUM_DELAY3

// ================================================================================
// 2.Method模块使用的宏定义
#ifdef DESIGN_NUM_FLOAT
#define CONFIG_Float        DESIGN_NUM_FLOAT
#else // DESIGN_NUM_FLOAT
#define CONFIG_Float        0
#endif // DESIGN_NUM_FLOAT

#ifdef DESIGN_NUM_INT8
#define CONFIG_Int8         DESIGN_NUM_INT8
#else // DESIGN_NUM_INT8
#define CONFIG_Int8         0
#endif // DESIGN_NUM_INT8

#ifdef DESIGN_NUM_UINT8
#define CONFIG_Uint8        DESIGN_NUM_UINT8
#else // DESIGN_NUM_UINT8
#define CONFIG_Uint8        0
#endif // DESIGN_NUM_UINT8

#ifdef DESIGN_NUM_INT16
#define CONFIG_Int16        DESIGN_NUM_INT16
#else // DESIGN_NUM_INT16
#define CONFIG_Int16        0
#endif // DESIGN_NUM_INT16

#ifdef DESIGN_NUM_UINT16
#define CONFIG_Uint16       DESIGN_NUM_UINT16
#else // DESIGN_NUM_UINT16
#define CONFIG_Uint16       0
#endif // DESIGN_NUM_UINT16

#ifdef DESIGN_NUM_INT32
#define CONFIG_Int32        DESIGN_NUM_INT32
#else // DESIGN_NUM_INT32
#define CONFIG_Int32        0
#endif // DESIGN_NUM_INT32

#ifdef DESIGN_NUM_UINT32
#define CONFIG_Uint32       DESIGN_NUM_UINT32
#else // DESIGN_NUM_UINT32
#define CONFIG_Uint32       0
#endif // DESIGN_NUM_UINT32

// ================================================================================
// 3.MOTOR电机使用数量的宏定义
#ifndef DESIGN_COUNT_MOTOR
#define DESIGN_COUNT_MOTOR  1
#endif

#if !(DESIGN_COUNT_MOTOR >= 1 && DESIGN_COUNT_MOTOR <= 6)
#define CONFIG_MOTOR        1
#else // DESIGN_COUNT_MOTOR
#define CONFIG_MOTOR        DESIGN_COUNT_MOTOR
#endif // DESIGN_COUNT_MOTOR

// ================================================================================
// 4.控制系统离散周期的宏定义
#define PMSM_RUN_T          math_from_float((SguanF)DEFINE_TIM_T,VALUE_BASE_TIME)


#endif // SGUAN_CONFIG_H

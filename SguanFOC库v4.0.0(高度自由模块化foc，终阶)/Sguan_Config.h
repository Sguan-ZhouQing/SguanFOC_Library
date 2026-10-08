#ifndef __SGUAN_CONFIG_H
#define __SGUAN_CONFIG_H

/* SguanFOC配置文件声明 */
#include "Sguan_IQmath.h"
#include "Sguan_Math.h"

// ======================== Transfer模块使用 宏定义 =========================
#define CONFIG_TRANSFER1    UNITLIB_CONFIG_TRANSFER1
#define CONFIG_TRANSFER2    UNITLIB_CONFIG_TRANSFER2
#define CONFIG_TRANSFER3    UNITLIB_CONFIG_TRANSFER3
#define CONFIG_TRANSFER4    UNITLIB_CONFIG_TRANSFER4
#define CONFIG_TRANSFER5    UNITLIB_CONFIG_TRANSFER5
#define CONFIG_INTEGRATOR   UNITLIB_CONFIG_INTEGRATOR
#define CONFIG_DERIVATIVE   UNITLIB_CONFIG_DERIVATIVE
#define CONFIG_CURVE        UNITLIB_CONFIG_CURVE
#define CONFIG_DFT          UNITLIB_CONFIG_DFT
#define CONFIG_HALL         UNITLIB_CONFIG_HALL
#define CONFIG_LADRC1       UNITLIB_CONFIG_LADRC1
#define CONFIG_LADRC2       UNITLIB_CONFIG_LADRC2
#define CONFIG_SMC          UNITLIB_CONFIG_SMC
#define CONFIG_DPCC         UNITLIB_CONFIG_DPCC
#define CONFIG_PIR          UNITLIB_CONFIG_PIR
#define CONFIG_PID          UNITLIB_CONFIG_PID
#define CONFIG_PLL          UNITLIB_CONFIG_PLL
#define CONFIG_LPF1         UNITLIB_CONFIG_LPF1
#define CONFIG_LPF2         UNITLIB_CONFIG_LPF2
#define CONFIG_HPF1         UNITLIB_CONFIG_HPF1
#define CONFIG_HPF2         UNITLIB_CONFIG_HPF2
#define CONFIG_BPF1         UNITLIB_CONFIG_BPF1
#define CONFIG_BPF2         UNITLIB_CONFIG_BPF2
#define CONFIG_SOGI         UNITLIB_CONFIG_SOGI
#define CONFIG_NF           UNITLIB_CONFIG_NF
#define CONFIG_TPNF         UNITLIB_CONFIG_TPNF
#define CONFIG_DOB          UNITLIB_CONFIG_DOB
#define CONFIG_RLS          UNITLIB_CONFIG_RLS
#define CONFIG_SMO          UNITLIB_CONFIG_SMO
#define CONFIG_NLFO         UNITLIB_CONFIG_NLFO
#define CONFIG_VCFO         UNITLIB_CONFIG_VCFO
#define CONFIG_HFI          UNITLIB_CONFIG_HFI
#define CONFIG_ROLO         UNITLIB_CONFIG_ROLO
#define CONFIG_MARS         UNITLIB_CONFIG_MARS
#define CONFIG_EKF          UNITLIB_CONFIG_EKF
#define CONFIG_DELAY1       UNITLIB_CONFIG_DELAY1
#define CONFIG_DELAY2       UNITLIB_CONFIG_DELAY2
#define CONFIG_DELAY3       UNITLIB_CONFIG_DELAY3

// ======================== Value模块使用 宏定义 =========================
#define CONFIG_Float        UNITLIB_CONFIG_FLOAT
#define CONFIG_Int8         UNITLIB_CONFIG_INT8
#define CONFIG_Uint8        UNITLIB_CONFIG_UINT8
#define CONFIG_Int16        UNITLIB_CONFIG_INT16
#define CONFIG_Uint16       UNITLIB_CONFIG_UINT16
#define CONFIG_Int32        UNITLIB_CONFIG_INT32
#define CONFIG_Uint32       UNITLIB_CONFIG_UINT32
// ======================== Value模块使用 宏定义 =========================
#define CONFIG_MOTOR        UNITLIB_CONFIG_MOTOR


// ======================== 控制系统离散周期 宏定义 =========================
#define PMSM_RUN_T          TIM_T                       // 系统离散运行时间


// ........................... 宏定义保护措施 .............................
#if !(CONFIG_MOTOR >= 1 && CONFIG_MOTOR <= 6)
#ifdef CONFIG_MOTOR
#undef CONFIG_MOTOR
#endif // CONFIG_MOTOR
#define CONFIG_MOTOR 1  // 或根据你的需求设置
#endif


#endif // SGUAN_CONFIG_H

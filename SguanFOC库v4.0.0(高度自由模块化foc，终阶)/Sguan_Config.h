#ifndef __SGUAN_CONFIG_H
#define __SGUAN_CONFIG_H

/* SguanFOC配置文件声明 */
#include "Sguan_IQmath.h"
#include "Sguan_Math.h"
#include "UnitLib_Config.h"
#include "UserData_Config.h"


// ======================== Transfer模块使用 宏定义 =========================
#define CONFIG_TRANSFER1    CODE_DEFINE_TRANSFER1
#define CONFIG_TRANSFER2    CODE_DEFINE_TRANSFER2
#define CONFIG_TRANSFER3    CODE_DEFINE_TRANSFER3
#define CONFIG_TRANSFER4    CODE_DEFINE_TRANSFER4
#define CONFIG_TRANSFER5    CODE_DEFINE_TRANSFER5
#define CONFIG_INTEGRATOR   CODE_DEFINE_INTEGRATOR
#define CONFIG_DERIVATIVE   CODE_DEFINE_DERIVATIVE
#define CONFIG_CURVE        CODE_DEFINE_CURVE
#define CONFIG_DFT          CODE_DEFINE_DFT
#define CONFIG_HALL         CODE_DEFINE_HALL
#define CONFIG_LADRC1       CODE_DEFINE_LADRC1
#define CONFIG_LADRC2       CODE_DEFINE_LADRC2
#define CONFIG_SMC          CODE_DEFINE_SMC
#define CONFIG_DPCC         CODE_DEFINE_DPCC
#define CONFIG_PIR          CODE_DEFINE_PIR
#define CONFIG_PID          CODE_DEFINE_PID
#define CONFIG_PLL          CODE_DEFINE_PLL
#define CONFIG_LPF1         CODE_DEFINE_LPF1
#define CONFIG_LPF2         CODE_DEFINE_LPF2
#define CONFIG_HPF1         CODE_DEFINE_HPF1
#define CONFIG_HPF2         CODE_DEFINE_HPF2
#define CONFIG_BPF1         CODE_DEFINE_BPF1
#define CONFIG_BPF2         CODE_DEFINE_BPF2
#define CONFIG_SOGI         CODE_DEFINE_SOGI
#define CONFIG_NF           CODE_DEFINE_NF
#define CONFIG_TPNF         CODE_DEFINE_TPNF
#define CONFIG_DOB          CODE_DEFINE_DOB
#define CONFIG_RLS          CODE_DEFINE_RLS
#define CONFIG_SMO          CODE_DEFINE_SMO
#define CONFIG_NLFO         CODE_DEFINE_NLFO
#define CONFIG_VCFO         CODE_DEFINE_VCFO
#define CONFIG_HFI          CODE_DEFINE_HFI
#define CONFIG_ROLO         CODE_DEFINE_ROLO
#define CONFIG_MARS         CODE_DEFINE_MARS
#define CONFIG_EKF          CODE_DEFINE_EKF
#define CONFIG_DELAY1       CODE_DEFINE_DELAY1
#define CONFIG_DELAY2       CODE_DEFINE_DELAY2
#define CONFIG_DELAY3       CODE_DEFINE_DELAY3

// ======================== Value模块使用 宏定义 =========================
#define CONFIG_Float        CODE_DEFINE_FLOAT
#define CONFIG_Int8         CODE_DEFINE_INT8
#define CONFIG_Uint8        CODE_DEFINE_UINT8
#define CONFIG_Int16        CODE_DEFINE_INT16
#define CONFIG_Uint16       CODE_DEFINE_UINT16
#define CONFIG_Int32        CODE_DEFINE_INT32
#define CONFIG_Uint32       CODE_DEFINE_UINT32
// ======================== Value模块使用 宏定义 =========================
// #define CONFIG_IQMATH       DATA_DEFINE_IQMATH
#define CONFIG_Q31          0x01
#define CONFIG_Q15          0x02


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

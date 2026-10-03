#ifndef __SGUAN_CONFIG_H
#define __SGUAN_CONFIG_H

/* SguanFOC配置文件声明 */
#include "Sguan_IQmath.h"

// ============================ 系统配置 宏定义 ============================
#define CONFIG_MODE         Define_Run_Mode
#define CONFIG_CUR          Current_Offset_Open

// ============================ Q31标幺化 宏定义 ===========================
// 标幺化基准值(Config)
#define BASE_Time           Q_Time
#define BASE_Rad            Q_Rad
#define BASE_Current        Q_Current
#define BASE_Voltage        Q_Voltage
#define BASE_Speed          Q_Speed
#define BASE_Hz             Q_Hz
#define BASE_Inductor       Q_Inductor
#define BASE_Flux           Q_Flux
#define BASE_Resistor       Q_Resistor

#define BASE_SUB            32.0f

// ======================== 控制系统离散周期 宏定义 =========================
// 离散控制周期大小
// #define PMSM_RUN_T_q31   iqmath_from_float(TIM_T,BASE_Time)
// #define Dead_T_q31       iqmath_from_float(Dead_Time,BASE_Time)
// #define ADC_T_q31        iqmath_from_float(ADC_Time,BASE_Time)
// (这些Q31的数值定义，都放在了Sguan_Value.h中)
#define PMSM_RUN_T          TIM_T



#endif // SGUAN_CONFIG_H

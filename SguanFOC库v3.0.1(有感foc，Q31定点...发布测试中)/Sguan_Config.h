#ifndef __SGUAN_CONFIG_H
#define __SGUAN_CONFIG_H

#include "Sguan_IQmath.h"

// ============================ 系统配置 宏定义 ============================
#define CONFIG_PI           Open_PI_Control
#define CONFIG_MTPA         Open_MTPA_Calculate
#define CONFIG_FW           Open_FW_Calculate
#define CONFIG_Q31          Open_Q31_Calculate
#define CONFIG_Debug        Open_Printf_Debug


// ============================ Q31标幺化 宏定义 ===========================
// 标幺化基准值(Config)
#define BASE_Time           Q_Time
#define BASE_Rad            Q_Rad
#define BASE_Speed          Q_Speed
#define BASE_Hz             Q_Hz
#define BASE_Current        Q_Current
#define BASE_Voltage        Q_Voltage
#define BASE_Inductor       Q_Inductor
#define BASE_Flux           Q_Flux

// 默认数值缩放倍率(Sguan_Fiter)
#define BASE_Filter_Num     16.0f
#define SHIFT_Filter_Num    4

// 默认数值缩放倍率(Sguan_PLL)
#define BASE_PLL_Num        16.0f
#define INT32_PLL_Num_16    134217728
#define SHIFT_PLL_Num       4
// #define BASE_Num         16.0f
// #define INT32_Num_16     IQmath_Q31_from_float(1.0f,BASE_Num)
// #define SHIFT_Num        4


// ======================== 控制系统离散周期 宏定义 =========================
// 离散控制周期大小
#define PMSM_RUN_T          TIM_T


#endif // SGUAN_CONFIG_H

#ifndef __SGUAN_CONFIG_H
#define __SGUAN_CONFIG_H

#include "Sguan_math.h"
#include "UserData_Config.h"

// ============================ 系统配置 宏定义 ============================
#define CONFIG_CtrlVel      Switch_Control_Velocity
#define CONFIG_CtrlPos      Switch_Control_Position
#define CONFIG_PWM          Switch_PWM_Calculate
#define CONFIG_Identify     Switch_MOTOR_Identify
#define CONFIG_CurFF        Open_Current_Feedforward
#define CONFIG_VelFF        Open_Velocity_Feedforward
#define CONFIG_DOB          Open_DOB_Calculate
#define CONFIG_DeadZone     Open_DeadZone_Calculate
#define CONFIG_MTPA         Open_MTPA_Calculate
#define CONFIG_FW           Open_FW_Calculate
#define CONFIG_Debug        Open_Printf_Debug

// ============================ 安全边界 宏定义 ============================
#define PMSM_MAX_Ctrl       3   // 控制模式PID,LADRC,SMC,STA这4种(0-3)

// ======================== 控制系统离散周期 宏定义 =========================
// 离散控制周期大小
#define PMSM_RUN_T          TIM_T


#endif // SGUAN_CONFIG_H

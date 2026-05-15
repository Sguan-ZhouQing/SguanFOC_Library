#ifndef __SGUAN_CONFIG_H
#define __SGUAN_CONFIG_H

#include "Sguan_IQmath.h"
#include "Sguan_math.h"
#include "UserData_Config.h"

// ============================ 系统配置 宏定义 ==============================
#define CONFIG_MODE         Define_Run_Mode             // (定义)电机运行模式
#define CONFIG_Debug        Define_Run_Debug            // (定义)无感算法调试
#define CONFIG_CtrlVel      Switch_Control_Velocity     // (选择)转速环控制器
#define CONFIG_CtrlPos      Switch_Control_Position     // (选择)位置环控制器
#define CONFIG_PWM          Switch_PWM_Calculate        // (选择)PWM调制技术
#define CONFIG_Filter       Switch_Filter_Calculate     // (选择)不同滤波算法
#define CONFIG_Identify     Switch_MOTOR_Identify       // (选择)电机参数辨识
#define CONFIG_AngleComp    Switch_AngleComp_Calculate  // (选择)相位延迟补偿
#define CONFIG_CurFF        Open_Current_Feedforward    // (开关)电流前馈解耦
#define CONFIG_VelFF        Open_Velocity_Feedforward   // (开关)速度前馈解耦
#define CONFIG_DOB          Open_DOB_Calculate          // (开关)扰动观测器
#define CONFIG_MTPA         Open_MTPA_Calculate         // (开关)最大力矩控制
#define CONFIG_FW           Open_FW_Calculate           // (开关)弱磁控制
#define CONFIG_DeadZone     Open_DeadZone_Calculate     // (开关)死区补偿算法
#define CONFIG_Printf       Open_Printf_Debug           // (开关)实时打印状态机
#define CONFIG_Cogging      Open_Cogging_Calculate      // (开关)抗齿槽算法
#define CONFIG_BASE         BASE_Cogging_Num            // (基值)抗齿槽Q轴电流标定

// ============================ 安全边界 宏定义 ============================
#define PMSM_MAX_Ctrl       3                           // 控制模式PID,LADRC,SMC,STA这4种(0-3)

// ======================== 控制系统离散周期 宏定义 =========================
// 离散控制周期大小
#define PMSM_RUN_T          TIM_T                       // 系统离散运行时间


#endif // SGUAN_CONFIG_H

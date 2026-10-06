#ifndef __SGUANFOC_H
#define __SGUANFOC_H

/* USER CODE BEGIN Includes */
// 电机控制核心函数文件声明
#include "Sguan_Config.h"
#include "Sguan_LPF.h"
#include "Sguan_MotorStatus.h"
#include "Sguan_NLFO.h"
#include "Sguan_PID.h"
#include "Sguan_PLL.h"
#include "Sguan_Printf.h"
#include "Sguan_SingleRs.h"
#include "Sguan_SVPWM.h"
/* USER CODE END Includes */

// 无感算法实现
// (其中MODE_NLFO_Speed0和MODE_NLFO_Speed1效果一般)
// (本代码暂时不去实现)
// (其他模块可以正常使用：模式0-3)
#define MODE_VF_Only                0x00        // VF控制(纯开环控制)
#define MODE_IF_Only                0x01        // IF控制(纯开环控制)
#define MODE_NLFO_Voltag            0x02        // NLFO单电压开环
#define MODE_NLFO_Vel               0x03        // NLFO零速双闭环
#define MODE_NLFO_Speed0            0x04        // 磁链双闭环(VF切磁链观测器)
#define MODE_NLFO_Speed1            0x05        // 磁链双闭环(IF切磁链观测器)

typedef struct{
    // ....................... [LPF低通滤波器] .......................
    LPF_STRUCT LPF_D;                           // (LPF)电流D轴滤波
    LPF_STRUCT LPF_Q;                           // (LPF)电流Q轴滤波
    LPF_STRUCT LPF_Speed;                       // (LPF)机械角速度滤波

    LPF_STRUCT LPF_Ltd;                         // (LPF)最速控制

    // ....................... [PID闭环控制器] .......................
    PID_STRUCT PID_D;                           // (PID)电流环D轴
    PID_STRUCT PID_Q;                           // (PID)电流环Q轴
    PID_STRUCT PID_Speed;                       // (PID)速度外环

    uint8_t Response;                           // (PID)内外环控制倍率

    // ....................... [PLL角度锁相环] .......................
    PLL_STRUCT PLL;                             // (PLL)锁相环

    // ....................... [NLFO无感磁链] ........................
    NLFO_STRUCT NLFO;                           // (NLFO)无感磁链观测器
}Transfer_STRUCT;

typedef struct{
    // ....................... [Encoder电机角度相关] .................
    Q31_t Real_Speed;                           // (Encoder)机械角速度
    Q31_t Real_We;                              // (Encoder)电子角速度
    Q31_t Real_Re;                              // (Encoder)电子角度

    // ....................... [Current电机电流相关] .................
    Q31_t Real_Id;                              // (Current)D轴电流
    Q31_t Real_Iq;                              // (Current)Q轴电流

    Q31_t Real_Id_temp;                         // (Current)D轴电流_临时
    Q31_t Real_Iq_temp;                         // (Current)Q轴电流_临时

    Q31_t Real_Ia;                              // (Current)U相电流数据
    Q31_t Real_Ib;                              // (Current)V相电流数据
    Q31_t Real_Ic;                              // (Current)W相电流数据

    Q31_t Real_Ialpha;                          // (Current)alpha轴电流
    Q31_t Real_Ibeta;                           // (Current)beta轴电流

    int8_t Current_Dir;                        // (Current)电流方向
    int16_t Current_Offset0;                   // (Current)电流偏置
    int16_t Current_Offset1;                   // (Current)电流偏置
    int16_t Current_Offset2;                   // (Current)电流偏置

    // ....................... [MOTOR电机本体相关] ...................
    uint8_t Poles;                              // (MOTOR)电机极对数
    uint16_t Duty;                              // (MOTOR)PWM满计数值

    float Rs;                                   // (MOTOR)电机相电阻
    float Ld;                                   // (MOTOR)电机D轴电感
    float Lq;                                   // (MOTOR)电机Q轴电感
    float Flux;                                 // (MOTOR)电机磁链
}Motor_STRUCT;

typedef struct{
    // ....................... [Target期望数值设定] ...................
    Q31_t Target_Speed;                         // (Target)期望机械角速度
    Q31_t Target_Uq;                            // (Target)期望Q轴电压

    Q31_t Target_VF_Uq;                         // (Target)VF控制
    Q31_t Target_IF_Iq;                         // (Target)IF控制
    
    Q31_t Target_Id;                            // (Target)期望D轴电流
    Q31_t Target_Iq;                            // (Target)期望Q轴电流
    // ....................... [Real实际数值设定] .....................
    Q31_t Speed_in;                             // (Real)实际输入的数值
    Q31_t Ud_in;                                // (Real)D轴电压
    Q31_t Uq_in;                                // (Real)Q轴电压

    Q31_t Ualpha;                               // (Real)alpha轴电压
    Q31_t Ubeta;                                // (Real)beta轴电压

    Q31_t Du;                                   // (Real)归一化占空比数值
    Q31_t Dv;                                   // (Real)归一化占空比数值
    Q31_t Dw;                                   // (Real)归一化占空比数值

    uint16_t Duty_u;                            // (Real)PWM比较器数值
    uint16_t Duty_v;                            // (Real)PWM比较器数值
    uint16_t Duty_w;                            // (Real)PWM比较器数值

    Q31_t Sine;                                 // (Real)正弦数值
    Q31_t Cosine;                               // (Real)余弦数值

    Q31_t VBUS;                                 // (Real)电机当前电压
}Foc_STRUCT;

typedef struct{
    // ...................... [float->Foc] .........................
    float Target_Speed;                         // (float->Foc)输入测试
    float Target_Uq;                            // (float->Foc)输入测试

    float Target_VF_Uq;                         // (float->Foc)输入测试
    float Target_IF_Iq;                         // (float->Foc)输入测试

    float Target_Id;                            // (float->Foc)输入测试
    float Target_Iq;                            // (float->Foc)输入测试

    // ...................... [float->Motor] .........................
    float Real_Speed;                           // (float->Foc)输出测试
    float Real_Uq;                              // (float->Foc)输出测试
    float Real_Re;                              // (float->Foc)输出测试

    float Real_Id;                              // (float->Foc)输出测试
    float Real_Iq;                              // (float->Foc)输出测试
}Float_STRUCT;

typedef struct{
    // ....................... [Safe实际数据获取] .....................
    uint16_t Vbus_Real;                         // (Safe)ADC采样原始数值
    uint16_t Temp_Real;                         // (Safe)ADC采样原始数值
    uint16_t Ibus_Real;                         // (Safe)ADC采样原始数值

    // ....................... [Limit保护动作处理] ....................
    uint16_t Vbus_Max;                          // (Limit)Vbus阈值
    uint16_t Vbus_Min;                          // (Limit)Vbus阈值

    uint16_t Temp_Max;                          // (Limit)Temp阈值
    uint16_t Temp_Min;                          // (Limit)Temp阈值

    uint16_t Ibus_Max;                          // (Limit)Ibus阈值
}Safe_STRUCT;

typedef struct{
    uint8_t Status;                             // [数据]status存储电机运行状态
    uint8_t Error_Code;                         // [数据]Error_Code存储电机错误状态

    Transfer_STRUCT Transfer;                   // [嵌套结构体]传递函数模块
    Motor_STRUCT Motor;                         // [嵌套结构体]电机参数
    Foc_STRUCT Foc;                             // [嵌套结构体]FOC运行参数
    Float_STRUCT Float;                         // [嵌套结构体]Float便捷浮点
    Safe_STRUCT Safe;                           // [嵌套结构体]电机保护

    PRINTF_STRUCT Printf;                       // [嵌套结构体]调参和波形打印
}SguanFOC_System_STRUCT;

// 电机控制核心结构体声明
extern SguanFOC_System_STRUCT Sguan;

void SguanFOC_High_Loop(void);
void SguanFOC_Low_Loop(void);
void SguanFOC_Printf_Loop(uint8_t *data, uint16_t length);
void SguanFOC_main_Loop(void);


#endif // SGUANFOC_H

#ifndef __SGUANFOC_H
#define __SGUANFOC_H

/* USER CODE BEGIN Includes */
// 电机控制核心函数文件声明
#include "Sguan_Filter.h"
#include "Sguan_Identify.h"
#include "Sguan_InternalModel.h"
#include "Sguan_Ladrc.h"
#include "Sguan_math.h"
#include "Sguan_MotorStatus.h"
#include "Sguan_PID.h"
#include "Sguan_PLL.h"
#include "Sguan_printf.h"
#include "Sguan_SensorlessHFI.h"
#include "Sguan_SensorlessSMO.h"
/* USER CODE END Includes */

#define Velocity_OPEN_MODE      0x00 // 速度开环(Uq_in电机方向测试和电机参数测算)
#define Current_SINGLE_MODE     0x01 // 电流单闭环(力矩控制)
#define VelCur_DOUBLE_MODE      0x02 // 速度-电流串级闭环控制
#define PosVelCur_THREE_MODE    0x03 // 位置-速度-电流多环
#define Sensorless_HFI_MODE     0x04 // 无感高频方波注入控制(低速域)
#define Sensorless_SMO_MODE     0x05 // 无感滑膜观测器控制(强拖切高速域)
#define Sensorless_HS_MODE      0x06 // 无感方波注入切滑膜(全速域)

typedef struct{
    uint8_t PWM_Calc;               // PWM计算标志位
    uint8_t PWM_watchdog_limit;     // PWM错误限幅(uint8_t够用，PWM计算不能错误太多次)

    uint8_t in_PWM_Calc_ISR;        // (互斥锁)标记是否在PWM计算中断中
}MOTOR_FLAG_STRUCT;

typedef struct{
    BPF_STRUCT CurrentD;            // (电流数据)电机D轴滤波
    BPF_STRUCT CurrentQ;            // (电流数据)电机Q轴滤波
    BPF_STRUCT Encoder;             // (速度数据)速度信号滤波
}MOTOR_BPF_STRUCT;

typedef struct{
    #if !Open_PI_Control
    INTERNALMODEL_STRUCT Dimc;      // (IMC)电流环调优
    INTERNALMODEL_STRUCT Qimc;      // (IMC)电流环调优
    PID_STRUCT Internal_D;          // (电流内模控制)PI控制器D轴参数
    PID_STRUCT Internal_Q;          // (电流内模控制)PI控制器Q轴参数
    LADRC_STRUCT Speed;             // (LADRC)线自抗扰控制
    #else // Open_PI_Control
    PID_STRUCT Current_D;           // (电流单环)PID电流环D轴参数
    PID_STRUCT Current_Q;           // (电流单环)PID电流环Q轴参数
    PID_STRUCT Velocity;            // (速度-电流双环)双PID速度外环参数
    #endif // Open_PI_Control
    PID_STRUCT Position;            // (高性能伺服三环)Position

    uint8_t Response;               // (参数设计)响应带宽倍数
}MOTOR_CONTROL_STRUCT;

typedef struct{
    float Ld;                       // (电机实体参数)D轴电感
    float Lq;                       // (电机实体参数)Q轴电感
    float Ls;                       // (电机实体参数)相线电感
    float Rs;                       // (电机实体参数)相线电阻
    float Flux;                     // (电机实体参数)电机磁链    
}MOTOR_IDENTIFY_STRUCT;

typedef struct{
    uint8_t Poles;                  // (电机实体参数)电机极对极数
    float VBUS;                     // (电机实体参数)母线电压

    int8_t Motor_Dir;               // (参数设计)电机的运行方向设计
    int8_t PWM_Dir;                 // (参数设计)PWM占空比高低对应
    uint16_t Duty;                  // (参数设计)PWM满占空比

    float Encoder_T;                // (参数设计)编码器的离散周期
    int8_t Encoder_Dir;             // (参数设计)编码器的方向设置

    int8_t Current_Dir0;            // (参数设计)电流采样方向0
    int8_t Current_Dir1;            // (参数设计)电流采样方向1
    uint8_t Current_Num;            // (参数设计)电流通道0->AB相，1->AC相，2->BC相
    uint32_t ADC_Precision;         // (参数设计)ADC采样精度,如12位精度为4096
    float Amplifier;                // (参数设计)运放的放大倍数
    float MCU_Voltage;              // (参数设计)DSP/单片机的ADC基准电压
    float Sampling_Rs;              // (参数设计)采样电阻的阻值大小
}MOTOR_QUANTIZE_STRUCT;

typedef struct{    
    float VBUS_MAX;                 // (参数设计)母线电压值波动MAX阈值
    float VBUS_MIM;                 // (参数设计)母线电压值波动MIN阈值
    uint32_t VBUS_watchdog_limit;   // (参数设计)电压异常的警告周期
    
    float Temp_MAX;                 // (参数设计)驱动器允许最大温度
    float Temp_MIN;                 // (参数设计)驱动器允许最小温度
    uint32_t Temp_watchdog_limit;   // (参数设计)温度异常的警告周期

    float Dcur_MAX;                 // (参数设计)电机最大电流D轴限制
    float Qcur_MAX;                 // (参数设计)电机最大电流Q轴限制
    uint32_t DQcur_watchdog_limit;  // (参数设计)过流保护的警告周期

    uint32_t DISABLED_watchdog_limit;//(参数设计)电机DISABLED状态机进待机模式的延时周期
}MOTOR_SAFE_STRUCT;

typedef struct{
    float Target_Speed;             // (期望速度)Target期望机械角速度
    double Target_Pos;              // (期望角度)Target期望机械角度
    float Target_Id;                // (期望电流)期望D轴电流
    float Target_Iq;                // (期望电流)期望Q轴电流

    float Ud_in;                    // (输入值)D轴电压输入
    float Uq_in;                    // (输入值)Q轴电压输入

    uint16_t Duty_u;                // (输入值)U相占空比输入0~pwmMAX
    uint16_t Duty_v;                // (输入值)V相占空比输入0~pwmMAX
    uint16_t Duty_w;                // (输入值)W相占空比输入0~pwmMAX

    float Du;                       // (数据)U相占空比输入0~1
    float Dv;                       // (数据)V相占空比输入0~1
    float Dw;                       // (数据)W相占空比输入0~1

    float sine;                     // (数据)sine临时保存的正弦值
    float cosine;                   // (数据)cosine临时保存的余弦值

    float Real_VBUS;                // (数据)Real实际的电机母线电压
    float Real_Temp;                // (数据)Temp实际的驱动器物理温度
}MOTOR_FOC_STRUCT;

typedef struct{
    PLL_STRUCT pll;                 // (PLL锁相环)锁相环结构体

    float Real_Speed;               // (Encoder速度)Real实际机械角速度
    double Real_Pos;                // (Encoder多圈角度)Real实际机械角度
    float Real_Rad;                 // (Encoder单圈角度)Real实际机械角度
    float Real_Erad;                // (Encoder电角度)Real实际电子角度
    float Real_Espeed;              // (Encoder电速度)Real实际电子角速度

    float Pos_offset;               // (Encoder角度偏置)offset偏置位
}MOTOR_ENCODER_STRUCT;

typedef struct{
    float Real_Id;                  // (Current电流)Real实际D轴电流
    float Real_Iq;                  // (Current电流)Real实际Q轴电流

    float Real_Ia;                  // (Current相电流)A相电流
    float Real_Ib;                  // (Current相电流)B相电流
    float Real_Ic;                  // (Current相电流)C相电流

    float Real_Ialpha;              // (Current中间量电流)alpha轴电流
    float Real_Ibeta;               // (Current中间量电流)beta轴电流

    float Final_Gain;               // (ADC增益)最终的ADC电流采样增益
    int32_t Pos_offset0;            // (Current电流偏置)offset偏置位
    int32_t Pos_offset1;            // (Current电流偏置)offset偏置位
}MOTOR_CURRENT_STRUCT;

typedef struct{
    float PMSM_RUN_T;               // 【有参数设计】系统电机运行时间周期
    uint8_t mode;                   // 【有参数设计】mode选择电机的运行模式
    uint8_t status;                 // 【数据】status存储电机运行状态
    
    MOTOR_FLAG_STRUCT flag;         // 【有参数设计】flag电机运行标志位
    MOTOR_BPF_STRUCT bpf;           // 【有参数设计】bpf低通滤波器设计
    MOTOR_CONTROL_STRUCT control;   // 【有参数设计】pid闭环控制系统设计
    MOTOR_IDENTIFY_STRUCT identify; // 【数据】identify电机参数辨识结果
    MOTOR_QUANTIZE_STRUCT motor;    // 【有参数设计】motor电机参数量化
    MOTOR_SAFE_STRUCT safe;         // 【有参数设计】safe电机安全设置
    MOTOR_FOC_STRUCT foc;           // 【有参数设计】foc控制的参数输入“缓存”
    MOTOR_ENCODER_STRUCT encoder;   // 【数据】电机角速度和角度信息“缓存”
    MOTOR_CURRENT_STRUCT current;   // 【数据】电机电流采样信息“缓存”
    PRINTF_STRUCT TXdata;           // 【数据】data串口或CAN发送的信息
    HFI_STRUCT hfi;                 // 【数据】HFI高频注入算法
    SMO_STRUCT smo;                 // 【数据】SMO滑膜观测器算法
}SguanFOC_System_STRUCT;

// 电机控制核心结构体声明
extern SguanFOC_System_STRUCT Sguan;

void SguanFOC_Loop(void);
void SguanFOC_msTick(void);
void SguanFOC_PrintfTick(uint8_t *data, uint16_t length);
void SguanFOC_mainTick(void);


#endif // SGUANFOC_H

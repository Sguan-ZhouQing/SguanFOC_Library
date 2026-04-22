#ifndef __SGUANFOC_H
#define __SGUANFOC_H

/* USER CODE BEGIN Includes */
// 电机控制核心函数文件声明
#include "Sguan_DOB.h"                  // DOB超螺旋滑模扰动观测器
#include "Sguan_Feedforward.h"          // Feedforward前馈环节
#include "Sguan_Filter.h"               // Filter巴特沃斯滤波器
#include "Sguan_Hall.h"                 // Hall三霍尔信号处理
#include "Sguan_Identify.h"             // Identify电机参数辨识
#include "Sguan_Ladrc.h"                // Ladrc线自抗扰控制
#include "Sguan_MotorStatus.h"          // MotorStatus电机状态机
#include "Sguan_Optimize.h"             // Optimize电机优化算法
#include "Sguan_PID.h"                  // PID传统闭环控制
#include "Sguan_PLL.h"                  // PLL角度跟踪“锁相环”
#include "Sguan_printf.h"               // printf通信调试
#include "Sguan_SMC.h"                  // SMC传统滑模控制
#include "Sguan_SPWM.h"                 // SPWM零序分量注入法的PWM调制
#include "Sguan_STA.h"                  // STA超螺旋简化滑模控制
#include "Sguan_SVPWM.h"                // SVPWM七段式空间矢量PWM调制
/* USER CODE END Includes */

// ╔═══════════════════════════════════════════════════════╗
// ║                       开环控制模式                     ║
// ╚═══════════════════════════════════════════════════════╝
#define VF_OPENLOOP_MODE        0x00    // VF压频比开环(Sguan.foc.Target_Speed，Sguan.foc.Uq_in)
#define IF_OPENLOOP_MODE        0x01    // IF流频比开环(Sguan.foc.Target_Speed，Sguan.foc.Target_Iq)
// ╔═══════════════════════════════════════════════════════╗
// ║                   闭环控制模式（有传感器）              ║
// ╚═══════════════════════════════════════════════════════╝
#define Voltag_OPEN_MODE        0x02    // 电压开环(Sguan.foc.Uq_in)
#define Current_SINGLE_MODE     0x03    // 电流单闭环(Sguan.foc.Target_Iq)
#define VelCur_DOUBLE_MODE      0x04    // 速度-电流串级闭环(Sguan.foc.Target_Speed)
#define PosVelCur_THREE_MODE    0x05    // 位置-速度-电流三环(Sguan.foc.Target_Pos)
// ╔═══════════════════════════════════════════════════════╗
// ║                   霍尔控制模式（有传感器）              ║
// ╚═══════════════════════════════════════════════════════╝
#define Sensor_Hall_MODE        0x06    // 有感霍尔_转速环(Sguan.foc.Target_Speed)
// ╔═══════════════════════════════════════════════════════╗
// ║                   无感控制模式（无传感器）              ║
// ╚═══════════════════════════════════════════════════════╝
#define Sensorless_HFI_MODE     0x07    // 高频注入_转速环(Sguan.foc.Target_Speed)
#define Sensorless_SMO_MODE     0x08    // 滑模观测_转速环(Sguan.foc.Target_Speed)
#define Sensorless_HS_MODE      0x09    // 前两结合_转速环(Sguan.foc.Target_Speed)

typedef struct{
    uint8_t PWM_Calc;                   // PWM计算“标志位”
    uint8_t PWM_watchdog_limit;         // PWM错误限幅(PWM计算不能中断错误太多次)
    uint8_t in_PWM_Calc_ISR;            // (互斥锁)标记是否在PWM计算中断中
}MOTOR_FLAG_STRUCT;

typedef struct{
    // ====================== 1.传递函数“控制器” ===========================
    PID_STRUCT Current_D;               // (电流单环)PID电流环D轴参数
    PID_STRUCT Current_Q;               // (电流单环)PID电流环Q轴参数

    #if CONFIG_CtrlVel==1
    LADRC_STRUCT Velocity;              // (速度-电流双环)LADRC速度外环参数
    #elif CONFIG_CtrlVel==2
    SMC_STRUCT Velocity;                // (速度-电流双环)SMC速度外环参数
    #elif CONFIG_CtrlVel==3
    STA_STRUCT Velocity;                // (速度-电流双环)STA速度外环参数
    #else  // CONFIG_CtrlVel
    PID_STRUCT Velocity;                // (速度-电流双环)双PID速度外环参数
    #endif // CONFIG_CtrlVel

    #if CONFIG_CtrlPos==1
    LADRC_STRUCT Position;              // (高性能伺服三环)Position的LADRC
    #elif CONFIG_CtrlPos==2
    SMC_STRUCT Position;                // (高性能伺服三环)Position的SMC
    #elif CONFIG_CtrlPos==3
    STA_STRUCT Position;                // (高性能伺服三环)Position的STA
    #else  // CONFIG_CtrlPos
    PID_STRUCT Position;                // (高性能伺服三环)Position的PID
    #endif // CONFIG_CtrlPos

    uint8_t Response;                   // (参数设计)响应带宽倍数

    // ====================== 2.传递函数“滤波器” ===========================
    LPF_STRUCT LPF_D;                   // (电流数据)电机D轴滤波
    LPF_STRUCT LPF_Q;                   // (电流数据)电机Q轴滤波
    LPF_STRUCT LPF_encoder;             // (速度数据)速度信号滤波

    // ==================== 3.传递函数“锁相环及观测器” ======================
    PLL_STRUCT PLL;                     // (PLL锁相环)角度跟踪锁相环
    #if CONFIG_DOB
    DOB_STRUCT DOB;                     // (超螺旋滑模扰动观测器)DOB
    #endif // CONFIG_DOB
}MOTOR_TRANSFER_STRUCT;

typedef struct{
    IDENTIFY_STRUCT identify;           // (参数辨识)电机的核心实体参数

    uint8_t Poles;                      // (电机实体参数)电机极对数
    float VBUS;                         // (电机实体参数)母线电压

    int8_t Motor_Dir;                   // (参数设计)电机的运行方向设计
    int8_t PWM_Dir;                     // (参数设计)PWM占空比高低对应
    uint32_t Duty;                      // (参数设计)PWM满占空比

    int8_t Current_Dir0;                // (参数设计)电流采样方向0
    int8_t Current_Dir1;                // (参数设计)电流采样方向1
    uint8_t Current_Num;                // (参数设计)电流通道0->AB相，1->AC相，2->BC相
    uint32_t ADC_Precision;             // (参数设计)ADC采样精度,如12位精度为4096
    float Amplifier;                    // (参数设计)运放的放大倍数
    float MCU_Voltage;                  // (参数设计)DSP/单片机的ADC基准电压
    float Sampling_Rs;                  // (参数设计)采样电阻的阻值大小
}MOTOR_QUANTIZE_STRUCT;

typedef struct{    
    float VBUS_MAX;                     // (参数设计)母线电压值波动MAX阈值
    float VBUS_MIM;                     // (参数设计)母线电压值波动MIN阈值
    uint32_t VBUS_watchdog_limit;       // (参数设计)电压异常的警告周期
    // 如果有电机电压预警，电机正常运行...经过Sguan_Low_Loop()的VBUS_watchdog_limit此运行周期
    // 若2-10次，中间有一次再触发电压警告，电机停转进待机
    // 若首次后10次，都未再触发，电机以后都正常运行
    
    float Temp_MAX;                     // (参数设计)驱动器允许最大温度
    float Temp_MIN;                     // (参数设计)驱动器允许最小温度
    uint32_t Temp_watchdog_limit;       // (参数设计)温度异常的警告周期
    // 如果有驱动器温度预警，电机正常运行...经过Sguan_Low_Loop()的Temp_watchdog_limit此运行周期
    // 若2-10次，中间有一次再触发温度警告，电机停转进待机
    // 若首次后10次，都未再触发，电机以后都正常运行
 
    float Dcur_MAX;                     // (参数设计)电机最大电流D轴限制
    float Qcur_MAX;                     // (参数设计)电机最大电流Q轴限制
    uint32_t DQcur_watchdog_limit;      // (参数设计)过流保护的警告周期
    // 如果有电机过流预警，电机正常运行...经过Sguan_Low_Loop()的DQcur_watchdog_limit此运行周期
    // 若2-10次，中间有一次再触发过流警告，电机停转进待机
    // 若首次后10次，都未再触发，电机以后都正常运行

    float Current_limit;                // (参数设计)电流正负区间设计，电流状态机判断
    float Speed_limit;                  // (参数设计)速度正负区间设计，速度状态机判断
    float Position_limit;               // (参数设计)位置正负区间设计，位置状态机判断

    uint32_t DISABLED_watchdog_limit;   //(参数设计)电机DISABLED状态机进待机模式的延时周期
}MOTOR_SAFE_STRUCT;

typedef struct{
    float Target_Speed;                 // (期望速度)Target期望机械角速度
    float Target_Pos;                   // (期望角度)Target期望机械角度
    float Target_Id;                    // (期望电流)期望D轴电流
    float Target_Iq;                    // (期望电流)期望Q轴电流

    float Ud_in;                        // (输入值)D轴电压输入
    float Uq_in;                        // (输入值)Q轴电压输入

    uint32_t Duty_u;                    // (输入值)U相占空比输入0~pwmMAX
    uint32_t Duty_v;                    // (输入值)V相占空比输入0~pwmMAX
    uint32_t Duty_w;                    // (输入值)W相占空比输入0~pwmMAX

    float Du;                           // (数据)U相占空比输入0~1
    float Dv;                           // (数据)V相占空比输入0~1
    float Dw;                           // (数据)W相占空比输入0~1

    float sine;                         // (数据)sine临时保存的正弦值
    float cosine;                       // (数据)cosine临时保存的余弦值

    float Real_VBUS;                    // (数据)Real实际的电机母线电压
    float Real_Temp;                    // (数据)Temp实际的驱动器物理温度
}MOTOR_FOC_STRUCT;

typedef struct{
    float Real_Speed;                   // (Encoder速度)Real实际机械角速度
    double Real_Pos;                    // (Encoder多圈角度)Real实际机械角度
    float Real_Rad;                     // (Encoder单圈角度)Real实际机械角度
    float Real_Erad;                    // (Encoder电角度)Real实际电子角度
    float Real_Espeed;                  // (Encoder电速度)Real实际电子角速度

    float Pos_offset;                   // (Encoder角度偏置)offset偏置位
}MOTOR_ENCODER_STRUCT;

typedef struct{
    float Real_Id;                      // (Current电流)Real实际D轴电流
    float Real_Iq;                      // (Current电流)Real实际Q轴电流

    float Real_Ia;                      // (Current相电流)A相电流
    float Real_Ib;                      // (Current相电流)B相电流
    float Real_Ic;                      // (Current相电流)C相电流

    float Real_Ialpha;                  // (Current中间量电流)alpha轴电流
    float Real_Ibeta;                   // (Current中间量电流)beta轴电流

    float Final_Gain;                   // (ADC增益)最终的ADC电流采样增益
    int32_t Current_offset0;            // (Current电流偏置)offset偏置位
    int32_t Current_offset1;            // (Current电流偏置)offset偏置位
}MOTOR_CURRENT_STRUCT;

typedef struct{
    uint8_t status;                     // 【数据】status存储电机运行状态
    uint8_t mode;                       // 【有参数设计】mode选择电机的运行模式
    
    MOTOR_TRANSFER_STRUCT transfer;     // 【有参数设计】Transfer传递函数
    
    MOTOR_QUANTIZE_STRUCT motor;        // 【有参数设计】motor电机参数量化
    MOTOR_SAFE_STRUCT safe;             // 【有参数设计】safe电机安全设置
    MOTOR_FLAG_STRUCT flag;             // 【有参数设计】flag电机运行标志位
    
    MOTOR_FOC_STRUCT foc;               // 【有参数设计】foc控制的参数
    MOTOR_ENCODER_STRUCT encoder;       // 【数据】电机角速度和角度信息
    MOTOR_CURRENT_STRUCT current;       // 【数据】电机电流采样信息缓存

    PRINTF_STRUCT TXdata;               // 【数据】data串口或CAN发送的信息
}SguanFOC_System_STRUCT;

// 电机控制核心结构体声明
extern SguanFOC_System_STRUCT Sguan;

void SguanFOC_High_Loop(void);
void SguanFOC_Low_Loop(void);
void SguanFOC_Printf_Loop(uint8_t *data, uint16_t length);
void SguanFOC_main_Loop(void);


#endif // SGUANFOC_H

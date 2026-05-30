#ifndef __SGUANFOC_H
#define __SGUANFOC_H

/* USER CODE BEGIN Includes */
// 电机控制核心函数文件声明
#include "Sguan_Cogging.h"                  // Cogging离线标定抗齿槽算法
#include "Sguan_DOB.h"                      // DOB超螺旋滑模扰动观测器
#include "Sguan_Feedforward.h"              // Feedforward前馈环节
#include "Sguan_Filter.h"                   // Filter低通滤波器和陷波器
#include "Sguan_Hall.h"                     // Hall三霍尔信号处理
#include "Sguan_HFI.h"                      // HFI(无感)高频正弦波注入
#include "Sguan_Identify.h"                 // Identify电机参数辨识
#include "Sguan_Ladrc.h"                    // Ladrc线自抗扰控制
#include "Sguan_LTD.h"                      // LTD最速控制(平滑阶跃信号)
#include "Sguan_MotorStatus.h"              // MotorStatus电机状态机
#include "Sguan_NLFO.h"                     // NLFO(无感)非线性磁链观测
#include "Sguan_Optimize.h"                 // Optimize电机优化算法
#include "Sguan_PID.h"                      // PID传统闭环控制
#include "Sguan_PLL.h"                      // PLL角度跟踪“锁相环”
#include "Sguan_printf.h"                   // printf通信调试
#include "Sguan_SMC.h"                      // SMC传统滑模控制
#include "Sguan_SMO.h"                      // SMO(无感)静止坐标系下的滑模观测器
#include "Sguan_SPWM.h"                     // SPWM零序分量注入法的PWM调制
#include "Sguan_STA.h"                      // STA超螺旋简化滑模控制
#include "Sguan_SVPWM.h"                    // SVPWM七段式空间矢量PWM调制
/* USER CODE END Includes */

// ╔═════════════════════════════════════════════════════════╗
// ║                       开环控制模式                       ║
// ╚═════════════════════════════════════════════════════════╝
#define MODE_VF_OPENLOOP        0x00        // VF压频比开环(Sguan.foc.Target_Speed,Sguan.foc.Uq_in)
#define MODE_IF_OPENLOOP        0x01        // IF流频比开环(Sguan.foc.Target_Speed,Sguan.foc.Target_Iq)
// ╔═════════════════════════════════════════════════════════╗
// ║                   闭环控制模式（有传感器）                ║
// ╚═════════════════════════════════════════════════════════╝
#define MODE_Voltag_OPEN        0x02        // 电压开环(Sguan.foc.Uq_in)
#define MODE_Current_SINGLE     0x03        // 电流单闭环(Sguan.foc.Target_Iq)
#define MODE_VelCur_DOUBLE      0x04        // 速度-电流串级闭环(Sguan.foc.Target_Speed)
#define MODE_PosVelCur_THREE    0x05        // 位置-速度-电流三环(Sguan.foc.Target_Pos)
// ╔═════════════════════════════════════════════════════════╗
// ║                   霍尔控制模式（有传感器）                ║
// ╚═════════════════════════════════════════════════════════╝
#define MODE_Sensor_Hall        0x06        // 有感霍尔_转速环(Sguan.foc.Target_Speed)
// ╔═════════════════════════════════════════════════════════╗
// ║                   无感控制模式（无传感器）                ║
// ╚═════════════════════════════════════════════════════════╝
#define MODE_Sensorless_HFI     0x07        // 高频注入_转速环(Sguan.foc.Target_Speed)
#define MODE_Sensorless_SMO     0x08        // 滑模观测_转速环(Sguan.foc.Target_Speed)
#define MODE_Sensorless_NLFO    0x09        // 非线性磁链_转速环(Sguan.foc.Target_Speed)
#define MODE_Sensorless_HS      0x0A        // 高频滑模结合_转速环(Sguan.foc.Target_Speed)
#define MODE_Sensorless_HN      0x0B        // 高频磁链结合_转速环(Sguan.foc.Target_Speed)
#define MODE_Sensorless_AS      0x0C        // 霍尔滑模结合_转速环(Sguan.foc.Target_Speed)
#define MODE_Sensorless_AN      0x0D        // 霍尔磁链结合_转速环(Sguan.foc.Target_Speed)
// ╔═════════════════════════════════════════════════════════╗
// ║                 DEBUG模式（有感,外载“无感观测器”）        ║
// ╚═════════════════════════════════════════════════════════╝
#define MODE_Debug_HFI          0x0E        // HFI测试_转速环(Sguan.foc.Target_Speed)
#define MODE_Debug_SMO          0x0F        // SMO测试_转速环(Sguan.foc.Target_Speed)
#define MODE_Debug_NLFO         0x10        // NLFO测试_转速环(Sguan.foc.Target_Speed)
#define MODE_Debug_HS           0x11        // HFI切SMO_转速环(Sguan.foc.Target_Speed)
#define MODE_Debug_HN           0x12        // HFI切NLFO_转速环(Sguan.foc.Target_Speed)


// +---------------------------------------------------------+
// |                    控制器Ctrl宏定义                      |
// +---------------------------------------------------------+
#define Control_PID             0x00        // PID闭环控制Sguan_PID
#define Control_LADRC           0x01        // LADRC线自抗扰Sguan_LADRC
#define Control_SMC             0x02        // SMC传统滑模Sguan_SMC
#define Control_STA             0x03        // STA简易超螺旋Sguan_STA
// +---------------------------------------------------------+
// |                    滤波器Filter定义                      |
// +---------------------------------------------------------+
#define LPF_ButterWorth         0x00        // Butter二阶巴特沃斯滤波器
#define LPF_ChebyShev           0x01        // ChebyShev切比雪夫二阶I型
#define LPF_Bessel              0x02        // Bessel二阶贝塞尔滤波器


#define IS_LTD_MODE    (CONFIG_MODE == MODE_VF_OPENLOOP    ||\
                        CONFIG_MODE == MODE_IF_OPENLOOP    ||\
                        CONFIG_MODE == MODE_Sensor_Hall    ||\
                        CONFIG_MODE == MODE_Sensorless_HFI ||\
                        CONFIG_MODE == MODE_Sensorless_SMO ||\
                        CONFIG_MODE == MODE_Sensorless_NLFO||\
                        CONFIG_MODE == MODE_Sensorless_HS  ||\
                        CONFIG_MODE == MODE_Sensorless_HN  ||\
                        CONFIG_MODE == MODE_Sensorless_AS  ||\
                        CONFIG_MODE == MODE_Sensorless_AN  ||\
                        CONFIG_MODE == MODE_Debug_HFI      ||\
                        CONFIG_MODE == MODE_Debug_SMO      ||\
                        CONFIG_MODE == MODE_Debug_NLFO     ||\
                        CONFIG_MODE == MODE_Debug_HS       ||\
                        CONFIG_MODE == MODE_Debug_HN)

#define IS_IF_MODE     (CONFIG_MODE == MODE_Sensorless_SMO ||\
                        CONFIG_MODE == MODE_Sensorless_NLFO)

#define IS_HALL_MODE   (CONFIG_MODE == MODE_Sensor_Hall    ||\
                        CONFIG_MODE == MODE_Sensorless_AS  ||\
                        CONFIG_MODE == MODE_Sensorless_AN)

#define IS_HFI_MODE    (CONFIG_MODE == MODE_Sensorless_HFI ||\
                        CONFIG_MODE == MODE_Sensorless_HS  ||\
                        CONFIG_MODE == MODE_Sensorless_HN  ||\
                        CONFIG_MODE == MODE_Debug_HFI      ||\
                        CONFIG_MODE == MODE_Debug_HS       ||\
                        CONFIG_MODE == MODE_Debug_HN)

#define IS_SMO_MODE    (CONFIG_MODE == MODE_Sensorless_SMO ||\
                        CONFIG_MODE == MODE_Sensorless_HS  ||\
                        CONFIG_MODE == MODE_Sensorless_AS  ||\
                        CONFIG_MODE == MODE_Debug_SMO      ||\
                        CONFIG_MODE == MODE_Debug_HS)

#define IS_NLFO_MODE   (CONFIG_MODE == MODE_Sensorless_NLFO||\
                        CONFIG_MODE == MODE_Sensorless_HN  ||\
                        CONFIG_MODE == MODE_Sensorless_AN  ||\
                        CONFIG_MODE == MODE_Debug_NLFO     ||\
                        CONFIG_MODE == MODE_Debug_HN)

#define IS_COM_MODE    (CONFIG_MODE == MODE_Sensorless_HS  ||\
                        CONFIG_MODE == MODE_Sensorless_HN  ||\
                        CONFIG_MODE == MODE_Sensorless_AS  ||\
                        CONFIG_MODE == MODE_Sensorless_AN  ||\
                        CONFIG_MODE == MODE_Debug_HS       ||\
                        CONFIG_MODE == MODE_Debug_HN)

#define IS_DEBUG_MODE  (CONFIG_MODE == MODE_Debug_HFI      ||\
                        CONFIG_MODE == MODE_Debug_SMO      ||\
                        CONFIG_MODE == MODE_Debug_NLFO     ||\
                        CONFIG_MODE == MODE_Debug_HS       ||\
                        CONFIG_MODE == MODE_Debug_HN)

/**
 * @description: 传递函数指针数组定义
 * @reminder: （Sguan.transfer成员）
 * @struct {MOTOR_TRANSFER_STRUCT}
 */
typedef struct{
    // 1.电流环参数
    PID_STRUCT Current_D;                   // (电流单环)PID电流环D轴参数
    PID_STRUCT Current_Q;                   // (电流单环)PID电流环Q轴参数

    // 2.转速环参数
    #if CONFIG_CtrlVel==Control_LADRC
    LADRC_STRUCT Velocity;                  // (速度-电流双环)LADRC速度外环参数
    #elif CONFIG_CtrlVel==Control_SMC
    SMC_STRUCT Velocity;                    // (速度-电流双环)SMC速度外环参数
    #elif CONFIG_CtrlVel==Control_STA
    STA_STRUCT Velocity;                    // (速度-电流双环)STA速度外环参数
    #else  // CONFIG_CtrlVel
    PID_STRUCT Velocity;                    // (速度-电流双环)双PID速度外环参数
    #endif // CONFIG_CtrlVel

    // 3.位置环参数
    #if CONFIG_CtrlPos==Control_LADRC
    LADRC_STRUCT Position;                  // (高性能伺服三环)Position的LADRC
    #elif CONFIG_CtrlPos==Control_SMC
    SMC_STRUCT Position;                    // (高性能伺服三环)Position的SMC
    #elif CONFIG_CtrlPos==Control_STA
    STA_STRUCT Position;                    // (高性能伺服三环)Position的STA
    #else  // CONFIG_CtrlPos
    PID_STRUCT Position;                    // (高性能伺服三环)Position的PID
    #endif // CONFIG_CtrlPos

    // 4.响应倍数参数
    // （此处暂无数据）

    // 5.滤波器参数
    LPF_STRUCT LPF_D;                       // (电流数据)电机D轴滤波
    LPF_STRUCT LPF_Q;                       // (电流数据)电机Q轴滤波
    LPF_STRUCT LPF_encoder;                 // (速度数据)速度信号滤波

    // 6.锁相环参数
    PLL_STRUCT PLL_encoder;                 // (PLL锁相环)角度跟踪锁相环
    #if IS_COM_MODE
    PLL_STRUCT PLL_another;                 // (PLL锁相环)角度跟踪锁相环
    #endif // IS_COM_MODE

    // 7.Debug模式参数
    #if IS_DEBUG_MODE
    PLL_STRUCT PLL_Debug;                   // (PLL锁相环)角度跟踪锁相环
    LPF_STRUCT LPF_Debug;                   // (速度数据)速度信号滤波
    #endif // IS_DEBUG_MODE

    // 8.扰动观测器参数
    #if CONFIG_DOB
    DOB_STRUCT DOB;                         // (超螺旋滑模扰动观测器)DOB
    #endif // CONFIG_DOB

    // 9.谐波抑制
    #if CONFIG_Inhibit
    TPNF_STRUCT TPNF_D;                     // (谐波抑制)电机D轴滤波
    TPNF_STRUCT TPNF_Q;                     // (谐波抑制)电机Q轴滤波
    #endif // CONFIG_Inhibit

    // 10.弱磁控制参数
    #if CONFIG_FW
    PID_STRUCT FW;                          // (弱磁控制)PI控制器输出弱磁一区控制量
    #endif // CONFIG_FW

    // 11.速度前馈的参数
    // 12.死区补偿参数
    // 13.角度补偿参数
    // （此处暂无数据）
    
    // 14.最速控制结构体
    #if IS_LTD_MODE
    LTD_STRUCT LTD;                         // (最速控制)LTD仿制效果，输入速度不突变
    #endif // IS_LTD_MODE

    // 15.霍尔有感结构体
    #if IS_HALL_MODE
    HALL_STRUCT Hall;                       // (霍尔数据处理)三霍尔信号处理
    #endif // IS_HALL_MODE

    // 16.高频正弦波注入
    #if IS_HFI_MODE
    HFI_STRUCT HFI;                         // (无感算法)HFI高频正弦波注入算法
    #endif // IS_HFI_MODE

    // 17.滑模观测器
    #if IS_SMO_MODE
    SMO_STRUCT SMO;                         // (无感算法)SMO静止坐标系下的滑模观测器
    #endif // IS_SMO_MODE

    // 18.非线性磁链观测器
    #if IS_NLFO_MODE
    NLFO_STRUCT NLFO;                       // (无感算法)NLFO非线性磁链观测器
    #endif // IS_NLFO_MODE

    // 19.无感参数数据
    // （此处暂无数据）
}MOTOR_TRANSFER_STRUCT;

/**
 * @description: 传递函数数值数组定义
 * @reminder: （Sguan.value成员）
 * @struct {MOTOR_VALUE_STRUCT}
 */
typedef struct{
    // 1.电流环参数
    // 2.转速环参数
    // 3.位置环参数
    // （此处暂无数据）

    // 4.响应倍数参数
    uint8_t Response;                       // (参数设计)响应带宽倍数->提高响应裕度

    // 5.滤波器参数
    // 6.锁相环参数
    // 7.Debug模式参数
    // 8.扰动观测器参数
    // 9.谐波抑制
    // （此处暂无数据）

    // 10.弱磁控制参数
    #if CONFIG_FW
    float FW_BaseSpeed;                     // (弱磁控制)基速设计，使得MTPA过渡弱磁
    float FW_Percentage;                    // (弱磁控制)弱磁调制线占比,一般设计0.92
    #endif // CONFIG_FW

    // 11.速度前馈的参数
    #if CONFIG_VelFF
    float VelFF_Beta;                       // (参数设计)转速环角频率->提高转速稳定
    #endif // CONFIG_VelFF

    // 12.死区补偿参数
    #if CONFIG_DeadZone
    float DeadZone_Time;                    // (参数设计)三相死区补偿->降低低速抖震
    float DeadZone_CurMin;                  // (参数设计)死区低电流处理量，低于则不补偿
    #endif // CONFIG_DeadZone

    // 13.角度补偿参数
    #if CONFIG_AngleComp
    float AngleComp_Re;                     // (数据)补偿后的电机电子角度值

    float AngleComp_Td;                     // (参数设计)随电角速度的相位延迟时间
    float AngleComp_Offset;                 // (参数设计)随速度方向变化的固定相位偏置
    #endif // CONFIG_AngleComp

    // 14.最速控制结构体
    // 15.霍尔有感结构体
    // 16.高频正弦波注入
    // 17.滑模观测器
    // 18.非线性磁链观测器
    // （此处暂无数据）

    // 19.无感参数数据
    #if CONFIG_MODE>=MODE_Sensorless_HFI
    float Low_angle;                        // (中间量)低速域电机的机械角度
    float High_angle;                       // (中间量)高速域电机的机械角度
    float Low_Wm;                           // (中间量)低速域电机的机械角速度
    float High_Wm;                          // (中间量)高速域电机的机械角速度

    float Sensorless_Gain;                  // (中间量)观测器切换权重增益
    uint8_t Sensorless_Flag;                // (中间量)高频注入信号开关标志位

    float Sensorless_Stop;                  // (参数设计)无感低速域观测器停止运行界限
    float Sensorless_Open;                  // (参数设计)无感低速域观测器开始运行界限
    float Sensorless_AbsMax;                // (参数设计)角度解耦过渡区_高速域分界线
    float Sensorless_AbsMin;                // (参数设计)角度解耦低速域_过渡区分界线
    #endif // CONFIG_MODE
}MOTOR_VALUE_STRUCT;

/**
 * @description: 编码器数据结构体定义
 * @reminder: （Sguan.encoder成员）
 * @struct {MOTOR_ENCODER_STRUCT}
 */
typedef struct{
    float Real_Speed;                       // (Encoder机械速度)Real实际机械角速度
    float Real_Pos;                         // (Encoder机械角度)Real实际机械角度
    float Real_We;                          // (Encoder电速度)Real实际电子角速度
    float Real_Re;                          // (Encoder电角度)Real实际电子角度

    float Real_offset;                       // (Encoder角度偏置)offset偏置位

    #if IS_DEBUG_MODE
    float Sensorless_Speed;                 // (Sensorless机械速度)预测机械角速度
    float Sensorless_Pos;                   // (Sensorless机械角度)预测机械角度
    float Sensorless_We;                    // (Sensorless电速度)预测电子角速度
    float Sensorless_Re;                    // (Sensorless电角度)预测电子角度

    float Sensorless_offset;                // (Sensorless角度偏置)offset偏置位
    #endif // IS_DEBUG_MODE
}MOTOR_ENCODER_STRUCT;

/**
 * @description: 电流数据结构体定义
 * @reminder: （Sguan.current成员）
 * @struct {MOTOR_CURRENT_STRUCT}
 */
typedef struct{
    float Real_Id;                          // (Current电流)Real实际D轴电流
    float Real_Iq;                          // (Current电流)Real实际Q轴电流

    float Real_Ia;                          // (Current相电流)A相电流
    float Real_Ib;                          // (Current相电流)B相电流
    float Real_Ic;                          // (Current相电流)C相电流

    float Real_Ialpha;                      // (Current中间量电流)alpha轴电流
    float Real_Ibeta;                       // (Current中间量电流)beta轴电流

    float Final_Gain;                       // (ADC增益)最终的ADC电流采样增益
    int32_t Current_offset0;                // (Current电流偏置)offset偏置位
    int32_t Current_offset1;                // (Current电流偏置)offset偏置位
}MOTOR_CURRENT_STRUCT;

/**
 * @description: foc数据结构体定义
 * @reminder: （Sguan.foc成员）
 * @struct {MOTOR_FOC_STRUCT}
 */
typedef struct{
    float Target_Speed;                     // (期望速度)Target期望机械角速度
    float Target_Pos;                       // (期望角度)Target期望机械角度
    float Target_Id;                        // (期望电流)期望D轴电流
    float Target_Iq;                        // (期望电流)期望Q轴电流

    float Target_VF_Uq;                     // (期望电压)固定Q轴电压值
    float Target_IF_Iq;                     // (期望电流)固定Q轴电流值

    float Speed_in;                         // (输入量)速度环输入值
    float Ud_in;                            // (输入量)D轴电压输入
    float Uq_in;                            // (输入值)Q轴电压输入

    // ================= 修改线(上面可修改，下面为自动计算量) =================
    float Ualpha;                           // (中间量)alpha轴电压
    float Ubeta;                            // (中间量)beta轴电压

    float Du;                               // (数据)U相占空比输入0~1
    float Dv;                               // (数据)V相占空比输入0~1
    float Dw;                               // (数据)W相占空比输入0~1

    float sine;                             // (数据)sine临时保存的正弦值
    float cosine;                           // (数据)cosine临时保存的余弦值

    float Real_VBUS;                        // (数据)Real实际的电机母线电压
    float Real_Temp;                        // (数据)Temp实际的驱动器物理温度
}MOTOR_FOC_STRUCT;

/**
 * @description: 电机参数结构体定义
 * @reminder: （Sguan.motor成员）
 * @struct {MOTOR_QUANTIZE_STRUCT}
 */
typedef struct{
    IDENTIFY_STRUCT identify;               // (参数辨识)电机的核心实体参数

    uint8_t Poles;                          // (电机实体参数)电机极对数
    float VBUS;                             // (电机实体参数)母线电压

    int8_t Motor_Dir;                       // (参数设计)电机的运行方向设计
    int8_t Encoder_Dir;                     // (有感实体参数)编码器方向
    int8_t PWM_Dir;                         // (参数设计)PWM占空比高低对应
    uint32_t Duty;                          // (参数设计)PWM满占空比

    int8_t Current_Dir0;                    // (参数设计)电流采样方向0
    int8_t Current_Dir1;                    // (参数设计)电流采样方向1
    uint8_t Current_Num;                    // (参数设计)电流通道0->AB相，1->AC相，2->BC相
    uint32_t ADC_Precision;                 // (参数设计)ADC采样精度,如12位精度为4096
    float Amplifier;                        // (参数设计)运放的放大倍数
    float MCU_Voltage;                      // (参数设计)DSP/单片机的ADC基准电压
    float Sampling_Rs;                      // (参数设计)采样电阻的阻值大小
}MOTOR_QUANTIZE_STRUCT;

/**
 * @description: 安全设置结构体定义
 * @reminder: （Sguan.safe成员）
 * @struct {MOTOR_SAFE_STRUCT}
 */
typedef struct{    
    float VBUS_MAX;                         // (参数设计)母线电压值波动MAX阈值
    float VBUS_MIM;                         // (参数设计)母线电压值波动MIN阈值
    uint32_t VBUS_watchdog_limit;           // (参数设计)电压异常的警告周期
    // 如果有电机电压预警，电机正常运行...经过Sguan_Low_Loop()的VBUS_watchdog_limit此运行周期
    // 若2-10次，中间有一次再触发电压警告，电机停转进待机
    // 若首次后10次，都未再触发，电机以后都正常运行
    
    float Temp_MAX;                         // (参数设计)驱动器允许最大温度
    float Temp_MIN;                         // (参数设计)驱动器允许最小温度
    uint32_t Temp_watchdog_limit;           // (参数设计)温度异常的警告周期
    // 如果有驱动器温度预警，电机正常运行...经过Sguan_Low_Loop()的Temp_watchdog_limit此运行周期
    // 若2-10次，中间有一次再触发温度警告，电机停转进待机
    // 若首次后10次，都未再触发，电机以后都正常运行
 
    float Dcur_MAX;                         // (参数设计)电机最大电流D轴限制
    float Qcur_MAX;                         // (参数设计)电机最大电流Q轴限制
    uint32_t DQcur_watchdog_limit;          // (参数设计)过流保护的警告周期
    // 如果有电机过流预警，电机正常运行...经过Sguan_Low_Loop()的DQcur_watchdog_limit此运行周期
    // 若2-10次，中间有一次再触发过流警告，电机停转进待机
    // 若首次后10次，都未再触发，电机以后都正常运行

    float Current_limit;                    // (参数设计)电流正负区间设计，电流状态机判断
    float Speed_limit;                      // (参数设计)速度正负区间设计，速度状态机判断
    float Position_limit;                   // (参数设计)位置正负区间设计，位置状态机判断

    uint32_t DISABLED_watchdog_limit;       //(参数设计)电机DISABLED状态机进待机模式的延时周期
}MOTOR_SAFE_STRUCT;

/**
 * @description: 标志位结构体定义
 * @reminder: （Sguan.flag成员）
 * @struct {MOTOR_FLAG_STRUCT}
 */
typedef struct{
    uint8_t PWM_Calc;                       // PWM计算“标志位”
    uint8_t PWM_watchdog_limit;             // PWM错误限幅(PWM计算不能中断错误太多次)
    uint8_t in_PWM_Calc_ISR;                // (互斥锁)标记是否在PWM计算中断中
}MOTOR_FLAG_STRUCT;

/**
 * @description: SguanFOC系统核心结构体定义
 * @reminder: （Sguan总结构体成员）
 * @struct {SguanFOC_System_STRUCT}
 */
typedef struct{
    // ============================= ①电机标识位 =================================
    uint8_t status;                         // 【数据】status存储电机运行状态
    
    // ============================= ②嵌套结构体 =================================
    MOTOR_TRANSFER_STRUCT transfer;         // 【有参数设计】transfer传递函数
    MOTOR_VALUE_STRUCT value;               // 【有参数设计】value传递函数数值

    MOTOR_ENCODER_STRUCT encoder;           // 【数据】encoder电机角速度和角度信息
    MOTOR_CURRENT_STRUCT current;           // 【数据】current电机电流采样信息缓存
    MOTOR_FOC_STRUCT foc;                   // 【有参数设计】foc控制的参数
    MOTOR_QUANTIZE_STRUCT motor;            // 【有参数设计】motor电机参数量化
    MOTOR_SAFE_STRUCT safe;                 // 【有参数设计】safe电机安全设置
    MOTOR_FLAG_STRUCT flag;                 // 【有参数设计】flag电机运行标志位
    PRINTF_STRUCT txdata;                   // 【数据】data串口或CAN发送的信息

    // =========================== ③简易控制函数 =================================
    void (*Func_Start)(void);               // 【函数】control控制接口->启动电机
    void (*Func_Stop)(void);                // 【函数】control控制接口->停止电机
    void (*Func_Set_Ud)(float);             // 【函数】control控制接口->设计目标电压
    void (*Func_Set_Uq)(float);             // 【函数】control控制接口->设计目标电压
    void (*Func_Set_Id)(float);             // 【函数】control控制接口->设计目标电流
    void (*Func_Set_Iq)(float);             // 【函数】control控制接口->设计目标电流
    void (*Func_Set_Velocity)(float);       // 【函数】control控制接口->设计目标转速
    void (*Func_Set_Position)(float);       // 【函数】control控制接口->设计目标位置
    void (*Func_Set_TXdata)(uint8_t,float); // 【函数】control控制接口->数据打印
}SguanFOC_System_STRUCT;

// 电机控制核心结构体声明
extern SguanFOC_System_STRUCT Sguan;

void SguanFOC_High_Loop(void);
void SguanFOC_Low_Loop(void);
void SguanFOC_Printf_Loop(uint8_t *data, uint16_t length);
void SguanFOC_main_Loop(void);


#endif // SGUANFOC_H

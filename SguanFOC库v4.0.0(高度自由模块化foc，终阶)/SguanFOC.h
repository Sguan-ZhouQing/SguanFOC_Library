#ifndef __SGUANFOC_H
#define __SGUANFOC_H

#include "Sguan_Function.h"
#include "Sguan_MotorStatus.h"
#include "Sguan_Printf.h"
#include "Sguan_Transfer.h"



typedef struct{
    void (*Func_Start)(void);
}__MOTOR_FUNCTION_STRUCT;

typedef struct{
    #if CONFIG_TRANSFER1 // 典型一阶传递函数
    Transfer1 *Transfer1_CH[CONFIG_TRANSFER1];
    #endif // CONFIG_TRANSFER1

    #if CONFIG_TRANSFER2 // 典型二阶传递函数
    Transfer2 *Transfer2_CH[CONFIG_TRANSFER2];
    #endif // CONFIG_TRANSFER2

    #if CONFIG_TRANSFER3 // 典型三阶传递函数
    Transfer3 *Transfer3_CH[CONFIG_TRANSFER3];
    #endif // CONFIG_TRANSFER3

    #if CONFIG_TRANSFER4 // 典型四阶传递函数
    Transfer4 *Transfer4_CH[CONFIG_TRANSFER4];
    #endif // CONFIG_TRANSFER4

    #if CONFIG_TRANSFER5 // 典型五阶传递函数
    Transfer5 *Transfer5_CH[CONFIG_TRANSFER5];
    #endif // CONFIG_TRANSFER5

    #if CONFIG_INTEGRATOR // 积分器
    Integrator *Integrator_CH[CONFIG_INTEGRATOR];
    #endif // CONFIG_INTEGRATOR

    #if CONFIG_DERIVATIVE // 微分器
    Derivative *Derivative_CH[CONFIG_DERIVATIVE];
    #endif // CONFIG_DERIVATIVE

    #if CONFIG_DFT // 快速傅里叶变换
    Dft *Dft_CH[CONFIG_DFT];
    #endif // CONFIG_DFT

    #if CONFIG_HALL // 霍尔编码器
    Hall *Hall_CH[CONFIG_HALL];
    #endif // CONFIG_HALL

    #if CONFIG_LADRC1 // 一阶线性自适应抗干扰控制
    Ladrc1 *Ladrc1_CH[CONFIG_LADRC1];
    #endif // CONFIG_LADRC1

    #if CONFIG_LADRC2 // 二阶线性自适应抗干扰控制
    Ladrc2 *Ladrc2_CH[CONFIG_LADRC2];
    #endif // CONFIG_LADRC2

    #if CONFIG_SMC // 传统指数型趋近率的滑模控制
    Smc *Smc_CH[CONFIG_SMC];
    #endif // CONFIG_SMC

    #if CONFIG_DPCC // 增量式电流预测控制
    Dpcc *Dpcc_CH[CONFIG_DPCC];
    #endif // CONFIG_DPCC

    #if CONFIG_PIR // 比例积分谐振调节器
    Pir *Pir_CH[CONFIG_PIR];
    #endif // CONFIG_PIR

    #if CONFIG_PID // 传统闭环控制器
    Pid *Pid_CH[CONFIG_PID];
    #endif // CONFIG_PID

    #if CONFIG_PLL // 开环锁相环
    Pll *Pll_CH[CONFIG_PLL];
    #endif // CONFIG_PLL

    #if CONFIG_LPF1 // 一阶低通滤波器
    Lpf1 *Lpf1_CH[CONFIG_LPF1];
    #endif // CONFIG_LPF1

    #if CONFIG_LPF2 // 二阶低通滤波器
    Lpf2 *Lpf2_CH[CONFIG_LPF2];
    #endif // CONFIG_LPF2

    #if CONFIG_HPF1 // 一阶高通滤波器
    Hpf1 *Hpf1_CH[CONFIG_HPF1];
    #endif // CONFIG_HPF1

    #if CONFIG_HPF2 // 二阶高通滤波器
    Hpf2 *Hpf2_CH[CONFIG_HPF2];
    #endif // CONFIG_HPF2

    #if CONFIG_BPF1 // 带通滤波器(一阶低通和高通串联)
    Bpf1 *Bpf1_CH[CONFIG_BPF1];
    #endif // CONFIG_BPF1

    #if CONFIG_BPF2 // 带通滤波器(典型二阶系统改型)
    Bpf2 *Bpf2_CH[CONFIG_BPF2];
    #endif // CONFIG_BPF2

    #if CONFIG_NF // 陷波滤波器(典型二阶系统改型)
    Nf *Nf_CH[CONFIG_NF];
    #endif // CONFIG_NF

    #if CONFIG_TPNF // 陷波滤波器(三参数陷波滤波器)
    Tpnf *Tpnf_CH[CONFIG_TPNF];
    #endif // CONFIG_TPNF

    #if CONFIG_DOB // 超螺旋滑模扰动观测器
    Dob *Dob_CH[CONFIG_DOB];
    #endif // CONFIG_DOB

    #if CONFIG_RLS // 电机参数在线辨识观测器
    Rls *Rls_CH[CONFIG_RLS];
    #endif // CONFIG_RLS

    #if CONFIG_SMO // (无感)滑模观测器
    Smo *Smo_CH[CONFIG_SMO];
    #endif // CONFIG_SMO

    #if CONFIG_NLFO // (无感)非线性磁链观测器
    Nlfo *Nlfo_CH[CONFIG_NLFO];
    #endif // CONFIG_NLFO

    #if CONFIG_HFI // (无感)高频正弦波注入
    Hfi *Hfi_CH[CONFIG_HFI];
    #endif // CONFIG_HFI

    #if CONFIG_ROLO // (无感)降阶龙伯格观测器
    Rolo *Rolo_CH[CONFIG_ROLO];
    #endif // CONFIG_ROLO

    #if CONFIG_MARS // (无感)模型参考自适应观测器
    Mars *Mars_CH[CONFIG_MARS];
    #endif // CONFIG_MARS

    #if CONFIG_EKF // (无感)扩展卡尔曼滤波
    Ekf *Ekf_CH[CONFIG_EKF];
    #endif // CONFIG_EKF

    #if CONFIG_DELAY1 // 延时函数(延时一拍)
    Delay1 *Delay1_CH[CONFIG_DELAY1];
    #endif // CONFIG_DELAY1

    #if CONFIG_DELAY2 // 延时函数(延时两拍)
    Delay2 *Delay2_CH[CONFIG_DELAY2];
    #endif // CONFIG_DELAY2

    #if CONFIG_DELAY3 // 延时函数(延时三拍)
    Delay3 *Delay3_CH[CONFIG_DELAY3];
    #endif // CONFIG_DELAY3

    Sine *Sine;                     // 正弦发生器
    Cosine *Cosine;                 // 余弦发生器
    SinCos *SinCos;                 // 正余弦发生器
    Tan *Tan;                       // 正切求解器
    Atan *Atan;                     // 反正切求解器
    Limit *Limit;                   // 限幅函数
    Sign *Sign;                     // 符号函数
    Clarke *clarke;                 // 克拉克变换
    Park *park;                     // 帕克变换
    Ipark *ipark;                   // 帕克逆变换
    Spwm0 *SPWM0;                   // 零序注入的SPWM模块
    Spwm *SPWM;                     // 普通SPWM模块
    Svpwm *SVPWM;                   // 七段式SVPWM模块
    Swpwm *Swpwm;                   // 电调PWM无感方波
    SingleRs *SingleRs;             // 单电阻采样函数
}__MOTOR_TRANSFER_STRUCT;

typedef struct{
    #if CONFIG_Float // 浮点数
    SguanQ             Float_CH[CONFIG_Float];
    #endif // CONFIG_Float

    #if CONFIG_Int8 // 8位数据
    int8_t              Int8_CH[CONFIG_Int8];
    #endif // CONFIG_Int8

    #if CONFIG_Uint8 // 8位数据
    uint8_t             Uint8_CH[CONFIG_Uint8];
    #endif // CONFIG_Uint8

    #if CONFIG_Int16 // 16位数据
    int16_t             Int16_CH[CONFIG_Int16];
    #endif // CONFIG_Int16

    #if CONFIG_Uint16 // 16位数据
    uint16_t            Uint16_CH[CONFIG_Uint16];
    #endif // CONFIG_Uint16

    #if CONFIG_Int32 // 32位数据
    int32_t             Int32_CH[CONFIG_Int32];
    #endif // CONFIG_Int32

    #if CONFIG_Uint32 // 32位数据
    uint32_t            Uint32_CH[CONFIG_Uint32];
    #endif // CONFIG_Uint32

    uint8_t Response;                           // (环路倍率)内外环控制倍率

    SguanQ Num_Delta;                          // (固化参数)速度增减判断
    SguanQ Num_Abs;                            // (固化参数)速度绝对值

    SguanQ Num_L0;                             // (固化参数)速度段0标定
    SguanQ Num_LA;                             // (固化参数)速度段A标定
    SguanQ Num_LB;                             // (固化参数)速度段B标定
    SguanQ Num_LC;                             // (固化参数)速度段C标定
    SguanQ Num_LD;                             // (固化参数)速度段D标定

    SguanQ Num_High_Data;                      // (数据)高位观测器数值
    SguanQ Num_Low_Data;                       // (数据)低位观测器数值
}__MOTOR_VALUE_STRUCT;

typedef struct{
    SguanQ Target_Speed;                       // (期望速度)Target期望机械角速度
    SguanQ Target_Pos;                         // (期望角度)Target期望机械角度
    SguanQ Target_Id;                          // (期望电流)期望D轴电流
    SguanQ Target_Iq;                          // (期望电流)期望Q轴电流
    SguanQ Target_Ud;                          // (期望电压)期望D轴电压
    SguanQ Target_Uq;                          // (期望电压)期望Q轴电压

    SguanQ Speed_in;                           // (输入量end)速度环输入值
    SguanQ Ud_in;                              // (输入量end)D轴电压输入
    SguanQ Uq_in;                              // (输入值end)Q轴电压输入

    // ================= 修改线(上面可修改，下面为自动计算量) =================
    SguanQ Ualpha;                             // (中间量)alpha轴电压
    SguanQ Ubeta;                              // (中间量)beta轴电压

    SguanQ Du;                                 // (数据)U相占空比输入0~1
    SguanQ Dv;                                 // (数据)V相占空比输入0~1
    SguanQ Dw;                                 // (数据)W相占空比输入0~1

    SguanQ sine;                               // (数据)sine临时保存的正弦值
    SguanQ cosine;                             // (数据)cosine临时保存的余弦值

    SguanQ Real_VBUS;                          // (数据)Real实际的电机母线电压
    SguanQ Real_Temp;                          // (数据)Temp实际的驱动器物理温度
}__MOTOR_FOC_STRUCT;

typedef struct{
    SguanQ Real_Id;                            // (Current电流)Real实际D轴电流
    SguanQ Real_Iq;                            // (Current电流)Real实际Q轴电流

    SguanQ Real_Ia;                            // (Current相电流)A相电流
    SguanQ Real_Ib;                            // (Current相电流)B相电流
    SguanQ Real_Ic;                            // (Current相电流)C相电流

    SguanQ Real_Ialpha;                        // (Current中间量电流)alpha轴电流
    SguanQ Real_Ibeta;                         // (Current中间量电流)beta轴电流

    SguanQ Final_Gain;                         // (ADC增益)最终的ADC电流采样增益
    int32_t Current_offset0;                    // (Current电流偏置)offset偏置位
    int32_t Current_offset1;                    // (Current电流偏置)offset偏置位
}__MOTOR_CURRENT_STRUCT;

typedef struct{
    SguanQ Rs;                                 // (电机实体参数)Rs相电阻参数
    SguanQ Ld;                                 // (电机实体参数)Ld电感参数
    SguanQ Lq;                                 // (电机实体参数)Lq电感参数
    SguanQ Flux;                               // (电机实体参数)Flux磁链参数

    uint8_t Poles;                              // (电机实体参数)电机极对数
    SguanQ VBUS;                               // (电机实体参数)母线电压

    int8_t Motor_Dir;                           // (参数设计)电机的运行方向设计
    int8_t Encoder_Dir;                         // (有感实体参数)编码器方向
    int8_t PWM_Dir;                             // (参数设计)PWM占空比高低对应
    uint32_t Duty;                              // (参数设计)PWM满占空比

    int8_t Current_Dir0;                        // (参数设计)电流采样方向0
    int8_t Current_Dir1;                        // (参数设计)电流采样方向1
    uint8_t Current_Num;                        // (参数设计)电流通道0->AB相，1->AC相，2->BC相
    uint32_t ADC_Precision;                     // (参数设计)ADC采样精度,如12位精度为4096
    SguanQ Amplifier;                          // (参数设计)运放的放大倍数
    SguanQ MCU_Voltage;                        // (参数设计)DSP/单片机的ADC基准电压
    SguanQ Sampling_Rs;                        // (参数设计)采样电阻的阻值大小
}__MOTOR_MASTER_STRUCT;

typedef struct{
    uint8_t Run;

    uint8_t Mode;                   
    uint8_t Status;
    uint8_t Flag;

    __MOTOR_TRANSFER_STRUCT Transfer;
    __MOTOR_VALUE_STRUCT Value;
    __MOTOR_FOC_STRUCT Foc;
}SguanFOC_STRUCT;


#endif // SGUANFOC_H

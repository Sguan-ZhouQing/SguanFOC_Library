#ifndef __USERDATA_MOTOR_H
#define __USERDATA_MOTOR_H
#include "SguanFOC.h"
/* 电机控制User用户设置·电机参数(SguanFOC用户核心代码) */

// 电机实体参数设置(根据实际需要填写)
static inline void User_MotorSet(void){
    // 0.mode选择电机的运行模式
    Sguan.mode = PosVelCur_THREE_MODE;
    // 1.flag电机标志位
    Sguan.flag.PWM_watchdog_limit = 10; // (uint8_t)PWM错误限幅
    // 2.identify电机参数辨识结果(根据实际电机参数填写，或者通过辨识算法得到)
    Sguan.identify.Ld = 0.0000519338f;  // (float)D轴电感
    Sguan.identify.Lq = 0.0000519338f;  // (float)Q轴电感
    Sguan.identify.Ls = 0.0000519338f;  // (float)相线电感
    Sguan.identify.Rs = 0.19067f;       // (float)相线电阻
    Sguan.identify.Flux = 0.00028043f;  // (float)磁链
    // 3.motor电机参数辨识
    Sguan.motor.VBUS = 12.0f;           // (float)母线电压
    Sguan.motor.Poles = 7;              // (uint8_t)极对极数

    Sguan.motor.Limit = 0.2f;           // (float)预处理电压占比
    Sguan.motor.Dcur_MAX = 10.0f;       // (float)电机最大电流D轴限制
    Sguan.motor.Qcur_MAX = 10.0f;       // (float)电机最大电流Q轴限制

    #if Open_VBUS_Calculate
    Sguan.motor.VBUS_MAX = 14.0f;       // (float)母线电压值波动MAX阈值
    Sguan.motor.VBUS_MIM = 10.0f;       // (float)母线电压值波动MIN阈值
    #endif // Open_VBUS_Calculate

    #if Open_Temp_Calculate
    Sguan.motor.Temp_MAX = 60.0f;       // (float)驱动器允许最大温度
    Sguan.motor.Temp_MIN = -20.0f;      // (float)驱动器允许最小温度
    #endif // Open_Temp_Calculate

    Sguan.motor.Motor_Dir = 1;          // (int8_t)电机方向1->正向，负1->负向
    Sguan.motor.PWM_Dir = -1;           // (int8_t)PWM占空比高低对应1->正向，负1->负向
    Sguan.motor.Duty = 4249;           // (uint16_t)PWM满占空比数值

    Sguan.motor.Encoder_Dir = -1;       // (int8_t)编码器方向1->正向，负1->负向

    Sguan.motor.Current_Dir0 = 1;       // (int8_t)相线电流方向1->正向，负1->负向
    Sguan.motor.Current_Dir1 = 1;       // (int8_t)相线电流方向1->正向，负1->负向
    Sguan.motor.Current_Num = 1;        // (uint8_t)电流通道0->AB相，1->AC相，2->BC相
    Sguan.motor.ADC_Precision = 4096;   // (uint32_t)ADC采样精度
    Sguan.motor.Amplifier = 10.0f;      // (float)运算放大器增益
    Sguan.motor.MCU_Voltage = 3.3f;     // (float)DSP/单片机的ADC电压基准
    Sguan.motor.Sampling_Rs = 0.005f;   // (float)采样电阻大小
    // 4.系统定时中断周期设计
    Sguan.System_T = 0.00005f;           // (float)系统电机运行时间周期
    Sguan.TIM_ms_T = 0.001f;            // (float)系统ms级中断时间
}   


#endif // USERDATA_MOTOR_H

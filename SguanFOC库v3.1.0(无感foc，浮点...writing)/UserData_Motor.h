#ifndef __USERDATA_MOTOR_H
#define __USERDATA_MOTOR_H
#include "SguanFOC.h"
/* 电机控制User用户设置·电机参数(SguanFOC) */

/**
 * @description: 实体参数填写函数的初始化代码
 * @reminder: (此方函数->填写电机实际物理参数)
 * @param {SguanFOC_System_STRUCT} *user
 * @return {*}
 */
static inline void User_Motor_Init(SguanFOC_System_STRUCT *user){
    // +---------------------------------------------------------+
    // |                  电机实体参数Motor填写                   |
    // +---------------------------------------------------------+
    // 1.Sguan.foc参数设计(最初始的Target数值)
    user->foc.Target_Speed = 230.0f;                  // (float)期望的机械角速度
    user->foc.Target_Pos = 0.0f;                    // (float)期望的机械角度值
    user->foc.Target_Id = 0.0f;                     // (float)期望的D轴电流
    user->foc.Target_Iq = 0.0f;                     // (float)期望的Q轴电流

    user->foc.Target_VF_Uq = 0.0f;                  // (float)设置的Uq大小
    user->foc.Target_IF_Iq = 0.0f;                  // (float)设置的Iq大小

    user->foc.Ud_in = 0.0f;                         // (float)实际输入的D轴电压
    user->foc.Uq_in = 0.0f;                         // (float)实际输入的Q轴电压

    // 2.Sguan.motor参数设计(电机个性化配置)
    user->motor.identify.Rs = 0.19067f;             // (float)相线电阻_默认参数
    user->motor.identify.Ld = 0.0000519338f;        // (float)D轴电感_默认参数
    user->motor.identify.Lq = 0.0000519338f;        // (float)Q轴电感_默认参数
    
    user->motor.identify.Flux = 0.00028043f;        // (float)电机磁链_默认参数
    user->motor.identify.B = 0.00028043f;           // (float)粘性阻尼_默认参数
    user->motor.identify.J = 0.00028043f;           // (float)转动惯量_默认参数
    /* ====================================== 分割线 =================================== */
    user->motor.Poles = 7;                          // (uint8_t)电机的极对数
    user->motor.VBUS = 12.0f;                       // (float)标定的母线电压
    
    user->motor.Motor_Dir = 1;                      // (int8_t)电机方向1->正向，负1->负向
    user->motor.Encoder_Dir = -1;                   // (int8_t)Q轴电感_默认参数
    user->motor.PWM_Dir = -1;                       // (int8_t)PWM占空比电平，1为正向,-1为负向
    user->motor.Duty = 4249;                        // (uint16_t)PWM满占空比数值

    user->motor.Current_Dir0 = 1;                   // (int8_t)相线电流方向1->正向，负1->负向
    user->motor.Current_Dir1 = 1;                   // (int8_t)相线电流方向1->正向，负1->负向
    user->motor.Current_Num = 1;                    // (uint8_t)通道0->AB相，1->AC相，2->BC相
    user->motor.ADC_Precision = 4096;               // (uint32_t)ADC采样精度
    user->motor.Amplifier = 10.0f;                  // (float)运算放大器增益
    user->motor.MCU_Voltage = 3.3f;                 // (float)DSP/单片机的ADC电压基准
    user->motor.Sampling_Rs = 0.005f;               // (float)采样电阻的大小

    // 3.Sguan.safe参数设计(维护驱动器安全)
    user->safe.VBUS_MAX = 14.0f;                    // (float)母线电压值波动MAX阈值
    user->safe.VBUS_MIM = 10.0f;                    // (float)母线电压值波动MIN阈值
    user->safe.VBUS_watchdog_limit = 1000;          // (uint32_t)看门狗

    user->safe.Temp_MAX = 60.0f;                    // (float)驱动器允许最大温度
    user->safe.Temp_MIN = -20.0f;                   // (float)驱动器允许最小温度
    user->safe.Temp_watchdog_limit = 1000;          // (uint32_t)看门狗

    user->safe.Dcur_MAX = 60.0f;                    // (float)电机最大电流D轴限制
    user->safe.Qcur_MAX = 60.0f;                    // (float)电机最大电流Q轴限制
    user->safe.DQcur_watchdog_limit = 1000;         // (uint32_t)看门狗

    user->safe.Current_limit = 0.5f;                // (float)电机->电流状态机判断的电流范围
    user->safe.Speed_limit = 5.0f;                  // (float)电机->速度状态机判断的速度范围
    user->safe.Position_limit = 1.0f;               // (float)电机->位置状态机判断的位置范围

    user->safe.DISABLED_watchdog_limit = 1000;      // (uint32_t)看门狗

    // 4.Sguan.flag参数设计(非重要数据)
    user->flag.PWM_watchdog_limit = 10;             // (uint8_t)PWM错误次数限幅
}   


#endif // USERDATA_MOTOR_H

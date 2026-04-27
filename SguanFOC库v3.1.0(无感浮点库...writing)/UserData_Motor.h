#ifndef __USERDATA_MOTOR_H
#define __USERDATA_MOTOR_H
#include "SguanFOC.h"
/* 电机控制User用户设置·电机参数(SguanFOC用户核心代码) */

// 电机实体参数设置(根据实际需要填写)
static inline void User_MotorSet(void){
    // 1.mode选择电机的运行模式
    // Sguan.mode = Current_SINGLE_MODE;
    // 如果你要在电机启动后主动切换模式，这个地方请不要使用
    // 它会在每次启动时，刷新你的更改值
    // 2.flag电机标志位
    Sguan.flag.PWM_watchdog_limit = 10;             // (uint8_t)PWM错误次数限幅
    // 3.motor电机参数辨识
    Sguan.motor.identify.Rs = 0.19067f;             // (float)相线电阻_默认参数
    Sguan.motor.identify.Ld = 0.0000519338f;        // (float)D轴电感_默认参数
    Sguan.motor.identify.Lq = 0.0000519338f;        // (float)Q轴电感_默认参数
    Sguan.motor.identify.Encoder_Dir = -1;          // (int8_t)Q轴电感_默认参数

    Sguan.motor.identify.Flux = 0.00028043f;        // (float)电机磁链_默认参数
    Sguan.motor.identify.B = 0.00028043f;           // (float)粘性阻尼_默认参数
    Sguan.motor.identify.J = 0.00028043f;           // (float)转动惯量_默认参数
    /* =================================== 分割线 ================================= */
    Sguan.motor.Poles = 7;                          // (uint8_t)极对数
    Sguan.motor.VBUS = 12.0f;                       // (float)母线电压

    Sguan.motor.Motor_Dir = 1;                      // (int8_t)电机方向1->正向，负1->负向
    Sguan.motor.PWM_Dir = -1;                       // (int8_t)PWM占空比高低对应1->正向，负1->负向
    Sguan.motor.Duty = 4249;                        // (uint16_t)PWM满占空比数值

    Sguan.motor.Current_Dir0 = 1;                   // (int8_t)相线电流方向1->正向，负1->负向
    Sguan.motor.Current_Dir1 = 1;                   // (int8_t)相线电流方向1->正向，负1->负向
    Sguan.motor.Current_Num = 1;                    // (uint8_t)通道0->AB相，1->AC相，2->BC相
    Sguan.motor.ADC_Precision = 4096;               // (uint32_t)ADC采样精度
    Sguan.motor.Amplifier = 10.0f;                  // (float)运算放大器增益
    Sguan.motor.MCU_Voltage = 3.3f;                 // (float)DSP/单片机的ADC电压基准
    Sguan.motor.Sampling_Rs = 0.005f;               // (float)采样电阻大小
    // 4.电机安全设计
    Sguan.safe.VBUS_MAX = 14.0f;                    // (float)母线电压值波动MAX阈值
    Sguan.safe.VBUS_MIM = 10.0f;                    // (float)母线电压值波动MIN阈值
    Sguan.safe.VBUS_watchdog_limit = 1000;

    Sguan.safe.Temp_MAX = 60.0f;                    // (float)驱动器允许最大温度
    Sguan.safe.Temp_MIN = -20.0f;                   // (float)驱动器允许最小温度
    Sguan.safe.Temp_watchdog_limit = 1000;

    Sguan.safe.Dcur_MAX = 60.0f;                    // (float)电机最大电流D轴限制
    Sguan.safe.Qcur_MAX = 60.0f;                    // (float)电机最大电流Q轴限制
    Sguan.safe.DQcur_watchdog_limit = 1000;

    Sguan.safe.Current_limit = 0.5f;                // (float)电机->电流状态机判断的电流范围
    Sguan.safe.Speed_limit = 5.0f;                  // (float)电机->速度状态机判断的速度范围
    Sguan.safe.Position_limit = 1.0f;               // (float)电机->位置状态机判断的位置范围

    Sguan.safe.DISABLED_watchdog_limit = 1000;
}   


#endif // USERDATA_MOTOR_H

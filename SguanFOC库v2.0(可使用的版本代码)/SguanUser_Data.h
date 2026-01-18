/*
 * @Author: 星必尘Sguan
 * @Date: 2025-11-14 09:31:21
 * @LastEditors: 星必尘Sguan|3464647102@qq.com
 * @LastEditTime: 2025-11-16 15:00:48
 * @FilePath: \SguanFOC\SguanUser_Data.h
 * @Description: SguanFOC的“用户接口”库
 * 
 * Copyright (c) 2025 by $JUST, All Rights Reserved. 
 */
#ifndef __SGUANUSER_DATA_H
#define __SGUANUSER_DATA_H

/**
 * @description: 宏定义启动电机ABCD的使用(重要)
 * @key_reminder: 请依据需求开启“电机定义”(重要)
 */
#define SguanMotor0
// #define SguanMotor1
// #define SguanMotor2
// #define SguanMotor3

/* USER CODE BEGIN Includes */
#include "SguanFOC.h"

/* USER CODE END Includes */

#ifdef SguanMotor0
extern Motor_System_STRUCT Sguan0;
#endif // SguanMotor0
#ifdef SguanMotor1
extern Motor_System_STRUCT Sguan1;
#endif // SguanMotor1
#ifdef SguanMotor2
extern Motor_System_STRUCT Sguan2;
#endif // SguanMotor2
#ifdef SguanMotor3
extern Motor_System_STRUCT Sguan3;
#endif // SguanMotor3

void Sguan_ParameterSet(void) {
    #ifdef SguanMotor0
    /* 1.控制模式 */
    Sguan0.Control_mode = VelCur_DOUBLE_MODE;
    /* 2.电机结构体变量 */
    Sguan0.Motor.Dir_n = 0;
    Sguan0.Motor.Polepairs = 7;
    /* 3.FOC结构体变量 */
    Sguan0.Foc.Dir_m = 0;
    Sguan0.Foc.Period = 4249;
    Sguan0.Foc.Target_Id = 0.0f;
    Sguan0.Foc.Target_Iq = 0.0f;
    Sguan0.Foc.Target_Speed = 0.0f;
    Sguan0.Foc.Target_Pos = 0.0f;
    Sguan0.Foc.Response_Num = 5;
    /* 4.位置环PID */
    Sguan0.Position.Kp = 0.15f;
    Sguan0.Position.Ki = 0.00000025f;
    Sguan0.Position.Kd = 0.0f;
    Sguan0.Position.OutMax = 0.35f;
    Sguan0.Position.OutMin = -0.35f;
    Sguan0.Position.D_Filter = 0.2f;
    Sguan0.Position.ErrLimltFlag = 0;
    Sguan0.Position.ErrMax = 0.0f;
    Sguan0.Position.ErrMin = 0.0f;
    /* 5.速度环PID */
    Sguan0.Speed.Kp = 0.02226f;
    Sguan0.Speed.Ki = 0.000115f;
    Sguan0.Speed.Kd = 0.0f;
    Sguan0.Speed.OutMax = 0.9f;
    Sguan0.Speed.OutMin = -0.9f;
    Sguan0.Speed.D_Filter = 0.2f;
    Sguan0.Speed.ErrLimltFlag = 0;
    Sguan0.Speed.ErrMax = 0.0f;
    Sguan0.Speed.ErrMin = 0.0f;
    /* 6.电流环D轴PID */
    Sguan0.Id.Kp = 0.02226f;
    Sguan0.Id.Ki = 0.000115f;
    Sguan0.Id.Kd = 0.0f;
    Sguan0.Id.OutMax = 0.9f;
    Sguan0.Id.OutMin = -0.9f;
    Sguan0.Id.D_Filter = 0.2f;
    Sguan0.Id.ErrLimltFlag = 0;
    Sguan0.Id.ErrMax = 0.0f;
    Sguan0.Id.ErrMin = 0.0f;
    /* 7.电流环Q轴PID */
    Sguan0.Id.Kp = 0.02226f;
    Sguan0.Id.Ki = 0.000115f;
    Sguan0.Id.Kd = 0.0f;
    Sguan0.Id.OutMax = 0.9f;
    Sguan0.Id.OutMin = -0.9f;
    Sguan0.Id.D_Filter = 0.2f;
    Sguan0.Id.ErrLimltFlag = 0;
    Sguan0.Id.ErrMax = 0.0f;
    Sguan0.Id.ErrMin = 0.0f;
    /* 8.电流采样结构体变量 */
    Sguan0.Current.Dir_a = 0;
    Sguan0.Current.Dir_b = 0;
    Sguan0.Current.Dir_c = 0;
    Sguan0.Current.Id_Filter = 0.2f;
    Sguan0.Current.Iq_Filter = 0.2f;
    Sguan0.Current.VCC_Voltage = 3.3f;
    Sguan0.Current.Sampling_resistor = 0.005f;
    Sguan0.Current.Amplifier_Gain = 10;
    Sguan0.Current.ADC_Accuracy = 4096;
    /* 9.编码器结构体变量 */
    Sguan0.Encoder.Dir_n = 0;
    Sguan0.Encoder.Kalman_R_Filter = 10.0f;
    Sguan0.Encoder.Kalman_Q_Filter = 0.001f;
    #endif // SguanMotor0
    #ifdef SguanMotor1
    /* 1.控制模式 */
    Sguan1.Control_mode = VelCur_DOUBLE_MODE;
    /* 2.电机结构体变量 */
    Sguan1.Motor.Dir_n = 0;
    Sguan1.Motor.Polepairs = 7;
    /* 3.FOC结构体变量 */
    Sguan1.Foc.Dir_m = 0;
    Sguan1.Foc.Period = 4249;
    Sguan1.Foc.Target_Id = 0.0f;
    Sguan1.Foc.Target_Iq = 0.0f;
    Sguan1.Foc.Target_Speed = 0.0f;
    Sguan1.Foc.Target_Pos = 0.0f;
    Sguan1.Foc.Response_Num = 5;
    /* 4.位置环PID */
    Sguan1.Position.Kp = 0.15f;
    Sguan1.Position.Ki = 0.00000025f;
    Sguan1.Position.Kd = 0.0f;
    Sguan1.Position.OutMax = 0.35f;
    Sguan1.Position.OutMin = -0.35f;
    Sguan1.Position.D_Filter = 0.2f;
    Sguan1.Position.ErrLimltFlag = 0;
    Sguan1.Position.ErrMax = 0.0f;
    Sguan1.Position.ErrMin = 0.0f;
    /* 5.速度环PID */
    Sguan1.Speed.Kp = 0.02226f;
    Sguan1.Speed.Ki = 0.000115f;
    Sguan1.Speed.Kd = 0.0f;
    Sguan1.Speed.OutMax = 0.9f;
    Sguan1.Speed.OutMin = -0.9f;
    Sguan1.Speed.D_Filter = 0.2f;
    Sguan1.Speed.ErrLimltFlag = 0;
    Sguan1.Speed.ErrMax = 0.0f;
    Sguan1.Speed.ErrMin = 0.0f;
    /* 6.电流环D轴PID */
    Sguan1.Id.Kp = 0.02226f;
    Sguan1.Id.Ki = 0.000115f;
    Sguan1.Id.Kd = 0.0f;
    Sguan1.Id.OutMax = 0.9f;
    Sguan1.Id.OutMin = -0.9f;
    Sguan1.Id.D_Filter = 0.2f;
    Sguan1.Id.ErrLimltFlag = 0;
    Sguan1.Id.ErrMax = 0.0f;
    Sguan1.Id.ErrMin = 0.0f;
    /* 7.电流环Q轴PID */
    Sguan1.Id.Kp = 0.02226f;
    Sguan1.Id.Ki = 0.000115f;
    Sguan1.Id.Kd = 0.0f;
    Sguan1.Id.OutMax = 0.9f;
    Sguan1.Id.OutMin = -0.9f;
    Sguan1.Id.D_Filter = 0.2f;
    Sguan1.Id.ErrLimltFlag = 0;
    Sguan1.Id.ErrMax = 0.0f;
    Sguan1.Id.ErrMin = 0.0f;
    /* 8.电流采样结构体变量 */
    Sguan1.Current.Dir_a = 0;
    Sguan1.Current.Dir_b = 0;
    Sguan1.Current.Dir_c = 0;
    Sguan1.Current.Id_Filter = 0.2f;
    Sguan1.Current.Iq_Filter = 0.2f;
    Sguan1.Current.VCC_Voltage = 3.3f;
    Sguan1.Current.Sampling_resistor = 0.005f;
    Sguan1.Current.Amplifier_Gain = 10;
    Sguan1.Current.ADC_Accuracy = 4096;
    /* 9.编码器结构体变量 */
    Sguan1.Encoder.Dir_n = 0;
    Sguan1.Encoder.Kalman_R_Filter = 10.0f;
    Sguan1.Encoder.Kalman_Q_Filter = 0.001f;
    #endif // SguanMotor1
    #ifdef SguanMotor2
    /* 1.控制模式 */
    Sguan2.Control_mode = VelCur_DOUBLE_MODE;
    /* 2.电机结构体变量 */
    Sguan2.Motor.Dir_n = 0;
    Sguan2.Motor.Polepairs = 7;
    /* 3.FOC结构体变量 */
    Sguan2.Foc.Dir_m = 0;
    Sguan2.Foc.Period = 4249;
    Sguan2.Foc.Target_Id = 0.0f;
    Sguan2.Foc.Target_Iq = 0.0f;
    Sguan2.Foc.Target_Speed = 0.0f;
    Sguan2.Foc.Target_Pos = 0.0f;
    Sguan2.Foc.Response_Num = 5;
    /* 4.位置环PID */
    Sguan2.Position.Kp = 0.15f;
    Sguan2.Position.Ki = 0.00000025f;
    Sguan2.Position.Kd = 0.0f;
    Sguan2.Position.OutMax = 0.35f;
    Sguan2.Position.OutMin = -0.35f;
    Sguan2.Position.D_Filter = 0.2f;
    Sguan2.Position.ErrLimltFlag = 0;
    Sguan2.Position.ErrMax = 0.0f;
    Sguan2.Position.ErrMin = 0.0f;
    /* 5.速度环PID */
    Sguan2.Speed.Kp = 0.02226f;
    Sguan2.Speed.Ki = 0.000115f;
    Sguan2.Speed.Kd = 0.0f;
    Sguan2.Speed.OutMax = 0.9f;
    Sguan2.Speed.OutMin = -0.9f;
    Sguan2.Speed.D_Filter = 0.2f;
    Sguan2.Speed.ErrLimltFlag = 0;
    Sguan2.Speed.ErrMax = 0.0f;
    Sguan2.Speed.ErrMin = 0.0f;
    /* 6.电流环D轴PID */
    Sguan2.Id.Kp = 0.02226f;
    Sguan2.Id.Ki = 0.000115f;
    Sguan2.Id.Kd = 0.0f;
    Sguan2.Id.OutMax = 0.9f;
    Sguan2.Id.OutMin = -0.9f;
    Sguan2.Id.D_Filter = 0.2f;
    Sguan2.Id.ErrLimltFlag = 0;
    Sguan2.Id.ErrMax = 0.0f;
    Sguan2.Id.ErrMin = 0.0f;
    /* 7.电流环Q轴PID */
    Sguan2.Id.Kp = 0.02226f;
    Sguan2.Id.Ki = 0.000115f;
    Sguan2.Id.Kd = 0.0f;
    Sguan2.Id.OutMax = 0.9f;
    Sguan2.Id.OutMin = -0.9f;
    Sguan2.Id.D_Filter = 0.2f;
    Sguan2.Id.ErrLimltFlag = 0;
    Sguan2.Id.ErrMax = 0.0f;
    Sguan2.Id.ErrMin = 0.0f;
    /* 8.电流采样结构体变量 */
    Sguan2.Current.Dir_a = 0;
    Sguan2.Current.Dir_b = 0;
    Sguan2.Current.Dir_c = 0;
    Sguan2.Current.Id_Filter = 0.2f;
    Sguan2.Current.Iq_Filter = 0.2f;
    Sguan2.Current.VCC_Voltage = 3.3f;
    Sguan2.Current.Sampling_resistor = 0.005f;
    Sguan2.Current.Amplifier_Gain = 10;
    Sguan2.Current.ADC_Accuracy = 4096;
    /* 9.编码器结构体变量 */
    Sguan2.Encoder.Dir_n = 0;
    Sguan2.Encoder.Kalman_R_Filter = 10.0f;
    Sguan2.Encoder.Kalman_Q_Filter = 0.001f;
    #endif // SguanMotor2
    #ifdef SguanMotor3
    /* 1.控制模式 */
    Sguan3.Control_mode = VelCur_DOUBLE_MODE;
    /* 2.电机结构体变量 */
    Sguan3.Motor.Dir_n = 0;
    Sguan3.Motor.Polepairs = 7;
    /* 3.FOC结构体变量 */
    Sguan3.Foc.Dir_m = 0;
    Sguan3.Foc.Period = 4249;
    Sguan3.Foc.Target_Id = 0.0f;
    Sguan3.Foc.Target_Iq = 0.0f;
    Sguan3.Foc.Target_Speed = 0.0f;
    Sguan3.Foc.Target_Pos = 0.0f;
    Sguan3.Foc.Response_Num = 5;
    /* 4.位置环PID */
    Sguan3.Position.Kp = 0.15f;
    Sguan3.Position.Ki = 0.00000025f;
    Sguan3.Position.Kd = 0.0f;
    Sguan3.Position.OutMax = 0.35f;
    Sguan3.Position.OutMin = -0.35f;
    Sguan3.Position.D_Filter = 0.2f;
    Sguan3.Position.ErrLimltFlag = 0;
    Sguan3.Position.ErrMax = 0.0f;
    Sguan3.Position.ErrMin = 0.0f;
    /* 5.速度环PID */
    Sguan3.Speed.Kp = 0.02226f;
    Sguan3.Speed.Ki = 0.000115f;
    Sguan3.Speed.Kd = 0.0f;
    Sguan3.Speed.OutMax = 0.9f;
    Sguan3.Speed.OutMin = -0.9f;
    Sguan3.Speed.D_Filter = 0.2f;
    Sguan3.Speed.ErrLimltFlag = 0;
    Sguan3.Speed.ErrMax = 0.0f;
    Sguan3.Speed.ErrMin = 0.0f;
    /* 6.电流环D轴PID */
    Sguan3.Id.Kp = 0.02226f;
    Sguan3.Id.Ki = 0.000115f;
    Sguan3.Id.Kd = 0.0f;
    Sguan3.Id.OutMax = 0.9f;
    Sguan3.Id.OutMin = -0.9f;
    Sguan3.Id.D_Filter = 0.2f;
    Sguan3.Id.ErrLimltFlag = 0;
    Sguan3.Id.ErrMax = 0.0f;
    Sguan3.Id.ErrMin = 0.0f;
    /* 7.电流环Q轴PID */
    Sguan3.Id.Kp = 0.02226f;
    Sguan3.Id.Ki = 0.000115f;
    Sguan3.Id.Kd = 0.0f;
    Sguan3.Id.OutMax = 0.9f;
    Sguan3.Id.OutMin = -0.9f;
    Sguan3.Id.D_Filter = 0.2f;
    Sguan3.Id.ErrLimltFlag = 0;
    Sguan3.Id.ErrMax = 0.0f;
    Sguan3.Id.ErrMin = 0.0f;
    /* 8.电流采样结构体变量 */
    Sguan3.Current.Dir_a = 0;
    Sguan3.Current.Dir_b = 0;
    Sguan3.Current.Dir_c = 0;
    Sguan3.Current.Id_Filter = 0.2f;
    Sguan3.Current.Iq_Filter = 0.2f;
    Sguan3.Current.VCC_Voltage = 3.3f;
    Sguan3.Current.Sampling_resistor = 0.005f;
    Sguan3.Current.Amplifier_Gain = 10;
    Sguan3.Current.ADC_Accuracy = 4096;
    /* 9.编码器结构体变量 */
    Sguan3.Encoder.Dir_n = 0;
    Sguan3.Encoder.Kalman_R_Filter = 10.0f;
    Sguan3.Encoder.Kalman_Q_Filter = 0.001f;
    #endif // SguanMotor3
}


void SguanUser_Init(void) {
    /* Your code for initing TIM and gate driver and encoder and ADC here */
}

void SguanUser_Delay(uint32_t ms) {
    /* Your code for Delay_ms here */
}

uint32_t SguanUser_GetTick(void) {
    uint32_t Tick_num;
    /* Your code for getting time-tick here */
    return Tick_num;
}

uint32_t SguanUser_ReadADC_Raw(uint8_t Motor_CH,uint8_t Current_CH) {
    uint32_t ADC_num;
    switch (Motor_CH)
    {
    case 0:
        switch (Current_CH)
        {
        case 0:
            /* Your code for MotorA IA raw */
            break;
        case 1:
            /* Your code for MotorA IB raw */
            break;
        case 2:
            /* Your code for MotorA IC raw */
            break;
        default:
            break;
        }
        break;
    case 1:
        switch (Current_CH)
        {
        case 0:
            /* Your code for MotorB IA raw */
            break;
        case 1:
            /* Your code for MotorB IB raw */
            break;
        case 2:
            /* Your code for MotorB IC raw */
            break;
        default:
            break;
        }
        break;
    case 2:
        switch (Current_CH)
        {
        case 0:
            /* Your code for MotorC IA raw */
            break;
        case 1:
            /* Your code for MotorC IB raw */
            break;
        case 2:
            /* Your code for MotorC IC raw */
            break;
        default:
            break;
        }
        break;
    case 3:
        switch (Current_CH)
        {
        case 0:
            /* Your code for MotorD IA raw */
            break;
        case 1:
            /* Your code for MotorD IB raw */
            break;
        case 2:
            /* Your code for MotorD IC raw */
            break;
        default:
            break;
        }
        break;
    default:
        break;
    }
    return ADC_num;
}

float SguanUser_Encoder_ReadRad(uint8_t Motor_CH) {
    float Rad_num;
    switch (Motor_CH)
    {
    case 0:
        /* Your code for getting encoder CH0 radian position */
        break;
    case 1:
        /* Your code for getting encoder CH1 radian position */
        break;
    case 2:
        /* Your code for getting encoder CH2 radian position */
        break;
    case 3:
        /* Your code for getting encoder CH3 radian position */
        break;
    default:
        break;
    }
    return Rad_num;
}

void SguanUser_PwmDuty_Set(uint8_t Motor_CH,uint8_t PWM_CH,uint16_t Duty) {
    switch (Motor_CH)
    {
    case 0:
        switch (PWM_CH)
        {
        case 0:
            /* Your code for MotorA PWM_CH0 duty set */
            break;
        case 1:
            /* Your code for MotorA PWM_CH1 duty set */
            break;
        case 2:
            /* Your code for MotorA PWM_CH2 duty set */
            break;
        default:
            break;
        }
        break;
    case 1:
        switch (PWM_CH)
        {
        case 0:
            /* Your code for MotorB PWM_CH0 duty set */
            break;
        case 1:
            /* Your code for MotorB PWM_CH1 duty set */
            break;
        case 2:
            /* Your code for MotorB PWM_CH2 duty set */
            break;
        default:
            break;
        }
        break;
    case 2:
        switch (PWM_CH)
        {
        case 0:
            /* Your code for MotorC PWM_CH0 duty set */
            break;
        case 1:
            /* Your code for MotorC PWM_CH1 duty set */
            break;
        case 2:
            /* Your code for MotorC PWM_CH2 duty set */
            break;
        default:
            break;
        }
        break;
    case 3:
        switch (PWM_CH)
        {
        case 0:
            /* Your code for MotorD PWM_CH0 duty set */
            break;
        case 1:
            /* Your code for MotorD PWM_CH1 duty set */
            break;
        case 2:
            /* Your code for MotorD PWM_CH2 duty set */
            break;
        default:
            break;
        }
        break;
    default:
        break;
    }
}


/*
void Sguan_FocInit(void);             // 主函数初始化调用
void SguanFOC_Run_Tick(void);         // 定时1ms中断调用
void SguanFOC_GeneratePWM_Loop(void); // ADC采样完成中断调用(高于or等于1ms中断频率)
*/

#endif // SGUANUSER_DATA_H

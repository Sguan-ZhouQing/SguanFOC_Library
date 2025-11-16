/*
 * @Author: 星必尘Sguan
 * @Date: 2025-11-14 09:31:21
 * @LastEditors: 星必尘Sguan|3464647102@qq.com
 * @LastEditTime: 2025-11-16 13:32:55
 * @FilePath: \demo_SguanFOCv2.0\SguanFOC\SguanFOC.c
 * @Description: SguanFOC的“电机运算”库
 * 
 * Copyright (c) 2025 by $JUST, All Rights Reserved. 
 */
#include "SguanFOC.h"
// 外部函数包含
#include "SguanUser_Data.h"

#ifdef SguanMotor0
Motor_System_STRUCT Sguan0;
#endif // SguanMotor0
#ifdef SguanMotor1
Motor_System_STRUCT Sguan1;
#endif // SguanMotor1
#ifdef SguanMotor2
Motor_System_STRUCT Sguan2;
#endif // SguanMotor2
#ifdef SguanMotor3
Motor_System_STRUCT Sguan3;
#endif // SguanMotor3

static void Sguan_ReadADC_Offset_Init(void);
static void Sguan_ReadADC_Offset(Motor_System_STRUCT *sguan, uint32_t Motor_CH);
static void SguanUser_ReadRad_Offset_Init(void);
static void Sguan_SetDuty_Init(float du, float dv, float dw);
static void Sguan_SetDuty(uint8_t Motor_CH, float d_u, float d_v, float d_w);
static void copy_duty(Motor_System_STRUCT *sguan, float d_u, float d_v, float d_w);
static void set_pwm(Motor_System_STRUCT *sguan, uint8_t Motor_CH);
static float Sguan_Encoder_GetRad(Motor_System_STRUCT *sguan, uint8_t Motor_CH);
static float Sguan_Encoder_GetElRad(Motor_System_STRUCT *sguan, uint8_t Motor_CH);
static float Sguan_Encoder_GetPos(Motor_System_STRUCT *sguan, uint8_t Motor_CH);
static void Sguan_Encoder_ReadSpeed(Motor_System_STRUCT *sguan, uint8_t Motor_CH, float *real_speed);
static float Sguan_Encoder_FilterSpeed(Motor_System_STRUCT *sguan, uint8_t Motor_CH);
static void Sguan_Encoder_Tick(void);
static void Sguan_Current_Calculate(Motor_System_STRUCT *sguan, uint8_t Motor_CH);
static void Sguan_Current_Tick(void);
static void SguanFOC_Mode_PIDCalculate(Motor_System_STRUCT *sguan);

// PID闭环运算函数
static void PID_Control(PID_STRUCT *p) {
    p->Err = p->Ref - p->Fbk;
    if(p->ErrLimltFlag == 1) {
        p->Err = Value_Limit(p->Err, p->ErrMax, p->ErrMin);
    }
    // 积分抗饱和
    if(p->Out != p->OutMax && p->Out != p->OutMin) {
        p->Ui += p->Err;
    }
    // 微分项计算（带低通滤波，D_filter通常在0.1~0.9之间，值越小滤波效果越强）
    float derivative_term = p->Err - p->ErrLast;
    if(p->D_Filter > 0) {
        derivative_term = p->D_Filter * derivative_term + (1 - p->D_Filter) * p->LastDerivative;
        p->LastDerivative = derivative_term;
    }
    // PID计算
    p->Out = p->Kp * p->Err + p->Ki * p->Ui + p->Kd * derivative_term;
    p->ErrLast = p->Err;
    // 输出限幅
    p->Out = Value_Limit(p->Out, p->OutMax, p->OutMin);
}

// 电流采样ADC偏置初始化
static void Sguan_ReadADC_Offset_Init(void) {
    #ifdef SguanMotor0
    Sguan_ReadADC_Offset(&Sguan0,0);
    Sguan0.Current.Final_Gain = Sguan0.Current.VCC_Voltage*Sguan0.Current.Amplifier_Gain/Sguan0.Current.Sampling_resistor/Sguan0.Current.ADC_Accuracy;
    #endif // SguanMotor0
    #ifdef SguanMotor1
    Sguan_ReadADC_Offset(&Sguan1,1);
    Sguan1.Current.Final_Gain = Sguan1.Current.VCC_Voltage*Sguan1.Current.Amplifier_Gain/Sguan1.Current.Sampling_resistor/Sguan1.Current.ADC_Accuracy;
    #endif // SguanMotor1
    #ifdef SguanMotor2
    Sguan_ReadADC_Offset(&Sguan2,2);
    Sguan2.Current.Final_Gain = Sguan2.Current.VCC_Voltage*Sguan2.Current.Amplifier_Gain/Sguan2.Current.Sampling_resistor/Sguan2.Current.ADC_Accuracy;
    #endif // SguanMotor2
    #ifdef SguanMotor3
    Sguan_ReadADC_Offset(&Sguan3,3);
    Sguan3.Current.Final_Gain = Sguan3.Current.VCC_Voltage*Sguan3.Current.Amplifier_Gain/Sguan3.Current.Sampling_resistor/Sguan3.Current.ADC_Accuracy;
    #endif // SguanMotor3
}

// 读取零电流ADC数据
static void Sguan_ReadADC_Offset(Motor_System_STRUCT *sguan,uint32_t Motor_CH) {
    sguan->Current.Offset_a = sguan->Current.Dir_a*SguanUser_ReadADC_Raw(Motor_CH,0);
    sguan->Current.Offset_b = sguan->Current.Dir_b*SguanUser_ReadADC_Raw(Motor_CH,1);
    sguan->Current.Offset_c = sguan->Current.Dir_c*SguanUser_ReadADC_Raw(Motor_CH,2);
}

// 编码器Encoder偏置初始化
static void SguanUser_ReadRad_Offset_Init(void) {
    #ifdef SguanMotor0
    Sguan0.Encoder.Offset = SguanUser_Encoder_ReadRad(0);
    #endif // SguanMotor0
    #ifdef SguanMotor1
    Sguan1.Encoder.Offset = SguanUser_Encoder_ReadRad(1);
    #endif // SguanMotor1
    #ifdef SguanMotor2
    Sguan2.Encoder.Offset = SguanUser_Encoder_ReadRad(2);
    #endif // SguanMotor2
    #ifdef SguanMotor3
    Sguan3.Encoder.Offset = SguanUser_Encoder_ReadRad(3);
    #endif // SguanMotor3
}

// 全局Duty设置函数
static void Sguan_SetDuty_Init(float du,float dv,float dw) {
    Sguan_SetDuty(0,du,dv,dw);
    Sguan_SetDuty(1,du,dv,dw);
    Sguan_SetDuty(2,du,dv,dw);
    Sguan_SetDuty(3,du,dv,dw);
}

// 设置UWV电机通电
static void Sguan_SetDuty(uint8_t Motor_CH,float d_u,float d_v,float d_w) {
    switch (Motor_CH)
    {
    case 0:
        #ifdef SguanMotor0
        copy_duty(&Sguan0,d_u,d_v,d_w);
        #endif // SguanMotor0
        break;
    case 1:
        #ifdef SguanMotor1
        copy_duty(&Sguan1,d_u,d_v,d_w);
        #endif // SguanMotor1
        break;
    case 2:
        #ifdef SguanMotor2
        copy_duty(&Sguan2,d_u,d_v,d_w);
        #endif // SguanMotor2
        break;
    case 3:
        #ifdef SguanMotor3
        copy_duty(&Sguan3,d_u,d_v,d_w);
        #endif // SguanMotor3
        break;
    default:
        break;
    }
    switch (Motor_CH)
    {
    case 0:
        #ifdef SguanMotor0
        set_pwm(&Sguan0,0);
        #endif // SguanMotor0
        break;
    case 1:
        #ifdef SguanMotor1
        set_pwm(&Sguan1,1);
        #endif // SguanMotor1
        break;
    case 2:
        #ifdef SguanMotor2
        set_pwm(&Sguan2,2);
        #endif // SguanMotor2
        break;
    case 3:
        #ifdef SguanMotor3
        set_pwm(&Sguan3,3);
        #endif // SguanMotor3
        break;
    default:
        break;
    }
}

// 赋值Period给电机结构体变量
static void copy_duty(Motor_System_STRUCT *sguan,float d_u,float d_v,float d_w) {
    if (sguan->Foc.Dir_m) {
        sguan->Motor.du = (uint16_t)(sguan->Foc.Period * (1.0f - d_u));
        sguan->Motor.dv = (uint16_t)(sguan->Foc.Period * (1.0f - d_v));
        sguan->Motor.dw = (uint16_t)(sguan->Foc.Period * (1.0f - d_w));
    }
    else {
        sguan->Motor.du = (uint16_t)(sguan->Foc.Period * d_u);
        sguan->Motor.dv = (uint16_t)(sguan->Foc.Period * d_v);
        sguan->Motor.dw = (uint16_t)(sguan->Foc.Period * d_w);
    }
    if (sguan->Motor.Dir_n) {
        uint16_t temp = sguan->Motor.dv;
        sguan->Motor.dv = sguan->Motor.dw;
        sguan->Motor.dw = temp;
    }
}

// 设置电机实际PWM占空比
static void set_pwm(Motor_System_STRUCT *sguan,uint8_t Motor_CH) {
    SguanUser_PwmDuty_Set(Motor_CH,0,sguan->Motor.du);
    SguanUser_PwmDuty_Set(Motor_CH,1,sguan->Motor.dv);
    SguanUser_PwmDuty_Set(Motor_CH,2,sguan->Motor.dw);
}

// 获取实际电机单圈角度
static float Sguan_Encoder_GetRad(Motor_System_STRUCT *sguan,uint8_t Motor_CH) {
    if (sguan->Encoder.Dir_n) {
        float rad_count = -(SguanUser_Encoder_ReadRad(Motor_CH) - sguan->Encoder.Offset);
        return normalize_angle(rad_count);
    }
    else {
        float rad_count = -(SguanUser_Encoder_ReadRad(Motor_CH) - sguan->Encoder.Offset);
        return rad_count;
    }
}

static float Sguan_Encoder_GetElRad(Motor_System_STRUCT *sguan,uint8_t Motor_CH) {
    return normalize_angle(Sguan_Encoder_GetRad(sguan,Motor_CH)*sguan->Motor.Polepairs);
}


// 获取实际电机多圈角度
static float Sguan_Encoder_GetPos(Motor_System_STRUCT *sguan,uint8_t Motor_CH) {
    return Sguan_Encoder_GetRad(sguan,Motor_CH) + sguan->Encoder.PosFlag*Value_PI*2;
}

// 获取实际电机速度(未滤波)
static void Sguan_Encoder_ReadSpeed(Motor_System_STRUCT *sguan,uint8_t Motor_CH,float *real_speed) {
    float This_Count = Sguan_Encoder_GetRad(sguan,Motor_CH);
    uint32_t This_Time = SguanUser_GetTick();
    float Encoder_num =  This_Count - sguan->Encoder.LastRad;
    int32_t Encoder_time = This_Time - sguan->Encoder.LastTick;
    if (Encoder_num >= Value_PI*1.5f) {
        Encoder_num -= Value_PI*2;
        sguan->Encoder.PosFlag--;
    }
    if (Encoder_num <= -Value_PI*1.5f) {
        Encoder_num += Value_PI*2;
        sguan->Encoder.PosFlag++;
    }
    if (Encoder_time) {
        *real_speed = (Encoder_num*1000)/Encoder_time;
    }
    sguan->Encoder.LastRad = This_Count;
    sguan->Encoder.LastTick = This_Time;
}

// 获取实际电机速度(已滤波)
static float Sguan_Encoder_FilterSpeed(Motor_System_STRUCT *sguan,uint8_t Motor_CH) {
    static float data = 0;
    Sguan_Encoder_ReadSpeed(sguan,Motor_CH,&data);
    return kalman_filter_std(Motor_CH, data, sguan->Encoder.Kalman_R_Filter, sguan->Encoder.Kalman_Q_Filter);
}

// 电机速度Tick函数
static void Sguan_Encoder_Tick(void) {
    #ifdef SguanMotor0
    Sguan0.Encoder.Real_Speed = Sguan_Encoder_FilterSpeed(&Sguan0,0);
    Sguan0.Encoder.Real_Pos = Sguan_Encoder_GetPos(&Sguan0,0);
    #endif // SguanMotor0
    #ifdef SguanMotor1
    Sguan1.Encoder.Real_Speed = Sguan_Encoder_FilterSpeed(&Sguan1,1);
    Sguan1.Encoder.Real_Pos = Sguan_Encoder_GetPos(&Sguan1,0);
    #endif // SguanMotor1
    #ifdef SguanMotor2
    Sguan2.Encoder.Real_Speed = Sguan_Encoder_FilterSpeed(&Sguan2,2);
    Sguan2.Encoder.Real_Pos = Sguan_Encoder_GetPos(&Sguan2,0);
    #endif // SguanMotor2
    #ifdef SguanMotor3
    Sguan3.Encoder.Real_Speed = Sguan_Encoder_FilterSpeed(&Sguan3,3);
    Sguan3.Encoder.Real_Pos = Sguan_Encoder_GetPos(&Sguan3,0);
    #endif // SguanMotor3
}

// 更新相电流并计算DQ电流大小
static void Sguan_Current_Calculate(Motor_System_STRUCT *sguan,uint8_t Motor_CH) {
    if (sguan->Current.Dir_a) {
        sguan->Current.Real_Iu = -(SguanUser_ReadADC_Raw(Motor_CH,0) - sguan->Current.Offset_a)*sguan->Current.Final_Gain;
        sguan->Current.Real_Iv = -(SguanUser_ReadADC_Raw(Motor_CH,1) - sguan->Current.Offset_b)*sguan->Current.Final_Gain;
        sguan->Current.Real_Iw = -(SguanUser_ReadADC_Raw(Motor_CH,2) - sguan->Current.Offset_c)*sguan->Current.Final_Gain;
    }
    else {
        sguan->Current.Real_Iu = (SguanUser_ReadADC_Raw(Motor_CH,0) - sguan->Current.Offset_a)*sguan->Current.Final_Gain;
        sguan->Current.Real_Iv = (SguanUser_ReadADC_Raw(Motor_CH,1) - sguan->Current.Offset_b)*sguan->Current.Final_Gain;
        sguan->Current.Real_Iw = (SguanUser_ReadADC_Raw(Motor_CH,2) - sguan->Current.Offset_c)*sguan->Current.Final_Gain;
    }
    sguan->Encoder.Real_El_Rad = Sguan_Encoder_GetElRad(sguan,Motor_CH);
    fast_sin_cos(sguan->Encoder.Real_El_Rad,&sguan->Encoder.Real_Sine,&sguan->Encoder.Real_Cosine);
    if (sguan->Foc.Dir_m) {
        clarke(&sguan->Current.Real_Ialpha,&sguan->Current.Real_Ibeta,sguan->Current.Real_Iu,sguan->Current.Real_Iw);
    }
    else {
        clarke(&sguan->Current.Real_Ialpha,&sguan->Current.Real_Ibeta,sguan->Current.Real_Iu,sguan->Current.Real_Iv);
    }
    park(&sguan->Current.Raw_Id,&sguan->Current.Raw_Iq,sguan->Current.Real_Ialpha,sguan->Current.Real_Ibeta,sguan->Encoder.Real_Sine,sguan->Encoder.Real_Cosine);
    low_pass_filter(sguan->Current.Raw_Id,sguan->Current.Real_Id,sguan->Current.Id_Filter);
    low_pass_filter(sguan->Current.Raw_Iq,sguan->Current.Real_Iq,sguan->Current.Iq_Filter);
}

// 电流采样Tick函数
static void Sguan_Current_Tick(void) {
    #ifdef SguanMotor0
    Sguan_Current_Calculate(&Sguan0,0);
    #endif // SguanMotor0
    #ifdef SguanMotor1
    Sguan_Current_Calculate(&Sguan1,1);
    #endif // SguanMotor1
    #ifdef SguanMotor2
    Sguan_Current_Calculate(&Sguan2,2);
    #endif // SguanMotor2
    #ifdef SguanMotor3
    Sguan_Current_Calculate(&Sguan3,3);
    #endif // SguanMotor3
}

// 多模式PID运算函数实现
static void SguanFOC_Mode_PIDCalculate(Motor_System_STRUCT *sguan) {
    switch (sguan->Control_mode)
    {
    case Current_SINGLE_MODE:
        sguan->Id.Ref = sguan->Foc.Target_Id;
        sguan->Iq.Ref = sguan->Foc.Target_Iq;
        sguan->Id.Fbk = sguan->Current.Real_Id;
        sguan->Iq.Fbk = sguan->Current.Real_Iq;
        PID_Control(&sguan->Id);
        PID_Control(&sguan->Iq);
        sguan->Foc.Id_Out = sguan->Id.Out;
        sguan->Foc.Iq_Out = sguan->Iq.Out;
        break;
    case Velocity_SINGLE_MODE:
        sguan->Speed.Ref = sguan->Foc.Target_Speed;
        sguan->Speed.Fbk = sguan->Encoder.Real_Speed;
        PID_Control(&sguan->Speed);
        sguan->Foc.Id_Out = 0;
        sguan->Foc.Iq_Out = sguan->Speed.Out;
        break;
    case Position_SINGLE_MODE:
        sguan->Position.Ref = sguan->Foc.Target_Pos;
        sguan->Position.Fbk = sguan->Encoder.Real_Pos;
        PID_Control(&sguan->Position);
        sguan->Foc.Id_Out = 0;
        sguan->Foc.Iq_Out = sguan->Position.Out;
        break;
    case VelCur_DOUBLE_MODE:
        sguan->Foc.Response_Count++;
        if (sguan->Foc.Response_Count == sguan->Foc.Response_Num) {
            sguan->Speed.Ref = sguan->Foc.Target_Speed;
            sguan->Speed.Fbk = sguan->Encoder.Real_Speed;
            PID_Control(&sguan->Speed);
            sguan->Foc.Response_Count = 0;
        }
        sguan->Id.Ref = 0;
        sguan->Iq.Ref = sguan->Speed.Out;
        sguan->Id.Fbk = sguan->Current.Real_Id;
        sguan->Iq.Fbk = sguan->Current.Real_Iq;
        PID_Control(&sguan->Id);
        PID_Control(&sguan->Iq);
        sguan->Foc.Id_Out = sguan->Id.Out;
        sguan->Foc.Iq_Out = sguan->Iq.Out;
        break;
    case PosVel_DOUBLE_MODE:
        sguan->Foc.Response_Count++;
        if (sguan->Foc.Response_Count == sguan->Foc.Response_Num) {
            sguan->Position.Ref = sguan->Foc.Target_Pos;
            sguan->Position.Fbk = sguan->Encoder.Real_Pos;
            PID_Control(&sguan->Position);
            sguan->Foc.Response_Count = 0;
        }
        sguan->Speed.Ref = sguan->Position.Out;
        sguan->Speed.Fbk = sguan->Encoder.Real_Speed;
        PID_Control(&sguan->Speed);
        sguan->Foc.Id_Out = 0;
        sguan->Foc.Iq_Out = sguan->Speed.Out;
        break;
    case PosVelCur_THREE_MODE:
        sguan->Foc.Response_Count++;
        if (sguan->Foc.Response_Count == (sguan->Foc.Response_Num*sguan->Foc.Response_Num)) {
            sguan->Position.Ref = sguan->Foc.Target_Pos;
            sguan->Position.Fbk = sguan->Encoder.Real_Pos;
            PID_Control(&sguan->Position);
            sguan->Foc.Response_Count = 0;
        }
        if (sguan->Foc.Response_Count % 5 == 0) {
            sguan->Speed.Ref = sguan->Position.Out;
            sguan->Speed.Fbk = sguan->Encoder.Real_Speed;
            PID_Control(&sguan->Speed);
        }
        sguan->Id.Ref = 0;
        sguan->Iq.Ref = sguan->Speed.Out;
        sguan->Id.Fbk = sguan->Current.Real_Id;
        sguan->Iq.Fbk = sguan->Current.Real_Iq;
        PID_Control(&sguan->Id);
        PID_Control(&sguan->Iq);
        sguan->Foc.Id_Out = sguan->Id.Out;
        sguan->Foc.Iq_Out = sguan->Iq.Out;
        break;
    default:
        break;
    }
}

// FOC磁场控制初始化（主函数调用）
void Sguan_FocInit(void) {
    // 定时器,栅极驱动,PWM,ADC的初始化
    SguanUser_Init();
    // 电流偏置Offset采集
    Sguan_ReadADC_Offset_Init();
    // d轴强拖，初始化对齐电角度
    float Init_du,Init_dv,Init_dw;
    SVPWM(0,0.2f,0,&Init_du,&Init_dv,&Init_dw);
    Sguan_SetDuty_Init(Init_du,Init_dv,Init_dw);
    SguanUser_Delay(600);
    // 转子位置偏置Offset采集
    SguanUser_ReadRad_Offset_Init();
    // 还原电机磁场
    SVPWM(0,0,0,&Init_du,&Init_dv,&Init_dw);
    Sguan_SetDuty_Init(Init_du,Init_dv,Init_dw);
    // PID和电机参数设定
    Sguan_ParameterSet();
}

// 中断服务函数（1ms中断调用）
void SguanFOC_Run_Tick(void) {
    Sguan_Encoder_Tick();
    Sguan_Current_Tick();
    #ifdef SguanMotor0
    SguanFOC_Mode_PIDCalculate(&Sguan0);
    #endif // SguanMotor0
    #ifdef SguanMotor1
    SguanFOC_Mode_PIDCalculate(&Sguan1);
    #endif // SguanMotor1
    #ifdef SguanMotor2
    SguanFOC_Mode_PIDCalculate(&Sguan2);
    #endif // SguanMotor2
    #ifdef SguanMotor3
    SguanFOC_Mode_PIDCalculate(&Sguan3);
    #endif // SguanMotor3
}

// ADC采样完成中断服务函数（大于or等于1ms中断调用）
void SguanFOC_GeneratePWM_Loop(void) {
    #ifdef SguanMotor0
    Sguan0.Encoder.Real_El_Rad = Sguan_Encoder_GetElRad(&Sguan0,0);
    SVPWM(Sguan0.Encoder.Real_El_Rad,  // phi: 电角度
        Sguan0.Foc.Id_Out,          // d: d轴电压
        Sguan0.Foc.Iq_Out,          // q: q轴电压  
        &Sguan0.Foc.Input_du,       // d_u: U相占空比输出
        &Sguan0.Foc.Input_dv,       // d_v: V相占空比输出
        &Sguan0.Foc.Input_dw);      // d_w: W相占空比输出
    Sguan_SetDuty(0,Sguan0.Foc.Input_du,Sguan0.Foc.Input_dv,Sguan0.Foc.Input_dw);
    #endif // SguanMotor0
    #ifdef SguanMotor1
    Sguan1.Encoder.Real_El_Rad = Sguan_Encoder_GetElRad(&Sguan1,1);
    SVPWM(Sguan1.Encoder.Real_El_Rad,  // phi: 电角度
        Sguan1.Foc.Id_Out,          // d: d轴电压
        Sguan1.Foc.Iq_Out,          // q: q轴电压  
        &Sguan1.Foc.Input_du,       // d_u: U相占空比输出
        &Sguan1.Foc.Input_dv,       // d_v: V相占空比输出
        &Sguan1.Foc.Input_dw);      // d_w: W相占空比输出
    Sguan_SetDuty(1,Sguan1.Foc.Input_du,Sguan1.Foc.Input_dv,Sguan1.Foc.Input_dw);
    #endif // SguanMotor1
    #ifdef SguanMotor2
    Sguan2.Encoder.Real_El_Rad = Sguan_Encoder_GetElRad(&Sguan2,2);
    SVPWM(Sguan2.Encoder.Real_El_Rad,  // phi: 电角度
        Sguan2.Foc.Id_Out,          // d: d轴电压
        Sguan2.Foc.Iq_Out,          // q: q轴电压  
        &Sguan2.Foc.Input_du,       // d_u: U相占空比输出
        &Sguan2.Foc.Input_dv,       // d_v: V相占空比输出
        &Sguan2.Foc.Input_dw);      // d_w: W相占空比输出
    Sguan_SetDuty(2,Sguan2.Foc.Input_du,Sguan2.Foc.Input_dv,Sguan2.Foc.Input_dw);
    #endif // SguanMotor2
    #ifdef SguanMotor3
    Sguan3.Encoder.Real_El_Rad = Sguan_Encoder_GetElRad(&Sguan3,3);
    SVPWM(Sguan3.Encoder.Real_El_Rad,  // phi: 电角度
        Sguan3.Foc.Id_Out,          // d: d轴电压
        Sguan3.Foc.Iq_Out,          // q: q轴电压  
        &Sguan3.Foc.Input_du,       // d_u: U相占空比输出
        &Sguan3.Foc.Input_dv,       // d_v: V相占空比输出
        &Sguan3.Foc.Input_dw);      // d_w: W相占空比输出
    Sguan_SetDuty(3,Sguan3.Foc.Input_du,Sguan3.Foc.Input_dv,Sguan3.Foc.Input_dw);
    #endif // SguanMotor3
}

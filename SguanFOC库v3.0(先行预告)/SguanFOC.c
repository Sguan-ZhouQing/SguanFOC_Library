/*
 * @Author: 星必尘Sguan
 * @GitHub: https://github.com/Sguan-ZhouQing
 * @Date: 2026-01-26 22:38:34
 * @LastEditors: 星必尘Sguan|3464647102@qq.com
 * @LastEditTime: 2026-01-30 14:50:06
 * @FilePath: \demo_SguanFOCCode\SguanFOC库\SguanFOC.c
 * @Description: SguanFOC库的“核心代码”实现
 * 
 * Copyright (c) 2026 by $星必尘Sguan, All Rights Reserved. 
 */
#include "SguanFOC.h"

/* USER CODE BEGIN Includes */
// 电机控制User用户设置声明
#include "UserData_Function.h"
#include "UserData_Motor.h"
#include "UserData_Parameter.h"
#include "UserData_UserControl.h"
/* USER CODE END Includes */

// 电机控制核心结构体设计
SguanFOC_System_STRUCT Sguan = {0};

/**
 * @description: 1.Offset内部静态函数声明
 * @param {SguanFOC_System_STRUCT} *sguan
 * @return {*}
 */
static void Offset_EncoderRead(SguanFOC_System_STRUCT *sguan);
static void Offset_CurrentRead(SguanFOC_System_STRUCT *sguan);
/**
 * @description: 2.Encoder内部静态函数声明
 * @param {SguanFOC_System_STRUCT} *sguan
 * @return {*}
 */
static float Encoder_ReadErad(SguanFOC_System_STRUCT *sguan);
static float Encoder_ReadPos(SguanFOC_System_STRUCT *sguan);
static float Encoder_ReadSpeed(SguanFOC_System_STRUCT *sguan);
static void Encoder_Tick(SguanFOC_System_STRUCT *sguan);
/**
 * @description: 3.Current内部静态函数声明
 * @param {SguanFOC_System_STRUCT} *sguan
 * @return {*}
 */
static void Current_ReadIabc(SguanFOC_System_STRUCT *sguan);
static void Current_Tick(SguanFOC_System_STRUCT *sguan);
/**
 * @description: 4.PID运算及其模式切换
 * @param {SguanFOC_System_STRUCT} *sguan
 * @return {*}
 */
static void PID_Velocity_OPEN(SguanFOC_System_STRUCT *sguan);
static void PID_Current_SINGLE(SguanFOC_System_STRUCT *sguan);
static void PID_Velocity_SINGLE(SguanFOC_System_STRUCT *sguan);
static void PID_Position_SINGLE(SguanFOC_System_STRUCT *sguan);
static void PID_VelCur_DOUBLE(SguanFOC_System_STRUCT *sguan);
static void PID_PosVel_DOUBLE(SguanFOC_System_STRUCT *sguan);
static void PID_PosVelCur_THREE(SguanFOC_System_STRUCT *sguan);
static void (*const PID_Tick[])(SguanFOC_System_STRUCT*)={
    /*这里注意“枚举变量”的边界, PID_Tick[sguan->mode](sguan)使用*/
    PID_Velocity_OPEN,      // Velocity_OPEN_MODE = 0
    PID_Current_SINGLE,     // Current_SINGLE_MODE = 1
    PID_Velocity_SINGLE,    // Velocity_SINGLE_MODE = 2
    PID_Position_SINGLE,    // Position_SINGLE_MODE = 3
    PID_VelCur_DOUBLE,      // VelCur_DOUBLE_MODE = 4
    PID_PosVel_DOUBLE,      // PosVel_DOUBLE_MODE = 5
    PID_PosVelCur_THREE     // PosVelCur_THREE_MODE = 6
};
/**
 * @description: 5.Data母线电压和驱动器物理温度数据更新和滤波
 * @param {SguanFOC_System_STRUCT} *sguan
 * @return {*}
 */
static void Data_Protection_Loop(SguanFOC_System_STRUCT *sguan);
/**
 * @description: 6.SVPWM电机驱动的马鞍波生成
 * @param {SguanFOC_System_STRUCT} *sguan
 * @param {float} d
 * @param {float} q
 * @return {*}
 */
static void SVPWM_Tick(SguanFOC_System_STRUCT *sguan,float Erad,float d_set,float q_set);
/**
 * @description: 7.Sguan...Loop定时计算并执行
 * @param {SguanFOC_System_STRUCT} *sguan
 * @return {*}
 */
static void Sguan_Calculate_Loop(SguanFOC_System_STRUCT *sguan);
static void Sguan_GeneratePWM_Loop(SguanFOC_System_STRUCT *sguan);
/**
 * @description: 8.Sguan...Init各种控制系统的初始化
 * @param {SguanFOC_System_STRUCT} *sguan
 * @return {*}
 */
static void Sguan_Quantize_Init(void);
static void Sguan_BPF_Init(SguanFOC_System_STRUCT *sguan);
static void Sguan_PID_Init(SguanFOC_System_STRUCT *sguan);
static void Sguan_PLL_Init(SguanFOC_System_STRUCT *sguan);
static void Sguan_Sensorless_Init(SguanFOC_System_STRUCT *sguan);
/**
 * @description: 9.Pre_Pos电机零点对齐
 * @param {SguanFOC_System_STRUCT} *sguan
 * @return {*}
 */
static void Sguan_Pre_Positioning(SguanFOC_System_STRUCT *sguan,float d_set);


// Offset读取编码器偏置
static void Offset_EncoderRead(SguanFOC_System_STRUCT *sguan){
    sguan->encoder.Pos_offset = User_Encoder_ReadRad();
}

// Offset读取电流偏置
static void Offset_CurrentRead(SguanFOC_System_STRUCT *sguan){
    for (uint8_t i = 0; i < 128; i++){
        sguan->current.Pos_offset0 += User_ReadADC_Raw(0);
        sguan->current.Pos_offset1 += User_ReadADC_Raw(1);
        User_Delay(3);
    }
    sguan->current.Pos_offset0 = sguan->current.Pos_offset0/128;
    sguan->current.Pos_offset1 = sguan->current.Pos_offset1/128;
    sguan->current.Final_Gain = sguan->motor.MCU_Voltage/
        (sguan->motor.ADC_Precision*sguan->motor.Amplifier*sguan->motor.Sampling_Rs);
}

// Encoder获取实际单圈电子角度(无需滤波)
static float Encoder_ReadErad(SguanFOC_System_STRUCT *sguan){
    return normalize_angle(sguan->encoder.Real_Rad*sguan->motor.Poles);
}

// Encoder获取实际多圈机械角度(无需滤波)
static float Encoder_ReadPos(SguanFOC_System_STRUCT *sguan){
    return (sguan->encoder.Real_Rad - sguan->encoder.Pos_offset) + 2*Value_PI*sguan->encoder.Pos_Flag;
}

// Encoder获取实际电机速度(未滤波)
static float Encoder_ReadSpeed(SguanFOC_System_STRUCT *sguan){
    float This_Count = sguan->encoder.Real_Rad;
    float Encoder_num = This_Count - sguan->encoder.Last_Rad;
    sguan->encoder.Last_Rad = This_Count;
    if (Encoder_num >= Value_PI) {
        Encoder_num -= Value_PI*2;
        if (sguan->mode == Position_SINGLE_MODE || 
            sguan->mode == PosVel_DOUBLE_MODE || 
            sguan->mode == PosVelCur_THREE_MODE){
            sguan->encoder.Pos_Flag--;
        }
    }
    if (Encoder_num <= -Value_PI) {
        Encoder_num += Value_PI*2;
        if (sguan->mode == Position_SINGLE_MODE || 
            sguan->mode == PosVel_DOUBLE_MODE || 
            sguan->mode == PosVelCur_THREE_MODE){
            sguan->encoder.Pos_Flag++;
        }
    }
    return Encoder_num/Sguan.motor.Encoder_T;
}

// Encoder实时更新角速度和角度信息(已滤波)
static void Encoder_Tick(SguanFOC_System_STRUCT *sguan){
    sguan->encoder.Real_Rad = User_Encoder_ReadRad();
    sguan->bpf.Encoder.filter.Input = Encoder_ReadSpeed(sguan)*sguan->motor.Motor_Dir;
    BPF_Loop(&sguan->bpf.Encoder);
    sguan->encoder.Real_Speed = sguan->bpf.Encoder.filter.Output;
    sguan->encoder.Real_Pos = Encoder_ReadPos(sguan)*sguan->motor.Motor_Dir;
    sguan->encoder.Real_Erad = Encoder_ReadErad(sguan);
}

// Current读取当前的电流值并更新3相电流(已滤波)
static void Current_ReadIabc(SguanFOC_System_STRUCT *sguan){
    sguan->bpf.Current0.filter.Input = (User_ReadADC_Raw(0) - sguan->current.Pos_offset0)*
                            sguan->current.Final_Gain*sguan->motor.Current_Dir0;
    sguan->bpf.Current1.filter.Input = (User_ReadADC_Raw(1) - sguan->current.Pos_offset1)*
                            sguan->current.Final_Gain*sguan->motor.Current_Dir1;
    BPF(&sguan->bpf.Current0);
    BPF(&sguan->bpf.Current1);
    float I0 = sguan->bpf.Current0.filter.Output;
    float I1 = sguan->bpf.Current1.filter.Output;
    float I2 = -(I0 + I1);
    if (sguan->motor.Current_Num == 0){  // AB采样(判断电流相序和电机方向)
        sguan->current.Real_Ia = sguan->motor.Motor_Dir == 1 ? I0 : I1;
        sguan->current.Real_Ib = sguan->motor.Motor_Dir == 1 ? I1 : I0;
        sguan->current.Real_Ic = I2;
    } else if (sguan->motor.Current_Num == 1){  // AC采样(判断电流相序和电机方向)
        sguan->current.Real_Ia = sguan->motor.Motor_Dir == 1 ? I0 : I2;
        sguan->current.Real_Ib = sguan->motor.Motor_Dir == 1 ? I2 : I0;
        sguan->current.Real_Ic = I1;
    } else {  // BC采样(判断电流相序和电机方向)
        sguan->current.Real_Ia = sguan->motor.Motor_Dir == 1 ? I2 : I0;
        sguan->current.Real_Ib = sguan->motor.Motor_Dir == 1 ? I0 : I2;
        sguan->current.Real_Ic = I1;
    }
}

// Current更新alpha,beta和DQ轴电流(已滤波)
static void Current_Tick(SguanFOC_System_STRUCT *sguan){
    Current_ReadIabc(sguan);
    clarke(&sguan->current.Real_Ialpha,
        &sguan->current.Real_Ibeta,
        sguan->current.Real_Ia,
        sguan->current.Real_Ib);
    park(&sguan->current.Real_Id,
        &sguan->current.Real_Iq,
        sguan->current.Real_Ialpha,
        sguan->current.Real_Ibeta,
        sguan->foc.sine,
        sguan->foc.cosine);
}

// PID速度开环(用于直接控制Uq_in,用于电机测试)
static void PID_Velocity_OPEN(SguanFOC_System_STRUCT *sguan){
    sguan->foc.Ud_in = 0.0f;
    sguan->foc.Uq_in = sguan->foc.Uq_in;
}

// PID电流单环(单闭环)
static void PID_Current_SINGLE(SguanFOC_System_STRUCT *sguan){
    sguan->pid.Current_D.run.Ref = sguan->foc.Target_Id; // 默认D轴励磁Id为0
    sguan->pid.Current_D.run.Fbk = sguan->current.Real_Id;
    sguan->pid.Current_Q.run.Ref = sguan->foc.Target_Iq;
    sguan->pid.Current_Q.run.Fbk = sguan->current.Real_Iq;
    PID_Loop(&sguan->pid.Current_D);
    PID_Loop(&sguan->pid.Current_Q);
    sguan->foc.Ud_in = sguan->pid.Current_D.run.Output;
    sguan->foc.Uq_in = sguan->pid.Current_Q.run.Output;
}

// PID速度单环(单闭环)
static void PID_Velocity_SINGLE(SguanFOC_System_STRUCT *sguan){
    sguan->pid.Velocity.run.Ref = sguan->foc.Target_Speed;
    sguan->pid.Velocity.run.Fbk = sguan->encoder.Real_Speed;
    PID_Loop(&sguan->pid.Velocity);
    sguan->foc.Ud_in = 0.0f;
    sguan->foc.Uq_in = sguan->pid.Velocity.run.Output;
}

// PID位置单环(单闭环)
static void PID_Position_SINGLE(SguanFOC_System_STRUCT *sguan){
    sguan->pid.Position.run.Ref = sguan->foc.Target_Pos;
    sguan->pid.Position.run.Fbk = sguan->encoder.Real_Pos;
    PID_Loop(&sguan->pid.Position);
    sguan->foc.Ud_in = 0.0f;
    sguan->foc.Uq_in = sguan->pid.Position.run.Output;
}

// PID速度-电流双环(双闭环)
static void PID_VelCur_DOUBLE(SguanFOC_System_STRUCT *sguan){
    sguan->pid.Count++;
    if (sguan->pid.Count == sguan->pid.Response){
        sguan->pid.VelCur_v.run.Ref = sguan->foc.Target_Speed;
        sguan->pid.VelCur_v.run.Fbk = sguan->encoder.Real_Speed;
        PID_Loop(&sguan->pid.VelCur_v);
        sguan->pid.Count = 0;
    }
    sguan->pid.VelCur_D.run.Ref = sguan->foc.Target_Id; // 默认D轴励磁Id为0
    sguan->pid.VelCur_D.run.Fbk = sguan->current.Real_Id;
    sguan->pid.VelCur_Q.run.Ref = sguan->pid.VelCur_v.run.Output;
    sguan->pid.VelCur_Q.run.Fbk = sguan->current.Real_Iq;
    PID_Control(&sguan->pid.VelCur_D);
    PID_Control(&sguan->pid.VelCur_Q);
    sguan->foc.Ud_in = sguan->pid.VelCur_D.run.Output;
    sguan->foc.Uq_in = sguan->pid.VelCur_Q.run.Output;
}

// PID位置-速度双环(双闭环)
static void PID_PosVel_DOUBLE(SguanFOC_System_STRUCT *sguan){
    sguan->pid.Count++;
    if (sguan->pid.Count == sguan->pid.Response){
        sguan->pid.PosVel_p.run.Ref = sguan->foc.Target_Pos;
        sguan->pid.PosVel_p.run.Fbk = sguan->encoder.Real_Pos;
        PID_Loop(&sguan->pid.PosVel_p);
        sguan->pid.Count = 0;
    }
    sguan->pid.PosVel_v.run.Ref = sguan->pid.PosVel_p.run.Output;
    sguan->pid.PosVel_v.run.Fbk = sguan->encoder.Real_Speed;
    PID_Control(&sguan->pid.PosVel_v);
    sguan->foc.Ud_in = 0.0f;
    sguan->foc.Uq_in = sguan->pid.PosVel_v.run.Output;
}

// PID高性能伺服三环(三闭环)
static void PID_PosVelCur_THREE(SguanFOC_System_STRUCT *sguan){
    sguan->pid.Count++;
    if (sguan->pid.Count == sguan->pid.Compare){
        sguan->pid.PosVelCur_p.run.Ref = sguan->foc.Target_Pos;
        sguan->pid.PosVelCur_p.run.Fbk = sguan->encoder.Real_Pos;
        PID_Loop(&sguan->pid.PosVelCur_p);
        sguan->pid.Count = 0;
    }
    if (sguan->pid.Count % sguan->pid.Response == 0){
        sguan->pid.PosVelCur_v.run.Ref = sguan->pid.PosVelCur_p.run.Output;
        sguan->pid.PosVelCur_v.run.Fbk = sguan->encoder.Real_Speed;
        PID_Loop(&sguan->pid.PosVelCur_v);
    }
    sguan->pid.PosVelCur_D.run.Ref = sguan->foc.Target_Id; // 默认D轴励磁Id为0
    sguan->pid.PosVelCur_D.run.Fbk = sguan->current.Real_Id;
    sguan->pid.PosVelCur_Q.run.Ref = sguan->foc.Target_Iq;
    sguan->pid.PosVelCur_Q.run.Fbk = sguan->current.Real_Iq;
    PID_Loop(&sguan->pid.PosVelCur_D);
    PID_Loop(&sguan->pid.PosVelCur_Q);
    sguan->foc.Ud_in = sguan->pid.PosVelCur_D.run.Output;
    sguan->foc.Uq_in = sguan->pid.PosVelCur_Q.run.Output;
}

// Data母线电压和驱动器物理温度数据更新和滤波
static void Data_Protection_Loop(SguanFOC_System_STRUCT *sguan){
    sguan->bpf.VBUS.filter.Input = User_VBUS_DataGet();
    sguan->bpf.Thermistor.filter.Input = User_Temperature_DataGet();
    BPF_Loop(&sguan->bpf.VBUS);
    BPF_Loop(&sguan->bpf.Thermistor);
}

// SVPWM电机驱动的马鞍波生成
static void SVPWM_Tick(SguanFOC_System_STRUCT *sguan,float Erad,float d_set,float q_set){
    SVPWM(Erad,d_set,q_set,
        &sguan->foc.Du,
        &sguan->foc.Dv,
        &sguan->foc.Dw);
    if (sguan->motor.PWM_Dir == 1){
        sguan->foc.Duty_u = sguan->foc.Du*sguan->motor.Duty;
        sguan->foc.Duty_v = sguan->foc.Dv*sguan->motor.Duty;
        sguan->foc.Duty_w = sguan->foc.Dw*sguan->motor.Duty;
    }
    else if (sguan->motor.PWM_Dir == -1){
        sguan->foc.Duty_u = (1.0f - sguan->foc.Du)*sguan->motor.Duty;
        sguan->foc.Duty_v = (1.0f - sguan->foc.Dv)*sguan->motor.Duty;
        sguan->foc.Duty_w = (1.0f - sguan->foc.Dw)*sguan->motor.Duty;
    }
    if (sguan->motor.Motor_Dir == -1){ // 判断电机方向并修改(原理是AB相序交换)
        float duty_temp = sguan->foc.Duty_u;
        sguan->foc.Duty_u = sguan->foc.Duty_v;
        sguan->foc.Duty_v = duty_temp;
    }
    User_PwmDuty_Set(sguan->foc.Duty_u,sguan->foc.Duty_v,sguan->foc.Duty_w);
}

// Sguan...Loop计算角速度/角度和电流值
static void Sguan_Calculate_Loop(SguanFOC_System_STRUCT *sguan){
    Current_Tick(sguan);
    Encoder_Tick(sguan);
}

// Sguan...Loop计算PID并执行电机控制
static void Sguan_GeneratePWM_Loop(SguanFOC_System_STRUCT *sguan){
    if (sguan->status == MOTOR_STATUS_ENABLED){
        // 用户实时控制的参数传入
        User_UserControl();
        // PID运算PWM大小并执行
        PID_Tick[sguan->mode](sguan);
        SVPWM_Tick(sguan,sguan->encoder.Real_Erad,
            sguan->foc.Ud_in/sguan->motor.Vbus,
            sguan->foc.Uq_in/sguan->motor.Vbus);
    }
}

// Sguan...Init电机参数辨识和PI参数自适应初始化
static void Sguan_Quantize_Init(void){
    #if Quantize_Method 1
    uint8_t count = 0;
    #endif
}

// Sguan...Init巴特沃斯低通滤波器的初始化
static void Sguan_BPF_Init(SguanFOC_System_STRUCT *sguan){
    BPF_Init(&sguan->bpf.VBUS);
    BPF_Init(&sguan->bpf.Current0);
    BPF_Init(&sguan->bpf.Current1);
    BPF_Init(&sguan->bpf.Encoder);
    BPF_Init(&sguan->bpf.Thermistor);
    BPF_Init(&sguan->bpf.HFI);
    BPF_Init(&sguan->bpf.SMO);
}

// Sguan...Init闭环控制算法PID的初始化
static void Sguan_PID_Init(SguanFOC_System_STRUCT *sguan){
    // 单闭环初始化(电流，速度，位置)
    PID_Init(&sguan->pid.Current_D);
    PID_Init(&sguan->pid.Current_Q);
    PID_Init(&sguan->pid.Velocity);
    PID_Init(&sguan->pid.Position);
    // 双闭环(速度-电流，位置-速度)
    PID_Init(&sguan->pid.VelCur_v);
    PID_Init(&sguan->pid.VelCur_D);
    PID_Init(&sguan->pid.VelCur_Q);
    PID_Init(&sguan->pid.PosVel_p);
    PID_Init(&sguan->pid.PosVel_v);
    // 三闭环(位置-速度-电流)
    PID_Init(&sguan->pid.PosVelCur_p);
    PID_Init(&sguan->pid.PosVelCur_v);
    PID_Init(&sguan->pid.PosVelCur_D);
    PID_Init(&sguan->pid.PosVelCur_Q);
}

// Sguan...Init锁相环PLL的初始化
static void Sguan_PLL_Init(SguanFOC_System_STRUCT *sguan){
    PLL_Init(&sguan->pll.HFI);
    PLL_Init(&sguan->pll.SMO);
}

// Sguan...Init无感算法Sensorless的初始化
static void Sguan_Sensorless_Init(SguanFOC_System_STRUCT *sguan){
    if (sguan->control == Sensorless_HFI_Control){
        HFI_Init(&sguan->hfi);
    }
    else if (sguan->control == Sensorless_SMO_Control){
        SMO_Init(&sguan->smo);
    }
    else if (sguan->control == SensorlessFOC_Control){
        HFI_Init(&sguan->hfi);
        SMO_Init(&sguan->smo);
    }
}

// Pre_Pos电机零点对齐
static void Sguan_Pre_Positioning(SguanFOC_System_STRUCT *sguan,float d_set){
    SVPWM_Tick(sguan,0.0f,d_set,0.0f);
}

// 系统时钟设置(定时器中断周期)
static void Sguan_SystemT_Set(SguanFOC_System_STRUCT *sguan){
    sguan->pid.Compare = sguan->pid.Response*sguan->pid.Response;
    // 0.电机编码器参数
    sguan->motor.Encoder_T = sguan->System_T;
    // 1.bpf低通滤波器
    sguan->bpf.VBUS.T = sguan->TIM_ms_T;
    sguan->bpf.Current0.T = sguan->System_T;
    sguan->bpf.Current1.T = sguan->System_T;
    sguan->bpf.Thermistor.T = sguan->TIM_ms_T;
    sguan->bpf.HFI.T = sguan->System_T;
    sguan->bpf.SMO.T = sguan->System_T;
    // 2.pid闭环控制系统
    sguan->pid.Current_D.T = sguan->System_T;
    sguan->pid.Current_Q.T = sguan->System_T;
    sguan->pid.Velocity.T = sguan->System_T;
    sguan->pid.Position.T = sguan->System_T;

    sguan->pid.VelCur_v.T = sguan->System_T*sguan->pid.Response;
    sguan->pid.VelCur_D.T = sguan->System_T;
    sguan->pid.VelCur_Q.T = sguan->System_T;
    sguan->pid.PosVel_p.T = sguan->System_T*sguan->pid.Response;
    sguan->pid.PosVel_v.T = sguan->System_T;

    sguan->pid.PosVelCur_p.T = sguan->System_T*sguan->pid.Compare;
    sguan->pid.PosVelCur_v.T = sguan->System_T*sguan->pid.Response;
    sguan->pid.PosVelCur_D.T = sguan->System_T;
    sguan->pid.PosVelCur_Q.T = sguan->System_T;
    // 3.pll锁相环跟踪系统
    sguan->pll.HFI.T = sguan->System_T;
    sguan->pll.SMO.T = sguan->System_T;
}

/**
 * @description: SguanFOC核心文件，主任务初始化函数
 * @reminder: 主循环之前，任务优先级“高”
 * @return {*}
 */
void SguanFOC_Init(void){
    Sguan.status = MOTOR_STATUS_INITIALIZING;
    // 用户自定义的电机参数和控制系统参数
    User_MotorSet();
    User_ParameterSet();
    // 系统时间设定
    Sguan_SystemT_Set(&Sguan);
    // 用户自定义的Init(主要用于物理驱动器)
    User_InitialInit();
    // 控制系统量化和电机参数辨识(离线测量)
    Sguan_Quantize_Init();
    // 各种控制系统的初始化
    Sguan_BPF_Init(&Sguan);
    Sguan_PID_Init(&Sguan);
    Sguan_PLL_Init(&Sguan);
    Sguan_Sensorless_Init(&Sguan);
    // 读取电流偏置
    Sguan.status = MOTOR_STATUS_CALIBRATING;
    Offset_CurrentRead(&Sguan);
    // 电机回零操作
    Sguan_Pre_Positioning(&Sguan,Sguan.motor.Limit);
    User_Delay(800);
    // 读取角度偏置
    Offset_EncoderRead(&Sguan);
    // 电机失能并进入正常工作状态
    Sguan_Pre_Positioning(&Sguan,0.0f);
    User_Delay(600);
    // 正常工作中(状态机运行)
    Sguan.status = MOTOR_STATUS_ENABLED;
}

/**
 * @description: SguanFOC核心文件，定时中断服务函数(高频率电机载波)
 * @reminder: 20Khz或者更高定时中断中调用，任务优先级“最高”
 * @return {*}
 */
void SguanFOC_Loop(void){
    Sguan_Calculate_Loop(&Sguan);
    Sguan_GeneratePWM_Loop(&Sguan);
}

/**
 * @description: SguanFOC核心文件，定时中断服务函数(1ms周期数据更新)
 * @reminder: 1Khz或者更低定时中断中调用，任务优先级“低”
 * @return {*}
 */
void SguanFOC_msTick(void){
    Data_Protection_Loop(&Sguan);
    MotorStatus_Loop(&Sguan.status);
}

/**
 * @description: SguanFOC核心文件，主循环服务函数(主循环TXdata数据更新)
 * @reminder: 1Khz或者更低定时中断中调用，任务优先级“最低”
 * @return {*}
 */
void SguanFOC_mainTick(void){
    User_UserTX();
    Printf_Loop(&Sguan.TXdata);
}

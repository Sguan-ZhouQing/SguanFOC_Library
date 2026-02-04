/*
 * @Author: 星必尘Sguan
 * @GitHub: https://github.com/Sguan-ZhouQing
 * @Date: 2026-01-26 22:38:34
 * @LastEditors: 星必尘Sguan|3464647102@qq.com
 * @LastEditTime: 2026-02-04 23:34:02
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
 * @description: 3.Sensorless无感控制读取转子位置
 * @param {SguanFOC_System_STRUCT} *sguan
 * @return {*}
 */
static float HFI_Encoder_ReadRad(SguanFOC_System_STRUCT *sguan);
static float SMO_Encoder_ReadRad(SguanFOC_System_STRUCT *sguan);
/**
 * @description: 4.Current内部静态函数声明
 * @param {SguanFOC_System_STRUCT} *sguan
 * @return {*}
 */
static void Current_ReadIabc(SguanFOC_System_STRUCT *sguan);
static void Current_Tick(SguanFOC_System_STRUCT *sguan);
/**
 * @description: 5.PID运算及其模式切换
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
 * @description: 6.Data母线电压和驱动器物理温度数据更新和滤波
 * @param {SguanFOC_System_STRUCT} *sguan
 * @return {*}
 */
static void Data_Protection_Loop(SguanFOC_System_STRUCT *sguan);
/**
 * @description: 7.Status判断并切换状态机
 * @param {SguanFOC_System_STRUCT} *sguan
 * @return {*}
 */
static void Status_Switch_Loop(SguanFOC_System_STRUCT *sguan);
/**
 * @description: 8.Printf数据发送Debug和正常模式
 * @param {SguanFOC_System_STRUCT} *sguan
 * @return {*}
 */
static void Printf_Debug_Loop(SguanFOC_System_STRUCT *sguan);
static void Printf_Normal_Loop(SguanFOC_System_STRUCT *sguan);
/**
 * @description: 9.SVPWM电机驱动的马鞍波生成
 * @param {SguanFOC_System_STRUCT} *sguan
 * @param {float} d
 * @param {float} q
 * @return {*}
 */
static void SVPWM_Tick(SguanFOC_System_STRUCT *sguan,float Erad,float d_set,float q_set);
/**
 * @description: 10.Sguan...Loop定时计算并执行
 * @param {SguanFOC_System_STRUCT} *sguan
 * @return {*}
 */
static void Sguan_Calculate_Loop(SguanFOC_System_STRUCT *sguan);
static void Sguan_GeneratePWM_Loop(SguanFOC_System_STRUCT *sguan);
/**
 * @description: 11.Sguan...Init各种控制系统的初始化
 * @param {SguanFOC_System_STRUCT} *sguan
 * @return {*}
 */
static void Sguan_Quantize_Init(void);
static void Sguan_PIDset_Init(void);
static void Sguan_BPF_Init(SguanFOC_System_STRUCT *sguan);
static void Sguan_PID_Init(SguanFOC_System_STRUCT *sguan);
static void Sguan_PLL_Init(SguanFOC_System_STRUCT *sguan);
static void Sguan_Sensorless_Init(SguanFOC_System_STRUCT *sguan);
/**
 * @description: 12.Pre电机预处理
 * @param {SguanFOC_System_STRUCT} *sguan
 * @return {*}
 */
static void Sguan_Pre_Positioning(SguanFOC_System_STRUCT *sguan,float d_set);
static void Sguan_SystemT_Set(SguanFOC_System_STRUCT *sguan);
static void Sguan_ReStart_Loop(void);
/**
 * @description: 13.SguanFOC核心文件，主任务初始化函数
 * @return {*}
 */
static void SguanFOC_Init(void);


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
    // 1.有感FOC控制
    if (sguan->control == SensorFOC_Control){
        sguan->encoder.Real_Rad = User_Encoder_ReadRad();
    }
    // 2.无感控制HFI
    else if (sguan->control == Sensorless_HFI_Control){
        sguan->encoder.Real_Rad = HFI_Encoder_ReadRad(sguan);
    }
    // 3.无感控制SMO
    else if (sguan->control == Sensorless_SMO_Control){
        sguan->encoder.Real_Rad = SMO_Encoder_ReadRad(sguan);
    }
    // 4.无感全速域HFI-SMO
    else if (sguan->control == SensorlessFOC_Control){
        if (1){
            sguan->encoder.Real_Rad = HFI_Encoder_ReadRad(sguan);
        }
        else{
            sguan->encoder.Real_Rad = SMO_Encoder_ReadRad(sguan);
        }
    }
    sguan->bpf.Encoder.filter.Input = Encoder_ReadSpeed(sguan)*sguan->motor.Motor_Dir;
    BPF_Loop(&sguan->bpf.Encoder);
    sguan->encoder.Real_Speed = sguan->bpf.Encoder.filter.Output;
    sguan->encoder.Real_Pos = Encoder_ReadPos(sguan)*sguan->motor.Motor_Dir;
    sguan->encoder.Real_Erad = Encoder_ReadErad(sguan);
}

// Sensorless高频注入无感控制
static float HFI_Encoder_ReadRad(SguanFOC_System_STRUCT *sguan){

}

// Sensorless滑膜观测器无感控制
static float SMO_Encoder_ReadRad(SguanFOC_System_STRUCT *sguan){

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
    sguan->foc.Real_VBUS = sguan->bpf.VBUS.filter.Output;
    sguan->foc.Real_Temp = sguan->bpf.Thermistor.filter.Output;
}

// Status判断并切换状态机
static void Status_Switch_Loop(SguanFOC_System_STRUCT *sguan){
    // 1.电机母线电压VBUS状态机
    if (sguan->foc.Real_VBUS > sguan->motor.VBUS_MAX){
        sguan->status = MOTOR_STATUS_OVERVOLTAGE;
    }
    else if (sguan->foc.Real_VBUS < sguan->motor.VBUS_MIM){
        sguan->status = MOTOR_STATUS_UNDERVOLTAGE;
    }
    // 2.驱动器物理温度Temp状态机
    if (sguan->foc.Real_Temp > sguan->motor.Temp_MAX){
        sguan->status = MOTOR_STATUS_OVERTEMPERATURE;
    }
    else if (sguan->foc.Real_Temp < sguan->motor.Temp_MIN){
        sguan->status = MOTOR_STATUS_UNDERTEMPERATURE;
    }
    // 3.过流保护
    if (sguan->current.Real_Iq > sguan->motor.Qcur_MAX){
        sguan->status = MOTOR_STATUS_OVERCURRENT;
    }
    // 4.急停保护
    if (User_StatusSTOP_Signal()){
        sguan->status = MOTOR_STATUS_EMERGENCY_STOP;
    }
    // 5.关机失能
    
}

// Printf电机调试信息发送
static void Printf_Debug_Loop(SguanFOC_System_STRUCT *sguan){
    static uint8_t status = 0xFF;
    static uint32_t count = 0;
    if (sguan->status != status){
        static const char* status_names[] = {
            // ====== 初始化与运行状态(状态) ======
            "待机（未初始化，准备中）",
            "未初始化",
            "初始化中（参数加载、外设初始化）",
            "校准中（电阻、电感、编码器零位）",
            // ====== 运行状态(当前反馈) ======
            "空闲（已初始化，使能但零指令）",
            "力矩增大中~电流模式(下时刻->力矩保持)",
            "力矩减小中~电流模式(下时刻->力矩保持)",
            "力矩保持~电流模式(稳态)",
            "加速中~速度模式(下时刻->恒速保持)",
            "减速中~速度模式(下时刻->恒速保持)",
            "恒速保持~速度模式(稳态)",
            "位置增加中~位置模式(下时刻->位置保持)",
            "位置减少中~位置模式(下时刻->位置保持)",
            "位置保持~位置模式(稳态)",
            // ====== 硬件相关错误(状态) ======
            "过压保护(锁定->手动解除进待机)",
            "欠压保护(锁定->手动解除进待机)",
            "过温保护(锁定->手动解除进待机)",
            "低温保护(锁定->手动解除进待机)",
            "过流保护(稳态->电机电流限幅)",
            "编码器故障(锁定->手动解除进待机)",
            "传感器故障(锁定->手动解除进待机)",
            "缺相错误(锁定->手动解除进待机)",
            "相线短路错误(锁定->手动解除进待机)",
            // ====== 安全状态(状态) ======
            "急停（立即关闭PWM,会立即锁定->手动解除进待机）",
            "已失能（软关闭,会缓慢进入待机->自动进待机）"
        };
        
        printf("[状态机更新%lu]：%s...状态机编号%d\n", 
                   count, 
                   status_names[sguan->status], 
                   sguan->status);
        count += 1;
        status = sguan->status;
    }
}

// Printf电机数据正常发送
static void Printf_Normal_Loop(SguanFOC_System_STRUCT *sguan){
    // 用户数据填写(串口或者CAN通信)
    User_UserTX();
    // 发送数据to上位机
    Printf_Loop(&Sguan.TXdata);
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
    if (sguan->status == MOTOR_STATUS_IDLE){
        // 用户实时控制的参数传入
        User_UserControl();
        // PID运算PWM大小并执行
        if (sguan->mode >= 0 && sguan->mode < 7){
            PID_Tick[sguan->mode](sguan);
        } else{
            // 错误处理：自动跳转到默认速度单闭环模式
            sguan->mode = Velocity_SINGLE_MODE;
            PID_Tick[Velocity_SINGLE_MODE](sguan);
        }
        SVPWM_Tick(sguan,sguan->encoder.Real_Erad,
            sguan->foc.Ud_in/sguan->motor.Vbus,
            sguan->foc.Uq_in/sguan->motor.Vbus);
    }
}

// Sguan...Init电机参数辨识和自适应初始化
static void Sguan_Quantize_Init(void){
    #if Quantize_Method
    uint8_t count = 0;
    #endif // Quantize_Method
}

// Sguan...Init闭环控制系统的参数自适应
static void Sguan_PIDset_Init(void){
    #if PID_Calculate
    uint8_t temp = 0;
    #endif // PID_Calculate
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

// Pre电机零点对齐(机械角度对齐)
static void Sguan_Pre_Positioning(SguanFOC_System_STRUCT *sguan,float d_set){
    SVPWM_Tick(sguan,0.0f,d_set,0.0f);
}

// Pre系统时钟设置(定时器中断周期)
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

// Pre系统开始的核心文件,重新开始初始化
static void Sguan_ReStart_Loop(void){

}

// SguanFOC核心文件，主任务初始化函数
static void SguanFOC_Init(void){
    if (Sguan.status == MOTOR_STATUS_UNINITIALIZED){
        static uint8_t Flag = 0;
        if (!Flag){
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
            Sguan_PIDset_Init();
            // 各种控制系统的初始化
            Sguan_BPF_Init(&Sguan);
            Sguan_PID_Init(&Sguan);
            Sguan_PLL_Init(&Sguan);
            Sguan_Sensorless_Init(&Sguan);
            Printf_Init(&Sguan);
            Flag = 1;
        }
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
        Sguan.status = MOTOR_STATUS_IDLE;
    }
}

/**
 * @description: SguanFOC核心文件，定时中断服务函数(高频率电机载波)
 * @reminder: 10Khz或者更高定时中断中调用，任务优先级“最高”
 * @return {*}
 */
void SguanFOC_Loop(void){
    // 计算编码器和电流
    Sguan_Calculate_Loop(&Sguan);
    // 运算PID的执行SVPWM
    Sguan_GeneratePWM_Loop(&Sguan);
}

/**
 * @description: SguanFOC核心文件，定时中断服务函数(1ms周期数据更新)
 * @reminder: 1Khz或者更低定时中断中调用，任务优先级“低”
 * @return {*}
 */
void SguanFOC_msTick(void){
    // 读取母线电压VBUS和温度数据Temp
    Data_Protection_Loop(&Sguan);
    // 作用->根据环境切换电机状态机
    Status_Switch_Loop(&Sguan);
    // 作用->根据电机状态机,运行不同任务
    MotorStatus_Loop(&Sguan.status);
}

/**
 * @description: SguanFOC核心文件，主循环服务函数(主循环TXdata数据更新)
 * @reminder: 主循环函数调用，任务优先级“最低”
 * @return {*}
 */
void SguanFOC_mainTick(void){
    SguanFOC_Init();
    #if Printf_Debug
    Printf_Debug_Loop(&Sguan);
    #else // Printf_Debug
    Printf_Normal_Loop(&Sguan);
    #endif // Printf_Debug
}

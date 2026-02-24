/*
 * @Author: 星必尘Sguan
 * @GitHub: https://github.com/Sguan-ZhouQing
 * @Date: 2026-01-26 22:38:34
 * @LastEditors: 星必尘Sguan|3464647102@qq.com
 * @LastEditTime: 2026-02-21 01:27:00
 * @FilePath: \stm_SguanFOCtest\SguanFOC\SguanFOC.c
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
 * @description: 1.Transfer传递函数的离散化运算，采用双线性变换
 * @param {PID_STRUCT} *pid (控制)PID闭环控制系统运算
 * @param {LADRC_STRUCT} *ladrc (控制)LADRC线自抗扰运算
 * @param {INTERNALMODEL_STRUCT} *im (控制)IMC内模控制
 * @param {BPF_STRUCT} *bpf (滤波)BPF二阶巴特沃斯低通滤波
 * @param {PLL_STRUCT} *pll (估算)PLL速度跟踪锁相环
 * @return {*}
 */
static void Transfer_PID_Loop(PID_STRUCT *pid,float Ref,float Fbk);
static void Transfer_Ladrc_Loop(LADRC_STRUCT *ladrc,float Ref,float Fbk);
static void Transfer_IM_Loop(INTERNALMODEL_STRUCT *im,PID_STRUCT *pid,float Ref,float Fbk);
static void Transfer_BPF_Loop(BPF_STRUCT *bpf,float input);
static void Transfer_PLL_Loop(PLL_STRUCT *pll,uint8_t mode,float input_Rad);
/**
 * @description: 2.Sensorless无感控制读取转子位置
 * @param {SguanFOC_System_STRUCT} *sguan
 * @return {*}
 */
static float HFI_Encoder_ReadRad(SguanFOC_System_STRUCT *sguan);
static float SMO_Encoder_ReadRad(SguanFOC_System_STRUCT *sguan);
/**
 * @description: 3.Offset内部静态函数声明
 * @param {SguanFOC_System_STRUCT} *sguan
 * @return {*}
 */
static void Offset_EncoderRead(SguanFOC_System_STRUCT *sguan);
static void Offset_CurrentRead(SguanFOC_System_STRUCT *sguan);
/**
 * @description: 4.Encoder内部静态函数声明
 * @param {SguanFOC_System_STRUCT} *sguan
 * @return {*}
 */
static float Encoder_ReadErad(SguanFOC_System_STRUCT *sguan);
static float Encoder_ReadSpeed(SguanFOC_System_STRUCT *sguan);
static void Encoder_Tick(SguanFOC_System_STRUCT *sguan);
/**
 * @description: 5.Current内部静态函数声明
 * @param {SguanFOC_System_STRUCT} *sguan
 * @return {*}
 */
static void Current_ReadIabc(SguanFOC_System_STRUCT *sguan);
static void Current_Tick(SguanFOC_System_STRUCT *sguan);
/**
 * @description: 6.Control运算及其模式切换
 * @param {SguanFOC_System_STRUCT} *sguan
 * @return {*}
 */
static void Control_Velocity_OPEN(SguanFOC_System_STRUCT *sguan);
static void Control_Current_SINGLE(SguanFOC_System_STRUCT *sguan);
static void Control_VelCur_DOUBLE(SguanFOC_System_STRUCT *sguan);
static void Control_PosVelCur_THREE(SguanFOC_System_STRUCT *sguan);
static void Control_Sensorless_HFI(SguanFOC_System_STRUCT *sguan);
static void Control_Sensorless_SMO(SguanFOC_System_STRUCT *sguan);
static void Control_Sensorless_HS(SguanFOC_System_STRUCT *sguan);
static void (*const Control_Tick[])(SguanFOC_System_STRUCT*)={
    /*这里注意“枚举变量”的边界, PID_Tick[sguan->mode](sguan)使用*/
    Control_Velocity_OPEN,      // Velocity_OPEN_MODE  = 0
    Control_Current_SINGLE,     // Current_SINGLE_MODE = 1
    Control_VelCur_DOUBLE,      // VelCur_DOUBLE_MODE  = 2
    Control_PosVelCur_THREE,    // PosVelCur_THREE_MODE= 3
    Control_Sensorless_HFI,     // Sensorless_HFI_MODE = 4
    Control_Sensorless_SMO,     // Sensorless_SMO_MODE = 5
    Control_Sensorless_HS       // Sensorless_HS_MODE  = 6
};
/**
 * @description: 7.Data母线电压和驱动器物理温度数据更新
 * @param {SguanFOC_System_STRUCT} *sguan
 * @return {*}
 */
static void Data_Protection_Loop(SguanFOC_System_STRUCT *sguan);
/**
 * @description: 8.Status判断并切换状态机
 * @param {SguanFOC_System_STRUCT} *sguan
 * @return {*}
 */
static void Status_Switch_Loop(SguanFOC_System_STRUCT *sguan);
/**
 * @description: 9.Printf数据发送Debug和正常模式
 * @param {SguanFOC_System_STRUCT} *sguan
 * @return {*}
 */
#if Printf_Debug
static void Printf_Debug_Loop(SguanFOC_System_STRUCT *sguan);
#else // Printf_Debug
static void Printf_Normal_Loop(SguanFOC_System_STRUCT *sguan);
#endif // Printf_Debug
/**
 * @description: 10.SVPWM电机驱动的马鞍波生成
 * @param {SguanFOC_System_STRUCT} *sguan
 * @param {float} d
 * @param {float} q
 * @return {*}
 */
static void SVPWM_Tick(SguanFOC_System_STRUCT *sguan,float Erad,float d_set,float q_set);
/**
 * @description: 11.Sguan...Loop定时计算并执行
 * @param {SguanFOC_System_STRUCT} *sguan
 * @return {*}
 */
static void Sguan_Calculate_Loop(SguanFOC_System_STRUCT *sguan);
static void Sguan_GeneratePWM_Loop(SguanFOC_System_STRUCT *sguan);
/**
 * @description: 12.Sguan...Init各种控制系统的初始化
 * @param {SguanFOC_System_STRUCT} *sguan
 * @return {*}
 */
static void Sguan_Quantize_Init(void);
static void Sguan_BPF_Init(SguanFOC_System_STRUCT *sguan);
static void Sguan_PID_Init(SguanFOC_System_STRUCT *sguan);
static void Sguan_PLL_Init(SguanFOC_System_STRUCT *sguan);
static void Sguan_Sensorless_Init(SguanFOC_System_STRUCT *sguan);
/**
 * @description: 13.Pre电机预处理
 * @param {SguanFOC_System_STRUCT} *sguan
 * @return {*}
 */
static void Sguan_Pre_Positioning(SguanFOC_System_STRUCT *sguan);
static void Sguan_SystemT_Set(SguanFOC_System_STRUCT *sguan);
static void Sguan_Start_Loop(void);


// Transfer运算_PID运算
static void Transfer_PID_Loop(PID_STRUCT *pid,float Ref,float Fbk){
    pid->run.Ref = Ref;
    pid->run.Fbk = Fbk;
    PID_Loop(pid);
    // 输出pid->run.Output;
}

// Transfer运算_LADRC运算
static void Transfer_Ladrc_Loop(LADRC_STRUCT *ladrc,float Ref,float Fbk){
    ladrc->linear.Ref = Ref;
    ladrc->linear.Fbk = Fbk;
    Ladrc_Loop(ladrc);
    // 输出ladrc->linear.Output;
}

// Transfer运算_内模控制运算
static void Transfer_IM_Loop(INTERNALMODEL_STRUCT *im,PID_STRUCT *pid,float Ref,float Fbk){
    // 1.设置参考值并计算反馈误差(包含模型补偿)
    pid->run.Ref = Ref;
    pid->run.Fbk = Fbk - im->imc.Output;
    // 2.执行PI运算
    PID_Loop(pid);
    // 3.IMC数据输入，运行内模控制
    im->imc.Input = pid->run.Output;
    InternalModel_Loop(im);
    // 输出pid->run.Output;
}

// Transfer运算_二阶巴特沃斯低通滤波器
static void Transfer_BPF_Loop(BPF_STRUCT *bpf,float input){
    bpf->filter.Input = input;
    BPF_Loop(bpf);
    // 输出bpf->filter.Output;
}

// Transfer运算_速度锁相环
static void Transfer_PLL_Loop(PLL_STRUCT *pll,uint8_t mode,float input_Rad){
    float Rad_Error = input_Rad - normalize_angle(pll->go.OutRe);
    // 计算角度误差,始终归一化到[-π, π)范围
    if (Rad_Error >= Value_PI){
        Rad_Error -= Value_PI*2;
    }
    if (Rad_Error <= -Value_PI){
        Rad_Error += Value_PI*2;
    }
    
    if (mode == PosVelCur_THREE_MODE){
        // 位置环模式：PLL连续积分（可以超过2π）
        if (!pll->is_position_mode){
            pll->is_position_mode = 1;
        }
    } else{
        // 非位置环模式：PLL输出归一化到[0, 2π)
        if (pll->is_position_mode){
            pll->is_position_mode = 0;
        }
    }

    pll->go.Error = Rad_Error;
    PLL_Loop(pll);
    // 输出pll->go.OutWe;
    // 输出pll->go.OutRe;
}

// Sensorless高频注入无感控制
#if MOTOR_CONTROL == 1 || MOTOR_CONTROL == 3
static float HFI_Encoder_ReadRad(SguanFOC_System_STRUCT *sguan){
    float Erad = 0.0f;
    return Erad;
}
#endif // MOTOR_CONTROL

// Sensorless滑膜观测器无感控制
#if MOTOR_CONTROL == 2 || MOTOR_CONTROL == 3
static float SMO_Encoder_ReadRad(SguanFOC_System_STRUCT *sguan){
    float Erad = 0.0f;
    return Erad;
}
#endif // MOTOR_CONTROL

// Offset读取编码器偏置
static void Offset_EncoderRead(SguanFOC_System_STRUCT *sguan){
    sguan->encoder.Pos_offset = User_Encoder_ReadRad();
}

// Offset读取电流偏置
static void Offset_CurrentRead(SguanFOC_System_STRUCT *sguan){
    for (uint8_t i = 0; i < 24; i++){
        sguan->current.Pos_offset0 += User_ReadADC_Raw(0);
        sguan->current.Pos_offset1 += User_ReadADC_Raw(1);
        User_Delay(2);
    }
    sguan->current.Pos_offset0 = sguan->current.Pos_offset0/24;
    sguan->current.Pos_offset1 = sguan->current.Pos_offset1/24;
    sguan->current.Final_Gain = sguan->motor.MCU_Voltage/
        (sguan->motor.ADC_Precision*sguan->motor.Amplifier*sguan->motor.Sampling_Rs);
}

// Encoder获取实际单圈电子角度(无需滤波)
static float Encoder_ReadErad(SguanFOC_System_STRUCT *sguan){
    return (sguan->encoder.Real_Rad - sguan->encoder.Pos_offset)*sguan->motor.Poles;
}

// Encoder获取实际电机速度(未滤波)
static float Encoder_ReadSpeed(SguanFOC_System_STRUCT *sguan){
    float Encoder_num = sguan->encoder.Real_Rad - normalize_angle(sguan->encoder.pll.go.OutRe);
    
    // 计算角度误差,始终归一化到[-π, π)范围
    if (Encoder_num >= Value_PI){
        Encoder_num -= Value_PI*2;
    }
    if (Encoder_num <= -Value_PI){
        Encoder_num += Value_PI*2;
    }
    
    if (sguan->mode == PosVelCur_THREE_MODE){
        // 位置环模式：PLL连续积分（可以超过2π）
        sguan->encoder.pll.is_position_mode = 1;
    } else{
        // 非位置环模式：PLL输出归一化到[0, 2π)
        sguan->encoder.pll.is_position_mode = 0;
    }
    sguan->encoder.pll.go.Error = Encoder_num;
    PLL_Loop(&sguan->encoder.pll);
    return sguan->encoder.pll.go.OutWe;
}

// Encoder实时更新角速度和角度信息(已滤波)
static void Encoder_Tick(SguanFOC_System_STRUCT *sguan){
    // 0.有传感器FOC控制(全速域)
    sguan->encoder.Real_Rad = User_Encoder_ReadRad();
    sguan->bpf.Encoder.filter.Input = Encoder_ReadSpeed(sguan)*sguan->motor.Encoder_Dir;
    BPF_Loop(&sguan->bpf.Encoder);
    sguan->encoder.Real_Speed = sguan->bpf.Encoder.filter.Output;
    sguan->encoder.Real_Pos = sguan->encoder.pll.go.OutRe*sguan->motor.Encoder_Dir;
    sguan->encoder.Real_Erad = normalize_angle(Encoder_ReadErad(sguan)*sguan->motor.Encoder_Dir);
    sguan->encoder.Real_Espeed = sguan->encoder.Real_Speed*sguan->motor.Poles;
    fast_sin_cos(sguan->encoder.Real_Erad,&sguan->foc.sine,&sguan->foc.cosine);
}

// Current读取当前的电流值并更新3相电流(已滤波)
static void Current_ReadIabc(SguanFOC_System_STRUCT *sguan){
    float I0 = (User_ReadADC_Raw(0) - sguan->current.Pos_offset0)*
                            sguan->current.Final_Gain*sguan->motor.Current_Dir0;
    float I1 = (User_ReadADC_Raw(1) - sguan->current.Pos_offset1)*
                            sguan->current.Final_Gain*sguan->motor.Current_Dir1;
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
    park(&sguan->bpf.CurrentD.filter.Input,
        &sguan->bpf.CurrentQ.filter.Input,
        sguan->current.Real_Ialpha,
        sguan->current.Real_Ibeta,
        sguan->foc.sine,
        sguan->foc.cosine);
    BPF_Loop(&sguan->bpf.CurrentD);
    BPF_Loop(&sguan->bpf.CurrentQ);
    sguan->current.Real_Id = sguan->bpf.CurrentD.filter.Output;
    sguan->current.Real_Iq = sguan->bpf.CurrentQ.filter.Output;
}

// Control速度开环(用于直接控制Uq_in,用于电机测试)
static void Control_Velocity_OPEN(SguanFOC_System_STRUCT *sguan){
    // #if Open_Velocity_OPEN
    // // sguan->foc.Ud_in = 0.0f;
    // // sguan->foc.Uq_in = 2.0f;
    // #endif // Open_Velocity_OPEN
}

// Control电流单环(单闭环)
static void Control_Current_SINGLE(SguanFOC_System_STRUCT *sguan){
    // 1.前馈计算
    float Ud_ff = -sguan->encoder.Real_Espeed*sguan->identify.Lq*sguan->current.Real_Iq; 
    float Uq_ff = sguan->encoder.Real_Espeed*sguan->identify.Ld*sguan->current.Real_Id + 
                    sguan->encoder.Real_Espeed*sguan->identify.Flux;

    // 2.PID计算
    sguan->control.Current_D.run.Ref = sguan->foc.Target_Id; // 默认D轴励磁Id为0
    sguan->control.Current_D.run.Fbk = sguan->current.Real_Id;
    sguan->control.Current_Q.run.Ref = sguan->foc.Target_Iq;
    sguan->control.Current_Q.run.Fbk = sguan->current.Real_Iq;
    PID_Loop(&sguan->control.Current_D);
    PID_Loop(&sguan->control.Current_Q);

    // 3.叠加前馈和PID输出
    sguan->foc.Ud_in = sguan->control.Current_D.run.Output + Ud_ff;
    sguan->foc.Uq_in = sguan->control.Current_Q.run.Output + Uq_ff;
}

// Control速度-电流双环(双闭环)
static void Control_VelCur_DOUBLE(SguanFOC_System_STRUCT *sguan){
    static uint8_t PID_Count = 0;
    PID_Count++;
    if (PID_Count == sguan->control.Response){
        sguan->control.Velocity.run.Ref = sguan->foc.Target_Speed;
        sguan->control.Velocity.run.Fbk = sguan->encoder.Real_Speed;
        PID_Loop(&sguan->control.Velocity);
        PID_Count = 0;
    }
    // 1.前馈计算
    float Ud_ff = -sguan->encoder.Real_Espeed*sguan->identify.Lq*sguan->current.Real_Iq; 
    float Uq_ff = sguan->encoder.Real_Espeed*sguan->identify.Ld*sguan->current.Real_Id + 
                    sguan->encoder.Real_Espeed*sguan->identify.Flux;

    // 2.PID计算
    sguan->control.Current_D.run.Ref = sguan->foc.Target_Id; // 默认D轴励磁Id为0
    sguan->control.Current_D.run.Fbk = sguan->current.Real_Id;
    sguan->control.Current_Q.run.Ref = sguan->control.Velocity.run.Output;
    sguan->control.Current_Q.run.Fbk = sguan->current.Real_Iq;
    PID_Loop(&sguan->control.Current_D);
    PID_Loop(&sguan->control.Current_Q);
    sguan->foc.Ud_in = sguan->control.Current_D.run.Output + Ud_ff;
    sguan->foc.Uq_in = sguan->control.Current_Q.run.Output + Uq_ff;
}

// Control高性能伺服三环(三闭环)
static void Control_PosVelCur_THREE(SguanFOC_System_STRUCT *sguan){
    static uint8_t PID_Count = 0;
    PID_Count++;
    if (PID_Count == sguan->control.Response*sguan->control.Response){
        sguan->control.Position.run.Ref = sguan->foc.Target_Pos;
        sguan->control.Position.run.Fbk = sguan->encoder.Real_Pos;
        PID_Loop(&sguan->control.Position);
        PID_Count = 0;
    }
    if (PID_Count % sguan->control.Response == 0){
        sguan->control.Velocity.run.Ref = sguan->control.Position.run.Output;
        sguan->control.Velocity.run.Fbk = sguan->encoder.Real_Speed;
        PID_Loop(&sguan->control.Velocity);
    }
    // 1.前馈计算
    float Ud_ff = -sguan->encoder.Real_Espeed*sguan->identify.Lq*sguan->current.Real_Iq; 
    float Uq_ff = sguan->encoder.Real_Espeed*sguan->identify.Ld*sguan->current.Real_Id + 
                    sguan->encoder.Real_Espeed*sguan->identify.Flux;

    // 2.PID计算
    sguan->control.Current_D.run.Ref = sguan->foc.Target_Id; // 默认D轴励磁为0
    sguan->control.Current_D.run.Fbk = sguan->current.Real_Id;
    sguan->control.Current_Q.run.Ref = sguan->control.Velocity.run.Output;
    sguan->control.Current_Q.run.Fbk = sguan->current.Real_Iq;
    PID_Loop(&sguan->control.Current_D);
    PID_Loop(&sguan->control.Current_Q);
    sguan->foc.Ud_in = sguan->control.Current_D.run.Output + Ud_ff;
    sguan->foc.Uq_in = sguan->control.Current_Q.run.Output + Uq_ff;
}

static void Control_Sensorless_HFI(SguanFOC_System_STRUCT *sguan){

}

static void Control_Sensorless_SMO(SguanFOC_System_STRUCT *sguan){

}

static void Control_Sensorless_HS(SguanFOC_System_STRUCT *sguan){
    
}

// Data母线电压和驱动器物理温度数据更新
static void Data_Protection_Loop(SguanFOC_System_STRUCT *sguan){
    if (User_VBUS_DataGet() != -9999.0f){        
        sguan->foc.Real_VBUS = User_VBUS_DataGet();
    }
    if (User_Temperature_DataGet() != -9999.0f){        
        sguan->foc.Real_Temp = User_Temperature_DataGet();
    }
}

// Status判断并切换状态机
static void Status_Switch_Loop(SguanFOC_System_STRUCT *sguan){
    // ====== 安全状态(状态) ======
    if ((sguan->status != MOTOR_STATUS_EMERGENCY_STOP) && 
        MOTOR_STATUS_EMERGENCY_STOP_Signal()){
        sguan->status = MOTOR_STATUS_EMERGENCY_STOP;
    }
    if ((sguan->status != MOTOR_STATUS_DISABLED) && 
        MOTOR_STATUS_DISABLED_Signal()){
        sguan->status = MOTOR_STATUS_DISABLED;
    }

    // ====== 初始化与运行状态(状态) ======
    if ((sguan->status != MOTOR_STATUS_UNINITIALIZED) && 
        MOTOR_STATUS_UNINITIALIZED_Signal()){ // [重要]接收到开始信号后，切换到未初始化状态...准备初始化
        sguan->status = MOTOR_STATUS_UNINITIALIZED;
    }
    if ((sguan->status != MOTOR_STATUS_STANDBY) && 
        MOTOR_STATUS_STANDBY_Signal()){ // [重要]接收到待机信号后，解除锁定状态...进入待机状态
        sguan->status = MOTOR_STATUS_STANDBY;
    }
    if ((sguan->status == MOTOR_STATUS_EMERGENCY_STOP) || 
        (sguan->status == MOTOR_STATUS_DISABLED)){
        return; // 紧急停止和失能状态优先级最高, 直接返回不执行后续状态判断
    }

    // ====== 硬件相关错误(状态) ======
    // 1.电机母线电压VBUS状态机
    if (User_VBUS_DataGet() != -9999.0f){        
        if ((sguan->status != MOTOR_STATUS_OVERVOLTAGE) && 
            sguan->foc.Real_VBUS > sguan->safe.VBUS_MAX){
            sguan->status = MOTOR_STATUS_OVERVOLTAGE;
        }
        else if ((sguan->status != MOTOR_STATUS_UNDERVOLTAGE) && 
            sguan->foc.Real_VBUS < sguan->safe.VBUS_MIM){
            sguan->status = MOTOR_STATUS_UNDERVOLTAGE;
        }
    }
    // 2.驱动器物理温度Temp状态机
    if (User_Temperature_DataGet() != -9999.0f){        
        if ((sguan->status != MOTOR_STATUS_UNDERTEMPERATURE) && 
            sguan->foc.Real_Temp > sguan->safe.Temp_MAX){
            sguan->status = MOTOR_STATUS_OVERTEMPERATURE;
        }
        else if ((sguan->status != MOTOR_STATUS_UNDERTEMPERATURE) && 
            sguan->foc.Real_Temp < sguan->safe.Temp_MIN){
            sguan->status = MOTOR_STATUS_UNDERTEMPERATURE;
        }
    }
    // 3.过流保护
    if ((sguan->status != MOTOR_STATUS_OVERCURRENT) && 
        (sguan->current.Real_Id > sguan->safe.Dcur_MAX) || 
        (sguan->current.Real_Iq > sguan->safe.Qcur_MAX)){
        sguan->status = MOTOR_STATUS_OVERCURRENT;
    }
    // 4.编码错误
    #if !MOTOR_CONTROL
    if ((sguan->status != MOTOR_STATUS_ENCODER_ERROR) && 
        MOTOR_STATUS_ENCODER_ERROR_Signal()){
        sguan->status = MOTOR_STATUS_ENCODER_ERROR;
    }
    #endif // !MOTOR_CONTROL
    // 5.传感器错误
    if ((sguan->status != MOTOR_STATUS_SENSOR_ERROR) && 
        MOTOR_STATUS_SENSOR_ERROR_Signal()){
        sguan->status = MOTOR_STATUS_SENSOR_ERROR;
    }
    if (sguan->status >= 14 && sguan->status < 21){
        return; // 硬件相关错误状态优先级高于运行状态, 直接返回不执行后续状态判断
    }

    // ====== 运行状态(当前反馈) ======
    // 1.力矩模式检测
    if ((sguan->mode == Current_SINGLE_MODE) && 
        (sguan->status != MOTOR_STATUS_TORQUE_CONTROL) && 
        (sguan->current.Real_Iq > sguan->foc.Target_Iq*0.95f) && 
        (sguan->current.Real_Iq < sguan->foc.Target_Iq*1.05f)){
        sguan->status = MOTOR_STATUS_TORQUE_CONTROL;
    }
    if ((sguan->mode == Current_SINGLE_MODE) && 
        (sguan->status != MOTOR_STATUS_TORQUE_INCREASING) && 
        (sguan->current.Real_Iq < sguan->foc.Target_Iq*0.95f)){
        sguan->status = MOTOR_STATUS_TORQUE_INCREASING;
    }
    if ((sguan->mode == Current_SINGLE_MODE) && 
        (sguan->status != MOTOR_STATUS_TORQUE_DECREASING) && 
        (sguan->current.Real_Iq > sguan->foc.Target_Iq*1.05f)){
        sguan->status = MOTOR_STATUS_TORQUE_DECREASING;
    }
    // 2.速度模式检测
    if (((sguan->mode == VelCur_DOUBLE_MODE) || 
        (sguan->mode == Sensorless_HFI_MODE) || 
        (sguan->mode == Sensorless_SMO_MODE) || 
        (sguan->mode == Sensorless_HS_MODE)) && 
        (sguan->status != MOTOR_STATUS_CONST_SPEED) && 
        (sguan->encoder.Real_Speed > sguan->foc.Target_Speed*0.95f) && 
        (sguan->encoder.Real_Speed < sguan->foc.Target_Speed*1.05f)){
        sguan->status = MOTOR_STATUS_CONST_SPEED;
    }
    if (((sguan->mode == VelCur_DOUBLE_MODE) || 
        (sguan->mode == Sensorless_HFI_MODE) || 
        (sguan->mode == Sensorless_SMO_MODE) || 
        (sguan->mode == Sensorless_HS_MODE)) && 
        (sguan->status != MOTOR_STATUS_ACCELERATING) && 
        (sguan->encoder.Real_Speed < sguan->foc.Target_Speed*0.95f)){
        sguan->status = MOTOR_STATUS_ACCELERATING;
    }
    if (((sguan->mode == VelCur_DOUBLE_MODE) || 
        (sguan->mode == Sensorless_HFI_MODE) || 
        (sguan->mode == Sensorless_SMO_MODE) || 
        (sguan->mode == Sensorless_HS_MODE)) && 
        (sguan->status != MOTOR_STATUS_DECELERATING) && 
        (sguan->encoder.Real_Speed > sguan->foc.Target_Speed*1.05f)){
        sguan->status = MOTOR_STATUS_DECELERATING;
    }
    // 3.位置模式检测
    if ((sguan->mode == PosVelCur_THREE_MODE) && 
        (sguan->status != MOTOR_STATUS_POSITION_HOLD) && 
        (sguan->encoder.Real_Pos > sguan->foc.Target_Pos*0.95f) && 
        (sguan->encoder.Real_Pos < sguan->foc.Target_Pos*1.05f)){
        sguan->status = MOTOR_STATUS_POSITION_HOLD;
    }
    if ((sguan->mode == PosVelCur_THREE_MODE) && 
        (sguan->status != MOTOR_STATUS_POSITION_INCREASING) && 
        (sguan->encoder.Real_Pos < sguan->foc.Target_Pos*0.95f)){
        sguan->status = MOTOR_STATUS_POSITION_INCREASING;
    }
    if ((sguan->mode == PosVelCur_THREE_MODE) && 
        (sguan->status != MOTOR_STATUS_POSITION_DECREASING) && 
        (sguan->encoder.Real_Pos > sguan->foc.Target_Pos*1.05f)){
        sguan->status = MOTOR_STATUS_POSITION_DECREASING;
    }
}

static void Status_RUN_Loop(SguanFOC_System_STRUCT *sguan){
    // 1.状态机失能DISABLED安全状态处理
    static uint32_t count = 0;
    count++;
    if (sguan->status == MOTOR_STATUS_DISABLED){
        if ((count > sguan->safe.DISABLED_watchdog_limit) && 
            (sguan->status == MOTOR_STATUS_DISABLED)){
            sguan->status = MOTOR_STATUS_STANDBY;
            count = 0;
             // 已失能状态持续“自定义”次控制周期后，自动切换退出已失能状态
        }
    }
    // 2.电机电压异常_错误处理
    if (sguan->status == MOTOR_STATUS_OVERVOLTAGE){
        if (count > sguan->safe.VBUS_watchdog_limit/2)
        {
            
        }
        if ((count > sguan->safe.VBUS_watchdog_limit) && 
            (sguan->status == MOTOR_STATUS_OVERVOLTAGE)){ 
            sguan->status = MOTOR_STATUS_STANDBY;
            count = 0;
            // 持续自定义的周期后，自动切换退出过压保护，并进入STANDBY
        }
    }
    if (sguan->status == MOTOR_STATUS_UNDERVOLTAGE){
        if ((count > sguan->safe.VBUS_watchdog_limit) && 
            (sguan->status == MOTOR_STATUS_UNDERVOLTAGE)){ 
            sguan->status = MOTOR_STATUS_STANDBY;
            count = 0;
            // 持续自定义的周期后，自动切换退出欠压保护，并进入STANDBY
        }
    }
    // 3.电机驱动器温度异常_错误处理
    if (sguan->status == MOTOR_STATUS_OVERTEMPERATURE){
        if ((count > sguan->safe.Temp_watchdog_limit) && 
            (sguan->status == MOTOR_STATUS_OVERTEMPERATURE)){ 
            sguan->status = MOTOR_STATUS_STANDBY;
            count = 0;
            // 持续自定义的周期后，自动切换退出过温保护，并进入STANDBY
        }
    }
    if (sguan->status == MOTOR_STATUS_UNDERTEMPERATURE){
        if ((count > sguan->safe.Temp_watchdog_limit) && 
            (sguan->status == MOTOR_STATUS_UNDERTEMPERATURE)){ 
            sguan->status = MOTOR_STATUS_STANDBY;
            count = 0;
            // 持续自定义的周期后，自动切换退出低温保护，并进入STANDBY
        }
    }
    // 4.电机过流保护OVERCURRENT错误处理
    if (sguan->status == MOTOR_STATUS_OVERCURRENT){
        if ((count > sguan->safe.DQcur_watchdog_limit) && 
            (sguan->status == MOTOR_STATUS_OVERCURRENT)){ 
            sguan->status = MOTOR_STATUS_STANDBY;
            count = 0;
            // 持续自定义的周期后，自动切换退出过流保护，并进入STANDBY
        }
    }
    MotorStatus_Loop(&Sguan.status);
}

// Printf电机调试信息发送
#if Printf_Debug
static void Printf_Debug_Loop(SguanFOC_System_STRUCT *sguan){
    static uint8_t status = 0xFF;
    static uint32_t count = 0;
    if (sguan->status != status){
        static const char* status_names[] = {
            // ====== 初始化与运行状态(状态) ======
            "待机MOTOR_STATUS_STANDBY",
            "未初始化MOTOR_STATUS_UNINITIALIZED",
            "初始化中MOTOR_STATUS_INITIALIZING",
            "校准MOTOR_STATUS_CALIBRATING",
            // ====== 运行状态(当前反馈) ======
            "空闲MOTOR_STATUS_IDLE",
            "力矩增大中~电流模式MOTOR_STATUS_TORQUE_INCREASING",
            "力矩减小中~电流模式MOTOR_STATUS_TORQUE_DECREASING",
            "力矩保持~电流模式MOTOR_STATUS_TORQUE_HOLDING",
            "加速中~速度模式MOTOR_STATUS_ACCELERATING",
            "减速中~速度模式MOTOR_STATUS_DECELERATING",
            "恒速保持~速度模式MOTOR_STATUS_CONST_SPEED",
            "位置增加中~位置模式MOTOR_STATUS_POSITION_INCREASING",
            "位置减少中~位置模式MOTOR_STATUS_POSITION_DECREASING",
            "位置保持~位置模式MOTOR_STATUS_POSITION_HOLD",
            // ====== 硬件相关错误(状态) ======
            "过压保护MOTOR_STATUS_OVERVOLTAGE",
            "欠压保护MOTOR_STATUS_UNDERVOLTAGE",
            "过温保护MOTOR_STATUS_OVERTEMPERATURE",
            "低温保护MOTOR_STATUS_LOWTEMPERATURE",
            "过流保护MOTOR_STATUS_OVERCURRENT",
            "编码器故障MOTOR_STATUS_ENCODER_ERROR",
            "传感器故障MOTOR_STATUS_SENSOR_ERROR",
            "PWM计算错误MOTOR_STATUS_PWM_ERROR",
            // ====== 安全状态(状态) ======
            "急停MOTOR_STATUS_EMERGENCY_STOP",
            "已失能MOTOR_STATUS_DISABLED"
        };
        
        printf("[电机控制,状态机更新%lu]: %s...状态机编号%d\n", 
                   count, 
                   status_names[sguan->status], 
                   sguan->status);
        count ++;
        status = sguan->status;
    }
}
#else // Printf_Debug
// Printf电机数据正常发送
static void Printf_Normal_Loop(SguanFOC_System_STRUCT *sguan){
    // 用户数据填写(串口或者CAN通信)
    User_UserTX();
    // 发送数据to上位机
    Printf_Loop(&Sguan.TXdata);
}
#endif // Printf_Debug

// SVPWM电机驱动的马鞍波生成
static void SVPWM_Tick(SguanFOC_System_STRUCT *sguan,float Erad,float d_set,float q_set){
    SVPWM(d_set,q_set,
        sguan->foc.sine,
        sguan->foc.cosine,
        &sguan->foc.Du,
        &sguan->foc.Dv,
        &sguan->foc.Dw);
    if (sguan->motor.PWM_Dir == 1){
        sguan->foc.Duty_u = (uint16_t)(sguan->foc.Du*sguan->motor.Duty);
        sguan->foc.Duty_v = (uint16_t)(sguan->foc.Dv*sguan->motor.Duty);
        sguan->foc.Duty_w = (uint16_t)(sguan->foc.Dw*sguan->motor.Duty);
    }
    else if (sguan->motor.PWM_Dir == -1){
        sguan->foc.Duty_u = (uint16_t)((1.0f - sguan->foc.Du)*sguan->motor.Duty);
        sguan->foc.Duty_v = (uint16_t)((1.0f - sguan->foc.Dv)*sguan->motor.Duty);
        sguan->foc.Duty_w = (uint16_t)((1.0f - sguan->foc.Dw)*sguan->motor.Duty);
    }
    if (sguan->motor.Motor_Dir == -1){ // 判断电机方向并修改(原理是AB相序交换)
        uint16_t duty_temp = sguan->foc.Duty_u;
        sguan->foc.Duty_u = sguan->foc.Duty_v;
        sguan->foc.Duty_v = duty_temp;
    }
    User_PwmDuty_Set(sguan->foc.Duty_u,sguan->foc.Duty_v,sguan->foc.Duty_w);
}

// Sguan...Loop计算角速度/角度和电流值
static void Sguan_Calculate_Loop(SguanFOC_System_STRUCT *sguan){
    Encoder_Tick(sguan);
    Current_Tick(sguan);
}

// Sguan...Loop计算PID并执行电机控制
static void Sguan_GeneratePWM_Loop(SguanFOC_System_STRUCT *sguan){
    // 用户实时控制的参数传入
    User_UserControl();
    if ((sguan->status >= 4) && (sguan->status < 19)){
        // PID运算PWM大小并执行
        if (sguan->mode < 7){
            Control_Tick[sguan->mode](sguan);
        } else{
            // 错误处理：自动跳转到默认速度单闭环模式
            sguan->mode = Velocity_OPEN_MODE;
            Control_Tick[Velocity_OPEN_MODE](sguan);
        }
        SVPWM_Tick(sguan,sguan->encoder.Real_Erad,
            sguan->foc.Ud_in/sguan->motor.VBUS,
            sguan->foc.Uq_in/sguan->motor.VBUS);
    }
}

// Sguan...Init电机参数辨识和自适应初始化
static void Sguan_Quantize_Init(void){
    // #if Open_Quantize_Method
    // uint8_t count = 0;
    // #endif // Open_Quantize_Method
}

// Sguan...Init巴特沃斯低通滤波器的初始化
static void Sguan_BPF_Init(SguanFOC_System_STRUCT *sguan){
    BPF_Init(&sguan->bpf.CurrentD);
    BPF_Init(&sguan->bpf.CurrentQ);
    BPF_Init(&sguan->bpf.Encoder);
}

// Sguan...Init闭环控制算法PID的初始化
static void Sguan_PID_Init(SguanFOC_System_STRUCT *sguan){
    // 单闭环初始化(电流单闭环)
    PID_Init(&sguan->control.Current_D);
    PID_Init(&sguan->control.Current_Q);
    // 双闭环(速度-电流||无感控制)
    PID_Init(&sguan->control.Velocity);
    // 三闭环(位置-速度-电流)
    PID_Init(&sguan->control.Position);
}

// Sguan...Init锁相环PLL的初始化
static void Sguan_PLL_Init(SguanFOC_System_STRUCT *sguan){
    PLL_Init(&sguan->encoder.pll);
}

// Sguan...Init无感算法Sensorless的初始化
static void Sguan_Sensorless_Init(SguanFOC_System_STRUCT *sguan){
    // HFI_Init(&sguan->hfi);
    // SMO_Init(&sguan->smo);
}

// Pre电机零点对齐(机械角度对齐)
static void Sguan_Pre_Positioning(SguanFOC_System_STRUCT *sguan){
    SVPWM_Tick(sguan,0.0f,0.2f,0.0f);
}

// Pre系统时钟设置(定时器中断周期)
static void Sguan_SystemT_Set(SguanFOC_System_STRUCT *sguan){
    // 0.电机编码器参数
    sguan->motor.Encoder_T = sguan->PMSM_RUN_T;
    // 1.bpf低通滤波器
    sguan->bpf.CurrentD.T = sguan->PMSM_RUN_T;
    sguan->bpf.CurrentQ.T = sguan->PMSM_RUN_T;
    sguan->bpf.Encoder.T = sguan->PMSM_RUN_T;
    // 2.pid闭环控制系统
    sguan->control.Current_D.T = sguan->PMSM_RUN_T;
    sguan->control.Current_Q.T = sguan->PMSM_RUN_T;
    sguan->control.Velocity.T = sguan->PMSM_RUN_T*sguan->control.Response;
    sguan->control.Position.T = sguan->PMSM_RUN_T*sguan->control.Response*sguan->control.Response;
    // 3.pll锁相环跟踪系统
    sguan->encoder.pll.T = sguan->PMSM_RUN_T;
}

// Pre系统开始的核心文件，主任务初始化函数
static void Sguan_Start_Loop(void){
    if (Sguan.status == MOTOR_STATUS_UNINITIALIZED){
        // 用户自定义的电机参数和控制系统参数
        User_MotorSet();
        User_ParameterSet();
        // 系统时间设定
        Sguan_SystemT_Set(&Sguan);
        // 用户自定义的Init(主要用于物理驱动器)
        Sguan.status = MOTOR_STATUS_INITIALIZING;
        User_InitialInit();
        // 控制系统量化和电机参数辨识(离线测量)
        Sguan_Quantize_Init();
        // 各种控制系统的初始化
        Sguan_BPF_Init(&Sguan);
        Sguan_PID_Init(&Sguan);
        Sguan_PLL_Init(&Sguan);
        Sguan_Sensorless_Init(&Sguan);
        Printf_Init(&Sguan.TXdata);
        // 读取电流偏置
        Sguan.status = MOTOR_STATUS_CALIBRATING;
        Offset_CurrentRead(&Sguan);
        //电机回零操作
        Sguan_Pre_Positioning(&Sguan);
        User_Delay(1200);
        // 读取角度偏置
        Offset_EncoderRead(&Sguan);
        // 电机失能并进入正常工作状态
        Sguan_Pre_Positioning(&Sguan,0.0f);
        User_Delay(1200);
        //正常工作中(状态机运行)
        Sguan.status = MOTOR_STATUS_IDLE;
    }
}

/**
 * @description: SguanFOC核心文件，定时中断服务函数(高频率电机载波)
 * @reminder: 10Khz或者更高定时中断中调用，任务优先级“最高”
 * @return {*}
 */
void SguanFOC_Loop(void){
    static uint8_t PWM_watchdog_counter = 0;
    Sguan.flag.in_PWM_Calc_ISR = 1;
    if (!Sguan.flag.PWM_Calc){
        Sguan.flag.PWM_Calc = 1;
        PWM_watchdog_counter = 0;
        // 计算编码器和电流
        Sguan_Calculate_Loop(&Sguan);
        // 运算PID并执行SVPWM(如果计算超时，会更新错误状态并停用此线程)
        Sguan_GeneratePWM_Loop(&Sguan);
        #if Printf_Debug
        Printf_Debug_Loop(&Sguan);
        #endif // Printf_Debug
        Sguan.flag.PWM_Calc = 0;
    }
    else{
        PWM_watchdog_counter++;
        Sguan.flag.PWM_Calc = 0;
        if (PWM_watchdog_counter > Sguan.flag.PWM_watchdog_limit){
            Sguan.status = MOTOR_STATUS_PWM_CALC_FAULT;
            PWM_watchdog_counter = 0;
        }
    }
    Sguan.flag.in_PWM_Calc_ISR = 0;
}

/**
 * @description: SguanFOC核心文件，定时中断服务函数(1ms周期数据更新)
 * @reminder: 1Khz或者更低定时中断中调用，任务优先级“中”
 * @return {*}
 */
void SguanFOC_msTick(void){
    if (Sguan.flag.in_PWM_Calc_ISR) {
        return; // (互斥锁)PWM计算中断正在执行，跳过本次1ms更新
    }
    
    // 1.读取母线电压VBUS和温度数据Temp
    Data_Protection_Loop(&Sguan);
    // 2.根据环境切换电机状态机
    Status_Switch_Loop(&Sguan);
    // 3.根据电机状态机,运行不同任务
    Status_RUN_Loop(&Sguan);
}

/**
 * @description: SguanFOC核心文件，UART或者CAN接收完成中断服务函数
 * @reminder: 主循环函数调用，任务优先级“低”
 * @param {uint8_t} *data 接收到的数据
 * @param {uint16_t} length 数据长度
 * @return {*}
 */
void SguanFOC_PrintfTick(uint8_t *data, uint16_t length){
    if(length > sizeof(Sguan_PrintfBuff)){
        length = sizeof(Sguan_PrintfBuff);
    }

    for(uint16_t i = 0; i < length; i++){
        if(data[i] == '?'){
            Printf_Adjust();
            // 内存对齐的优化清零
            uint32_t *p32 = (uint32_t*)Sguan_PrintfBuff;
            uint16_t size = sizeof(Sguan_PrintfBuff);
            // 按4字节清空
            for(uint16_t j = 0; j < size / 4; j++){
                p32[j] = 0;
            }
            // 处理剩余字节
            uint8_t *p8 = (uint8_t*)&p32[size / 4];
            for(uint16_t j = 0; j < size % 4; j++){
                p8[j] = 0;
            }
            break;
        }
    }
}

/**
 * @description: SguanFOC核心文件，主循环服务函数(主循环TXdata数据更新)
 * @reminder: 主循环函数调用，任务优先级“最低”
 * @return {*}
 */
void SguanFOC_mainTick(void){
    Sguan_Start_Loop();
    #if !Printf_Debug
    Printf_Normal_Loop(&Sguan);
    #endif // Printf_Debug
}


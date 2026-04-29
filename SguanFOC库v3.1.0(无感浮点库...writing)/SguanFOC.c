/*
 * @Author: 星必尘Sguan
 * @GitHub: https://github.com/Sguan-ZhouQing
 * @Date: 2026-01-26 22:38:34
 * @LastEditors: 星必尘Sguan|3464647102@qq.com
 * @LastEditTime: 2026-04-30 01:28:46
 * @FilePath: \SguanFOC_Debug\SguanFOC\SguanFOC.c
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

/**
 * @description: 1.电机控制函数实现(Sguan.*使用)
 * @reminder: Sguan控制函数接口的使用方法，如下：
 * @reminder: 启动电机  Sguan.Func_Start()
 * @reminder: 停止电机  Sguan.Func_Stop()
 * @reminder: 设置模式  Sguan.Func_Set_Mode(VF_OPENLOOP_MODE)
 * @reminder: 设计电压  Sguan.Func_Set_Uq(1.68f)
 * @reminder: 设计电流  Sguan.Func_Set_Iq(2.68f)
 * @reminder: 设计转速  Sguan.Func_Set_Velocity(16.8f)
 * @reminder: 设计位置  Sguan.Func_Set_Position(26.8f)
 * @reminder: 数据打印  Sguan.Func_Set_TXdata(0,Sguan.status)
 * @param {void} *ctrl
 * @return {*}
 */
static void Function_Start(void);
static void Function_Stop(void);
static void Function_SetMode(uint8_t mode);
static void Function_SetUq(float uq);
static void Function_SetIq(float iq);
static void Function_SetVelocity(float speed);
static void Function_SetPosition(float pos);
static void Function_SetTXdata(uint8_t ch,float data);

// 电机控制核心结构体设计
SguanFOC_System_STRUCT Sguan = {
    .Func_Start = Function_Start,
    .Func_Stop = Function_Stop,
    .Func_Set_Mode = Function_SetMode,
    .Func_Set_Uq = Function_SetUq,
    .Func_Set_Iq = Function_SetIq,
    .Func_Set_Velocity = Function_SetVelocity,
    .Func_Set_Position = Function_SetPosition,
    .Func_Set_TXdata = Function_SetTXdata
};

// ============================= SguanFOC版本代码(仅声明) ============================
/**
 * @description: 2.Transfer传递函数的离散化运算，采用双线性变换
 * @param {PID_STRUCT} *pid (控制)PID闭环控制系统运算
 * @param {LADRC_STRUCT} *ladrc (控制)LADRC线自抗扰运算
 * @param {SMC_STRUCT} *smc (控制)SMC传统滑模控制运算
 * @param {STA_STRUCT} *sta (控制)STA超螺旋简化滑模运算
 * @param {BPF_STRUCT} *bpf (滤波)BPF二阶巴特沃斯低通滤波
 * @param {PLL_STRUCT} *pll (估算)PLL速度跟踪锁相环
 * @param {DOB_STRUCT} *dob (观测器)DOB超螺旋滑模扰动观测器
 * @return {*}
 */
static void Transfer_PID_Loop(void *pid,float Ref,float Fbk);
static void Transfer_Ladrc_Loop(void *ladrc,float Ref,float Fbk);
static void Transfer_SMC_Loop(void *smc,float Ref,float Fbk);
static void Transfer_STA_Loop(void *sta,float Ref,float Fbk);
static void (*const CtrlFunc_Tick[])(void *, float, float)={
    /*CtrlFunc_Tick[CONFIG_CtrlVel](*)或者CtrlFunc_Tick[CONFIG_CtrlPos](*)*/
    Transfer_PID_Loop,              // PID
    Transfer_Ladrc_Loop,            // Ladrc
    Transfer_SMC_Loop,              // SMC
    Transfer_STA_Loop               // STA
};
static void Transfer_LPF_Loop(LPF_STRUCT *lpf,float input);
static void Transfer_Hall_Loop(HALL_STRUCT *hal,
                            float Raw_A,
                            float Raw_B,
                            float Raw_C);
static void Transfer_PLL_Loop(PLL_STRUCT *pll,uint8_t mode,float input_Rad);
#if CONFIG_DOB
static void Transfer_DOB_Loop(DOB_STRUCT *dob,float Iq,float Wm);
#endif // CONFIG_DOB
/**
 * @description: 3.Transfer传递函数的初始化
 * @param {SguanFOC_System_STRUCT} *sguan
 * @return {*}
 */
static void Transfer_Init(SguanFOC_System_STRUCT *sguan);
static void Transfer_Null(SguanFOC_System_STRUCT *sguan);
/**
 * @description: 4.Offset内部静态函数声明
 * @param {SguanFOC_System_STRUCT} *sguan
 * @return {*}
 */
static void Offset_EncoderRead(SguanFOC_System_STRUCT *sguan);
static void Offset_CurrentRead(SguanFOC_System_STRUCT *sguan);
/**
 * @description: 5.Current内部静态函数声明
 * @param {SguanFOC_System_STRUCT} *sguan
 * @return {*}
 */
static void Current_ReadIabc(SguanFOC_System_STRUCT *sguan);
/**
 * @description: 6.Sguan_Calculate_Loop内部静态函数声明
 * @param {SguanFOC_System_STRUCT} *sguan
 * @return {*}
 */
static void Sensor_Encoder_Calculate(SguanFOC_System_STRUCT *sguan);
static void Sensor_HALL_Calculate(SguanFOC_System_STRUCT *sguan);
static void Sensorless_HFI_Calculate(SguanFOC_System_STRUCT *sguan);
static void Sensorless_SMO_Calculate(SguanFOC_System_STRUCT *sguan);
static void Sensorless_HS_Calculate(SguanFOC_System_STRUCT *sguan);
static void (*const Encoder_Tick[])(SguanFOC_System_STRUCT*)={
    /*这里注意“枚举变量”的边界, Encoder_Tick[sguan->mode](sguan)使用*/
    Sensor_Encoder_Calculate,       // VF_OPENLOOP_MODE    = 0
    Sensor_Encoder_Calculate,       // IF_OPENLOOP_MODE    = 1
    Sensor_Encoder_Calculate,       // Voltag_OPEN_MODE    = 2
    Sensor_Encoder_Calculate,       // Current_SINGLE_MODE = 3
    Sensor_Encoder_Calculate,       // VelCur_DOUBLE_MODE  = 4
    Sensor_Encoder_Calculate,       // PosVelCur_THREE_MODE= 5
    Sensor_HALL_Calculate,          // Sensor_Hall_MODE    = 6
    Sensorless_HFI_Calculate,       // Sensorless_HFI_MODE = 7
    Sensorless_SMO_Calculate,       // Sensorless_SMO_MODE = 8
    Sensorless_HS_Calculate         // Sensorless_HS_MODE  = 9
};
/**
 * @description: 7.Control运算及其模式切换
 * @param {SguanFOC_System_STRUCT} *sguan
 * @return {*}
 */
static void Control_Current_SINGLE(SguanFOC_System_STRUCT *sguan);
static void Control_VelCur_DOUBLE(SguanFOC_System_STRUCT *sguan);
static void Control_PosVelCur_THREE(SguanFOC_System_STRUCT *sguan);
static void (*const Control_Tick[])(SguanFOC_System_STRUCT*)={
    /*这里注意“枚举变量”的边界, Control_Tick[sguan->mode](sguan)使用*/
    Transfer_Null,                  // VF_OPENLOOP_MODE    = 0
    Control_Current_SINGLE,         // IF_OPENLOOP_MODE    = 1
    Transfer_Null,                  // Voltag_OPEN_MODE    = 2
    Control_Current_SINGLE,         // Current_SINGLE_MODE = 3
    Control_VelCur_DOUBLE,          // VelCur_DOUBLE_MODE  = 4
    Control_PosVelCur_THREE,        // PosVelCur_THREE_MODE= 5
    Control_VelCur_DOUBLE,          // Sensor_Hall_MODE    = 6
    Control_VelCur_DOUBLE,          // Sensorless_HFI_MODE = 7
    Control_VelCur_DOUBLE,          // Sensorless_SMO_MODE = 8
    Control_VelCur_DOUBLE           // Sensorless_HS_MODE  = 9
};
/**
 * @description: 8.SVPWM电机驱动的马鞍波生成
 * @param {SguanFOC_System_STRUCT} *sguan
 * @param {float} d
 * @param {float} q
 * @return {*}
 */
static void PWM_Tick(SguanFOC_System_STRUCT *sguan,
                    float sine,
                    float cosine,
                    float d_duty,
                    float q_duty);
/**
 * @description: 9.Data母线电压和驱动器物理温度数据更新
 * @param {SguanFOC_System_STRUCT} *sguan
 * @return {*}
 */
static void Data_Protection_Loop(SguanFOC_System_STRUCT *sguan);
/**
 * @description: 10.Status判断并切换状态机
 * @param {SguanFOC_System_STRUCT} *sguan
 * @param {uint32_t} *count
 * @return {*}
 */
static void Status_Switch_STANDBY(SguanFOC_System_STRUCT *sguan,uint32_t *count);
static void Status_Switch_IDLE(SguanFOC_System_STRUCT *sguan,uint32_t *count);
static void Status_VBUS_OVERVOLTAGE(SguanFOC_System_STRUCT *sguan,uint32_t *count);
static void Status_VBUS_UNDERVOLTAGE(SguanFOC_System_STRUCT *sguan,uint32_t *count);
static void Status_Temp_OVERTEMPERATURE(SguanFOC_System_STRUCT *sguan,uint32_t *count);
static void Status_Temp_UNDERTEMPERATURE(SguanFOC_System_STRUCT *sguan,uint32_t *count);
static void Status_Current_OVERCURRENT(SguanFOC_System_STRUCT *sguan,uint32_t *count);
static void Status_Switch_Loop(SguanFOC_System_STRUCT *sguan);
static void Status_RUN_Loop(SguanFOC_System_STRUCT *sguan);
/**
 * @description: 11.Printf数据发送Debug和正常模式
 * @param {SguanFOC_System_STRUCT} *sguan
 * @return {*}
 */
#if CONFIG_Debug
static void Printf_Debug_Loop(SguanFOC_System_STRUCT *sguan);
#else // CONFIG_Debug
static void Printf_Normal_Loop(SguanFOC_System_STRUCT *sguan);
#endif // CONFIG_Debug
/**
 * @description: 12.Set各种控制电机回零的设置
 * @param {SguanFOC_System_STRUCT} *sguan
 * @return {*}
 */
static void Positioning_Set(SguanFOC_System_STRUCT *sguan,float Ud,float Uq);
/**
 * @description: 13.Sguan...Loop电机总初始化与校准
 * @param {SguanFOC_System_STRUCT} *sguan
 * @return {*}
 */
static void Sguan_Calculate_Loop(SguanFOC_System_STRUCT *sguan);
static void Sguan_GeneratePWM_Loop(SguanFOC_System_STRUCT *sguan);
static void Sguan_ReInit_Loop(SguanFOC_System_STRUCT *sguan);


// =============================== 电机控制函数(接口) =============================
// Function控制接口->启动电机
static void Function_Start(void){
    Sguan.status = MOTOR_STATUS_UNINITIALIZED;
}

// Function控制接口->停止电机
static void Function_Stop(void){
    Sguan.status = MOTOR_STATUS_STANDBY;
}

// Function控制接口->设置模式
static void Function_SetMode(uint8_t mode){
    Sguan.mode = mode;
}

// Function控制接口->设计目标电压
static void Function_SetUq(float uq){
    Sguan.foc.Uq_in = uq;
}

// Function控制接口->设计目标电流
static void Function_SetIq(float iq){
    Sguan.foc.Target_Iq = iq;
}

// Function控制接口->设计目标转速
static void Function_SetVelocity(float speed){
    Sguan.foc.Target_Speed = speed;
}

// Function控制接口->设计目标位置
static void Function_SetPosition(float pos){
    Sguan.foc.Target_Pos = pos;
}

// Function控制接口->数据打印
static void Function_SetTXdata(uint8_t ch,float data){
    Sguan.txdata.fdata[ch] = data;
}

// ============================= SguanFOC版本代码(仅实现) ============================
// Transfer运算_PID运算
static void Transfer_PID_Loop(void *pid,float Ref,float Fbk){
    PID_STRUCT *p = (PID_STRUCT*)pid;
    p->run.Ref = Ref;
    p->run.Fbk = Fbk;
    PID_Loop(p);
    // 输出pid->run.Output;
}

// Transfer运算_LADRC控制
static void Transfer_Ladrc_Loop(void *ladrc,float Ref,float Fbk){
    LADRC_STRUCT *p = (LADRC_STRUCT*)ladrc;
    p->run.Ref = Ref;
    p->run.Fbk = Fbk;
    Ladrc_Loop(p);
    // 输出ladrc->run.Output;
}

// Transfer运算_SMC控制
static void Transfer_SMC_Loop(void *smc,float Ref,float Fbk){
    SMC_STRUCT *p = (SMC_STRUCT*)smc;
    p->run.Ref = Ref;
    p->run.Fbk = Fbk;
    SMC_Loop(p);
    // 输出SMC->run.Output;
}

// Transfer运算_STA控制
static void Transfer_STA_Loop(void *sta,float Ref,float Fbk){
    STA_STRUCT *p = (STA_STRUCT*)sta;
    p->run.Ref = Ref;
    p->run.Fbk = Fbk;
    STA_Loop(p);
    // 输出sta->run.Output;
}

// Transfer运算_二阶巴特沃斯低通滤波器
static void Transfer_LPF_Loop(LPF_STRUCT *lpf,float input){
    lpf->filter.Input = input;
    LPF_Loop(lpf);
    // 输出bpf->filter.Output;
}

static void Transfer_Hall_Loop(HALL_STRUCT *hall,
                            float Raw_A,
                            float Raw_B,
                            float Raw_C){
    hall->rad.Input_Ga = Raw_A;
    hall->rad.Input_Gb = Raw_B;
    hall->rad.Input_Gc = Raw_C;
    Hall_Loop(hall);
    // 输出hall->rad.Output_Rad;
}

// Transfer运算_速度锁相环
static void Transfer_PLL_Loop(PLL_STRUCT *pll,uint8_t mode,float input_Rad){
    if (mode == PosVelCur_THREE_MODE){
        pll->go.Error = input_Rad - Value_normalize(pll->go.OutRe);
        // 位置环模式：PLL连续积分（可以超过2π）
        if (!pll->is_position_mode){
            pll->is_position_mode = 1;
        }
    }
    else{
        pll->go.Error = input_Rad - pll->go.OutRe;
        // 非位置环模式：PLL输出归一化到[0, 2π)
        if (pll->is_position_mode){
            pll->is_position_mode = 0;
        }
    }

    // 计算角度误差,始终归一化到[-π, π)范围
    if (pll->go.Error >= Value_PI){
        pll->go.Error -= Value_2PI;
    }
    if (pll->go.Error <= -Value_PI){
        pll->go.Error += Value_2PI;
    }

    PLL_Loop(pll);
    // 输出pll->go.OutWe;
    // 输出pll->go.OutRe;
}

// Transfer运算_超螺旋滑模扰动观测器
#if CONFIG_DOB
static void Transfer_DOB_Loop(DOB_STRUCT *dob,float Iq,float Wm){
    dob->smdo.Input_Iq = Iq;
    dob->smdo.Input_Wm = Wm;
    DOB_Loop(dob);
    // 输出dob->smdo.Output_Fd;
    // 输出dob->smdo.Output_Wm;
}
#endif // CONFIG_DOB

// Transfer传递函数初始化
static void Transfer_Init(SguanFOC_System_STRUCT *sguan){
    // 1.初始化系统离散运行时间
    sguan->transfer.Current_D.T = PMSM_RUN_T;
    sguan->transfer.Current_Q.T = PMSM_RUN_T;
    sguan->transfer.Velocity.T = PMSM_RUN_T*sguan->transfer.Response;
    sguan->transfer.Position.T = PMSM_RUN_T*sguan->transfer.Response*
                                sguan->transfer.Response;
    sguan->transfer.LPF_D.T = PMSM_RUN_T;
    sguan->transfer.LPF_Q.T = PMSM_RUN_T;
    sguan->transfer.LPF_encoder.T = PMSM_RUN_T;
    sguan->transfer.Hall.T = PMSM_RUN_T;
    sguan->transfer.PLL.T = PMSM_RUN_T;
    #if CONFIG_DOB
    sguan->transfer.DOB.T = PMSM_RUN_T;
    #endif // CONFIG_DOB
    #if CONFIG_FW
    sguan->transfer.FW.T = PMSM_RUN_T;
    #endif // CONFIG_FW

    // 2.电机默认参数自动填写
    #if CONFIG_CtrlVel==2
    sguan->transfer.Velocity.Gain = sguan->motor.identify.J/
        (1.5f*sguan->motor.Poles*sguan->motor.identify.Flux);
    #endif // CONFIG_Control
    #if CONFIG_CtrlPos==2
    sguan->transfer.Position.Gain = 1.0f;
    #endif // CONFIG_Control

    #if CONFIG_DOB
    sguan->transfer.DOB.Pn = sguan->motor.Poles;
    sguan->transfer.DOB.Flux = sguan->motor.identify.Flux;
    sguan->transfer.DOB.B = sguan->motor.identify.B;
    sguan->transfer.DOB.J = sguan->motor.identify.J;
    #endif // CONFIG_DOB

    // 3.函数调用初始化
    PID_Init(&sguan->transfer.Current_D);
    PID_Init(&sguan->transfer.Current_Q);

    #if CONFIG_CtrlVel==1
    Ladrc_Init(&sguan->transfer.Velocity);
    #elif CONFIG_CtrlVel==2
    SMC_Init(&sguan->transfer.Velocity);
    #elif CONFIG_CtrlVel==3
    STA_Init(&sguan->transfer.Velocity);
    #else // CONFIG_CtrlVel
    PID_Init(&sguan->transfer.Velocity);
    #endif // CONFIG_Control

    #if CONFIG_CtrlPos==1
    Ladrc_Init(&sguan->transfer.Position);
    #elif CONFIG_CtrlPos==2
    SMC_Init(&sguan->transfer.Position);
    #elif CONFIG_CtrlPos==3
    STA_Init(&sguan->transfer.Position);
    #else // CONFIG_CtrlPos
    PID_Init(&sguan->transfer.Position);
    #endif // CONFIG_CtrlPos

    LPF_Init(&sguan->transfer.LPF_D);
    LPF_Init(&sguan->transfer.LPF_Q);
    LPF_Init(&sguan->transfer.LPF_encoder);

    Hall_Init(&sguan->transfer.Hall);
    PLL_Init(&sguan->transfer.PLL);

    #if CONFIG_DOB
    DOB_Init(&sguan->transfer.DOB);
    #endif // CONFIG_DOB
    #if CONFIG_FW
    PID_Init(&sguan->transfer.FW);
    #endif // CONFIG_FW
}

// Transfer空传递函数
static void Transfer_Null(SguanFOC_System_STRUCT *sguan){
    // 空函数，不执行具体数据
}

// Offset读取编码器偏置
static void Offset_EncoderRead(SguanFOC_System_STRUCT *sguan){
    for (uint8_t i = 0; i < 10; i++){
        sguan->encoder.Pos_offset += User_Encoder_ReadRad();
        User_Delay(2);
    }
    sguan->encoder.Pos_offset = sguan->encoder.Pos_offset/10.0f;
}

// Offset读取电流偏置
static void Offset_CurrentRead(SguanFOC_System_STRUCT *sguan){
    for (uint8_t i = 0; i < 24; i++){
        sguan->current.Current_offset0 += User_ReadADC_Raw(0);
        sguan->current.Current_offset1 += User_ReadADC_Raw(1);
        User_Delay(2);
    }
    sguan->current.Current_offset0 = sguan->current.Current_offset0/24;
    sguan->current.Current_offset1 = sguan->current.Current_offset1/24;
    sguan->current.Final_Gain = sguan->motor.MCU_Voltage/
        (sguan->motor.ADC_Precision*sguan->motor.Amplifier*sguan->motor.Sampling_Rs);
}

// Current读取当前的电流值并更新3相电流(未滤波)
static void Current_ReadIabc(SguanFOC_System_STRUCT *sguan){
    float I0 = (User_ReadADC_Raw(0) - sguan->current.Current_offset0)*
                            sguan->current.Final_Gain*sguan->motor.Current_Dir0;
    float I1 = (User_ReadADC_Raw(1) - sguan->current.Current_offset1)*
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

// 处理机械角度，计算编码数值Encoder
static void Sensor_Encoder_Calculate(SguanFOC_System_STRUCT *sguan){
    sguan->encoder.Real_Rad = User_Encoder_ReadRad();
    Transfer_PLL_Loop(&sguan->transfer.PLL,
                    sguan->mode,
                    (sguan->encoder.Real_Rad - sguan->encoder.Pos_offset)*
                    sguan->motor.identify.Encoder_Dir);
    Transfer_LPF_Loop(&sguan->transfer.LPF_encoder,
                    sguan->transfer.PLL.go.OutWe);
    sguan->encoder.Real_Speed = sguan->transfer.LPF_encoder.filter.Output;
    sguan->encoder.Real_Pos = sguan->transfer.PLL.go.OutRe;
    sguan->encoder.Real_Erad = Value_normalize(
                            sguan->transfer.PLL.go.OutRe*
                            sguan->motor.Poles);
    if (sguan->mode >= Voltag_OPEN_MODE){
        sguan->encoder.Real_We = sguan->encoder.Real_Speed*sguan->motor.Poles;
        fast_sin_cos(sguan->encoder.Real_Erad,&sguan->foc.sine,&sguan->foc.cosine);
    }
    else{
        sguan->encoder.Real_We = sguan->foc.Target_Speed*sguan->motor.Poles;
        float angle = Value_Rad_Loop(sguan->foc.Target_Speed*sguan->motor.Poles,PMSM_RUN_T);
        fast_sin_cos(angle,&sguan->foc.sine,&sguan->foc.cosine);
    }
}

// 处理电角度，计算编码器数值HALL
static void Sensor_HALL_Calculate(SguanFOC_System_STRUCT *sguan){
    Transfer_Hall_Loop(&sguan->transfer.Hall,
                    User_Encoder_ReadHall(0), 
                    User_Encoder_ReadHall(0), 
                    User_Encoder_ReadHall(0));
    sguan->encoder.Real_Rad = sguan->transfer.Hall.rad.Output_Rad;
    Transfer_PLL_Loop(&sguan->transfer.PLL, 
                    sguan->mode, 
                    (sguan->encoder.Real_Rad - sguan->encoder.Pos_offset)*
                    sguan->motor.identify.Encoder_Dir);
    Transfer_LPF_Loop(&sguan->transfer.LPF_encoder, 
                    sguan->transfer.PLL.go.OutWe);
    sguan->encoder.Real_Speed = sguan->transfer.LPF_encoder.filter.Output;
    sguan->encoder.Real_Pos = sguan->transfer.PLL.go.OutRe;
    if (sguan->mode >= Voltag_OPEN_MODE){
        sguan->encoder.Real_Erad = Value_normalize(
                                sguan->transfer.PLL.go.OutRe*
                                sguan->motor.Poles);
        sguan->encoder.Real_We = sguan->encoder.Real_Speed*sguan->motor.Poles;
        fast_sin_cos(sguan->encoder.Real_Erad,&sguan->foc.sine,&sguan->foc.cosine);
    }
    else{
        float angle = Value_Rad_Loop(
                    sguan->foc.Target_Speed*sguan->motor.Poles,
                    PMSM_RUN_T);
        fast_sin_cos(angle,&sguan->foc.sine,&sguan->foc.cosine);
    }
}

// 处理电角度，计算编码器数值HFI
static void Sensorless_HFI_Calculate(SguanFOC_System_STRUCT *sguan){

}

// 处理电角度，计算编码器数值SMO
static void Sensorless_SMO_Calculate(SguanFOC_System_STRUCT *sguan){

}

// 处理电角度，计算编码器数值HS
static void Sensorless_HS_Calculate(SguanFOC_System_STRUCT *sguan){

}

// Control电流单环(单闭环)
static void Control_Current_SINGLE(SguanFOC_System_STRUCT *sguan){
    // 0.(控制器数据赋值)如果不在工作状态，赋值为零
    // if (sguan->status < MOTOR_STATUS_IDLE){
    //     sguan->foc.Target_Id = 0.0f;
    //     sguan->foc.Target_Iq = 0.0f;
    //     sguan->current.Real_Id = 0.0f;
    //     sguan->current.Real_Iq = 0.0f;
    // }

    // 1.电流环PI控制器计算
    Transfer_PID_Loop(&sguan->transfer.Current_D,
        sguan->foc.Target_Id,
        sguan->current.Real_Id);
    Transfer_PID_Loop(&sguan->transfer.Current_Q,
        sguan->foc.Target_Iq,
        sguan->current.Real_Iq);
    sguan->foc.Ud_in = sguan->transfer.Current_D.run.Output;
    sguan->foc.Uq_in = sguan->transfer.Current_Q.run.Output;

    // 2.结果输出到Ud和Uq给定(带前馈计算)
    #if CONFIG_CurFF
    sguan->foc.Ud_in += Feedforward_CurrentD(sguan->encoder.Real_We,
                                    sguan->motor.identify.Lq,
                                    sguan->current.Real_Iq);
    sguan->foc.Uq_in += Feedforward_CurrentQ(sguan->encoder.Real_We,
                                    sguan->motor.identify.Ld,
                                    sguan->current.Real_Id,
                                    sguan->motor.identify.Flux);
    #endif // CONFIG_CurFF
}

// Control速度-电流双环(双闭环)
static void Control_VelCur_DOUBLE(SguanFOC_System_STRUCT *sguan){
    static uint8_t Control_Count = 0;
    Control_Count++;
    // 0.(控制器数据赋值)如果不在工作状态，赋值为零
    // if (sguan->status < MOTOR_STATUS_IDLE){
    //     sguan->foc.Target_Speed = 0.0f;
    //     sguan->encoder.Real_Speed = 0.0f;
    // }

    // 1.转速环PI控制器计算
    if (Control_Count >= sguan->transfer.Response){
        CtrlFunc_Tick[Value_set(CONFIG_CtrlVel,PMSM_MAX_Ctrl,0)](
            &sguan->transfer.Velocity,
            sguan->foc.Target_Speed,
            sguan->encoder.Real_Speed);
        sguan->foc.Target_Iq = sguan->transfer.Velocity.run.Output;
        Control_Count = 0;
    }

    // 2.MTPA控制
    #if CONFIG_MTPA
    MTPA_Loop(&sguan->foc.Target_Id,
                sguan->motor.identify.Flux,
                sguan->motor.identify.Ld,
                sguan->current.Real_Iq,
                sguan->current.Real_Iq);
    #endif // CONFIG_MTPA

    // 3.FW弱磁控制
    #if CONFIG_FW
    sguan->foc.Target_Id += FW_Loop(
                &sguan->transfer.FW, 
                sguan->foc.Ud_in, 
                sguan->foc.Uq_in, 
                sguan->transfer.Percentage_fw, 
                sguan->foc.Real_VBUS);
    #endif // CONFIG_FW

    // 4.计算速度环输出补偿(速度前馈量叠加)
    #if CONFIG_VelFF
    sguan->foc.Target_Iq += Feedforward_Velocity(sguan->encoder.Real_Speed,
                                                sguan->transfer.Beta_ff);
    #endif // CONFIG_VelFF

    // 5.补偿Q轴电流(扰动前馈量叠加)
    #if CONFIG_DOB
    Transfer_DOB_Loop(&sguan->transfer.DOB,
        sguan->current.Real_Iq,
        sguan->encoder.Real_Speed);
    sguan->foc.Target_Iq += Feedforward_DOB(sguan->transfer.DOB.smdo.Output_Fd,
                                                sguan->motor.Poles,
                                                sguan->motor.identify.Flux);
    #endif // CONFIG_DOB

    // 6.电流环PI控制器计算
    Transfer_PID_Loop(&sguan->transfer.Current_D,
        sguan->foc.Target_Id,       // 默认D轴励磁Id为0
        sguan->current.Real_Id);
    Transfer_PID_Loop(&sguan->transfer.Current_Q,
        sguan->foc.Target_Iq,
        sguan->current.Real_Iq);
    sguan->foc.Ud_in = sguan->transfer.Current_D.run.Output;
    sguan->foc.Uq_in = sguan->transfer.Current_Q.run.Output;

    // 7.结果输出到Ud和Uq给定(带前馈计算)
    #if CONFIG_CurFF
    sguan->foc.Ud_in += Feedforward_CurrentD(sguan->encoder.Real_We,
                                    sguan->motor.identify.Lq,
                                    sguan->current.Real_Iq);
    sguan->foc.Uq_in += Feedforward_CurrentQ(sguan->encoder.Real_We,
                                    sguan->motor.identify.Ld,
                                    sguan->current.Real_Id,
                                    sguan->motor.identify.Flux);
    #endif // CONFIG_CurFF
}

// Control高性能伺服三环(三闭环)
static void Control_PosVelCur_THREE(SguanFOC_System_STRUCT *sguan){
    static uint8_t Control_Count = 0;
    Control_Count++;
    // 0.(控制器数据赋值)如果不在工作状态，赋值为零
    // if (sguan->status < MOTOR_STATUS_IDLE){
    //     sguan->foc.Target_Pos = 0.0f;
    //     sguan->encoder.Real_Pos = 0.0f;
    // }

    // 1.位置环PD控制器计算
    if (Control_Count >= 
        (sguan->transfer.Response*sguan->transfer.Response)){
        CtrlFunc_Tick[Value_set(CONFIG_CtrlPos,PMSM_MAX_Ctrl,0)](
            &sguan->transfer.Position,
            sguan->foc.Target_Pos,
            sguan->encoder.Real_Pos);
        Control_Count = 0;
    }

    // 2.转速环PI控制器计算
    if (Control_Count % sguan->transfer.Response == 0){
        CtrlFunc_Tick[Value_set(CONFIG_CtrlVel,PMSM_MAX_Ctrl,0)](
            &sguan->transfer.Velocity,
            sguan->transfer.Position.run.Output,
            sguan->encoder.Real_Speed);
        sguan->foc.Target_Iq = sguan->transfer.Velocity.run.Output;
    }

    // 3.MTPA控制
    #if CONFIG_MTPA
    MTPA_Loop(&sguan->foc.Target_Id,
                sguan->motor.identify.Flux,
                sguan->motor.identify.Ld,
                sguan->current.Real_Iq,
                sguan->current.Real_Iq);
    #endif // CONFIG_MTPA

    // 4.FW弱磁控制
    #if CONFIG_FW
    sguan->foc.Target_Id += FW_Loop(
                &sguan->transfer.FW, 
                sguan->foc.Ud_in, 
                sguan->foc.Uq_in, 
                sguan->transfer.Percentage_fw, 
                sguan->foc.Real_VBUS);
    #endif // CONFIG_FW

    // 5.计算速度环输出补偿(速度前馈量叠加)
    #if CONFIG_VelFF
    sguan->foc.Target_Iq += Feedforward_Velocity(sguan->encoder.Real_Speed,
                                                sguan->transfer.Beta_ff);
    #endif // CONFIG_VelFF

    // 6.补偿Q轴电流(扰动前馈量叠加)
    #if CONFIG_DOB
    Transfer_DOB_Loop(&sguan->transfer.DOB,
        sguan->current.Real_Iq,
        sguan->encoder.Real_Speed);
    sguan->foc.Target_Iq += Feedforward_DOB(sguan->transfer.DOB.smdo.Output_Fd,
                                                sguan->motor.Poles,
                                                sguan->motor.identify.Flux);
    #endif // CONFIG_DOB

    // 7.电流环PI控制器计算
    Transfer_PID_Loop(&sguan->transfer.Current_D,
        sguan->foc.Target_Id,       // 默认D轴励磁为0
        sguan->current.Real_Id);
    Transfer_PID_Loop(&sguan->transfer.Current_Q,
        sguan->foc.Target_Iq,
        sguan->current.Real_Iq);
    sguan->foc.Ud_in = sguan->transfer.Current_D.run.Output;
    sguan->foc.Uq_in = sguan->transfer.Current_Q.run.Output;

    // 8.结果输出到Ud和Uq给定(电流前馈量叠加)
    #if CONFIG_CurFF
    sguan->foc.Ud_in += Feedforward_CurrentD(sguan->encoder.Real_We,
                                    sguan->motor.identify.Lq,
                                    sguan->current.Real_Iq);
    sguan->foc.Uq_in += Feedforward_CurrentQ(sguan->encoder.Real_We,
                                    sguan->motor.identify.Ld,
                                    sguan->current.Real_Id,
                                    sguan->motor.identify.Flux);
    #endif // CONFIG_CurFF
}

// SVPWM电机驱动的马鞍波生成
static void PWM_Tick(SguanFOC_System_STRUCT *sguan,
                    float sine,
                    float cosine,
                    float d_duty,
                    float q_duty){
    // 1.反帕克变换
    float U_alpha,U_beta;
    ipark(&U_alpha,&U_beta,d_duty,q_duty,sine,cosine);
    
    // 2.PWM调制量运算
    #if CONFIG_PWM
    SPWM(U_alpha,U_beta,
        &sguan->foc.Du,
        &sguan->foc.Dv,
        &sguan->foc.Dw);          
    #else // CONFIG_PWM
    SVPWM(U_alpha,U_beta,
        &sguan->foc.Du,
        &sguan->foc.Dv,
        &sguan->foc.Dw);
    #endif // CONFIG_PWM

    // 3.提高低速性能(死区补偿叠加量)
    #if CONFIG_DeadZone
    DeadZone_Loop(&sguan->foc.Du,
        &sguan->foc.Dv,
        &sguan->foc.Dw,
        sguan->current.Real_Ia,
        sguan->current.Real_Ib,
        sguan->current.Real_Ic,
        sguan->transfer.Dead_CurMin,
        sguan->transfer.DeadTime);
    #endif // CONFIG_DeadZone

    // 4.三相归一化电压值限幅
    Value_Limit(&sguan->foc.Du,1.0f,0.0f);
    Value_Limit(&sguan->foc.Dv,1.0f,0.0f);
    Value_Limit(&sguan->foc.Dw,1.0f,0.0f);

    // 5.电机占空比输出
    uint32_t Duty_u,Duty_v,Duty_w;
    if (sguan->motor.PWM_Dir == 1){
        Duty_u = (uint32_t)(sguan->foc.Du*sguan->motor.Duty);
        Duty_v = (uint32_t)(sguan->foc.Dv*sguan->motor.Duty);
        Duty_w = (uint32_t)(sguan->foc.Dw*sguan->motor.Duty);
    }
    else if (sguan->motor.PWM_Dir == -1){
        Duty_u = (uint32_t)((1.0f - sguan->foc.Du)*sguan->motor.Duty);
        Duty_v = (uint32_t)((1.0f - sguan->foc.Dv)*sguan->motor.Duty);
        Duty_w = (uint32_t)((1.0f - sguan->foc.Dw)*sguan->motor.Duty);
    }
    if (sguan->motor.Motor_Dir == -1){ // 判断电机方向并修改(原理是AB相序交换)
        uint32_t duty_temp = Duty_u;
        Duty_u = Duty_v;
        Duty_v = duty_temp;
    }
    User_PwmDuty_Set(Duty_u, Duty_v, Duty_w);
}


// =============================== 通用数据层(代码实现) =============================
// Data母线电压和驱动器物理温度数据更新
static void Data_Protection_Loop(SguanFOC_System_STRUCT *sguan){
    // 1.更新母线电压数值
    if (User_VBUS_DataGet() != Value_N_INF){        
        sguan->foc.Real_VBUS = User_VBUS_DataGet();
    }
    else{
        sguan->foc.Real_VBUS = sguan->motor.VBUS;
    }

    // 2.更新驱动器温度数值
    if (User_Temperature_DataGet() != Value_N_INF){        
        sguan->foc.Real_Temp = User_Temperature_DataGet();
    }
}

// Status判断DISABLED状态机的切换
static void Status_Switch_STANDBY(SguanFOC_System_STRUCT *sguan,uint32_t *count){
    sguan->status = MOTOR_STATUS_STANDBY;
    *count = 0;
    // 已失能状态持续“自定义”次控制周期后，自动切换退出已失能状态
}

static void Status_Switch_IDLE(SguanFOC_System_STRUCT *sguan,uint32_t *count){
    sguan->status = MOTOR_STATUS_IDLE;
    *count = 0;
}

// Status判断过压保护状态
static void Status_VBUS_OVERVOLTAGE(SguanFOC_System_STRUCT *sguan,uint32_t *count){
    if (sguan->foc.Real_VBUS > sguan->safe.VBUS_MAX){
        Status_Switch_STANDBY(sguan,count);
    }
}

// Status判断欠压保护状态
static void Status_VBUS_UNDERVOLTAGE(SguanFOC_System_STRUCT *sguan,uint32_t *count){
    if (sguan->foc.Real_VBUS < sguan->safe.VBUS_MIM){
        Status_Switch_STANDBY(sguan,count);
    }
}

// Status判断过温保护状态
static void Status_Temp_OVERTEMPERATURE(SguanFOC_System_STRUCT *sguan,uint32_t *count){
    if (sguan->foc.Real_Temp > sguan->safe.Temp_MAX){
        Status_Switch_STANDBY(sguan,count);
    }
}

// Status判断低温保护状态
static void Status_Temp_UNDERTEMPERATURE(SguanFOC_System_STRUCT *sguan,uint32_t *count){
    if (sguan->foc.Real_Temp < sguan->safe.Temp_MIN){
        Status_Switch_STANDBY(sguan,count);
    }
}

// Status判断过流保护状态
static void Status_Current_OVERCURRENT(SguanFOC_System_STRUCT *sguan,uint32_t *count){
    if ((sguan->current.Real_Id > sguan->safe.Dcur_MAX) || 
        ((sguan->current.Real_Iq > sguan->safe.Qcur_MAX))){
        Status_Switch_STANDBY(sguan,count);
    }
}

// Status判断并切换状态机
static void Status_Switch_Loop(SguanFOC_System_STRUCT *sguan){
    if (sguan->status < MOTOR_STATUS_IDLE){
        return;
    }
    
    // ====== 安全状态(状态) ======
    if ((sguan->status != MOTOR_STATUS_EMERGENCY_STOP) && 
        EMERGENCY_STOP_Signal()){
        sguan->status = MOTOR_STATUS_EMERGENCY_STOP;
    }
    if ((sguan->status != MOTOR_STATUS_DISABLED) && 
        DISABLED_Signal()){
        sguan->status = MOTOR_STATUS_DISABLED;
    }

    // ====== 初始化与运行状态(状态) ======
    if ((sguan->status != MOTOR_STATUS_STANDBY) && 
        STANDBY_Signal()){ // [重要]接收到待机信号后，解除锁定状态...进入待机状态
        sguan->status = MOTOR_STATUS_STANDBY;
    }
    if ((sguan->status == MOTOR_STATUS_EMERGENCY_STOP) || 
        (sguan->status == MOTOR_STATUS_DISABLED) || (sguan->status < 4)){
        return; // 紧急停止和失能状态优先级最高, 直接返回不执行后续状态判断
    }
    if ((sguan->status != MOTOR_STATUS_UNINITIALIZED) && 
        UNINITIALIZED_Signal()){ // [重要]接收到开始信号后，切换到未初始化状态...准备初始化
        sguan->status = MOTOR_STATUS_UNINITIALIZED;
    }

    // ====== 硬件相关错误(状态) ======
    // 1.电机母线电压VBUS状态机
    if (User_VBUS_DataGet() != Value_N_INF){
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
    if (User_Temperature_DataGet() != Value_N_INF){        
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
    if ((sguan->status != MOTOR_STATUS_ENCODER_ERROR) && 
        ENCODER_ERROR_Signal()){
        sguan->status = MOTOR_STATUS_ENCODER_ERROR;
    }
    // 5.传感器错误
    if ((sguan->status != MOTOR_STATUS_SENSOR_ERROR) && 
        SENSOR_ERROR_Signal()){
        sguan->status = MOTOR_STATUS_SENSOR_ERROR;
    }
    if (sguan->status >= 14 && sguan->status < 21){
        return; // 硬件相关错误状态优先级高于运行状态, 直接返回不执行后续状态判断
    }

    // ====== 运行状态(当前反馈) ======
    // 1.力矩模式检测
    if ((sguan->mode == Current_SINGLE_MODE) && 
        (sguan->status != MOTOR_STATUS_TORQUE_CONTROL) && 
        (sguan->current.Real_Iq > (sguan->foc.Target_Iq - sguan->safe.Current_limit)) && 
        (sguan->current.Real_Iq < (sguan->foc.Target_Iq + sguan->safe.Current_limit))){
        sguan->status = MOTOR_STATUS_TORQUE_CONTROL;
    }
    if ((sguan->mode == Current_SINGLE_MODE) && 
        (sguan->status != MOTOR_STATUS_TORQUE_INCREASING) && 
        (sguan->current.Real_Iq < (sguan->foc.Target_Iq + sguan->safe.Current_limit))){
        sguan->status = MOTOR_STATUS_TORQUE_INCREASING;
    }
    if ((sguan->mode == Current_SINGLE_MODE) && 
        (sguan->status != MOTOR_STATUS_TORQUE_DECREASING) && 
        (sguan->current.Real_Iq > (sguan->foc.Target_Iq - sguan->safe.Current_limit))){
        sguan->status = MOTOR_STATUS_TORQUE_DECREASING;
    }
    // 2.速度模式检测
    if (((sguan->mode == VelCur_DOUBLE_MODE) || 
        (sguan->mode == Sensor_Hall_MODE) || 
        (sguan->mode == Sensorless_HFI_MODE) || 
        (sguan->mode == Sensorless_SMO_MODE) || 
        (sguan->mode == Sensorless_HS_MODE)) && 
        (sguan->status != MOTOR_STATUS_CONST_SPEED) && 
        (sguan->encoder.Real_Speed > sguan->foc.Target_Speed - sguan->safe.Speed_limit) && 
        (sguan->encoder.Real_Speed < sguan->foc.Target_Speed + sguan->safe.Speed_limit)){
        sguan->status = MOTOR_STATUS_CONST_SPEED;
    }
    if (((sguan->mode == VelCur_DOUBLE_MODE) || 
        (sguan->mode == Sensor_Hall_MODE) || 
        (sguan->mode == Sensorless_HFI_MODE) || 
        (sguan->mode == Sensorless_SMO_MODE) || 
        (sguan->mode == Sensorless_HS_MODE)) && 
        (sguan->status != MOTOR_STATUS_ACCELERATING) && 
        (sguan->encoder.Real_Speed < sguan->foc.Target_Speed + sguan->safe.Speed_limit)){
        sguan->status = MOTOR_STATUS_ACCELERATING;
    }
    if (((sguan->mode == VelCur_DOUBLE_MODE) || 
        (sguan->mode == Sensor_Hall_MODE) || 
        (sguan->mode == Sensorless_HFI_MODE) || 
        (sguan->mode == Sensorless_SMO_MODE) || 
        (sguan->mode == Sensorless_HS_MODE)) && 
        (sguan->status != MOTOR_STATUS_DECELERATING) && 
        (sguan->encoder.Real_Speed > sguan->foc.Target_Speed - sguan->safe.Speed_limit)){
        sguan->status = MOTOR_STATUS_DECELERATING;
    }
    // 3.位置模式检测
    if ((sguan->mode == PosVelCur_THREE_MODE) && 
        (sguan->status != MOTOR_STATUS_POSITION_HOLD) && 
        (sguan->encoder.Real_Pos > sguan->foc.Target_Pos - sguan->safe.Position_limit) && 
        (sguan->encoder.Real_Pos < sguan->foc.Target_Pos + sguan->safe.Position_limit)){
        sguan->status = MOTOR_STATUS_POSITION_HOLD;
    }
    if ((sguan->mode == PosVelCur_THREE_MODE) && 
        (sguan->status != MOTOR_STATUS_POSITION_INCREASING) && 
        (sguan->encoder.Real_Pos < sguan->foc.Target_Pos + sguan->safe.Position_limit)){
        sguan->status = MOTOR_STATUS_POSITION_INCREASING;
    }
    if ((sguan->mode == PosVelCur_THREE_MODE) && 
        (sguan->status != MOTOR_STATUS_POSITION_DECREASING) && 
        (sguan->encoder.Real_Pos > sguan->foc.Target_Pos - sguan->safe.Position_limit)){
        sguan->status = MOTOR_STATUS_POSITION_DECREASING;
    }
}

// Status定时器中断调用的状态机运行函数
static void Status_RUN_Loop(SguanFOC_System_STRUCT *sguan){
    // 1.状态机失能DISABLED安全状态处理
    static uint32_t count = 0;
    count++;
    if (sguan->status == MOTOR_STATUS_DISABLED){
        if (count > sguan->safe.DISABLED_watchdog_limit){
            Status_Switch_STANDBY(sguan,&count);
        }
    }
    // 2.电机电压异常_错误处理
    if (sguan->status == MOTOR_STATUS_OVERVOLTAGE){
        if (count % sguan->safe.VBUS_watchdog_limit == 0){
            Status_VBUS_OVERVOLTAGE(sguan,&count);
        }
        if (count > sguan->safe.VBUS_watchdog_limit*10){ 
            Status_Switch_IDLE(sguan,&count);
            // 持续自定义的周期后，自动切换退出过压保护，并进入STANDBY
        }
    }
    if (sguan->status == MOTOR_STATUS_UNDERVOLTAGE){
        if (count % sguan->safe.VBUS_watchdog_limit == 0){
            Status_VBUS_UNDERVOLTAGE(sguan,&count);
        }
        if (count > sguan->safe.VBUS_watchdog_limit*10){ 
            Status_Switch_IDLE(sguan,&count);
            // 持续自定义的周期后，自动切换退出过压保护，并进入STANDBY
        }
    }
    // 3.电机驱动器温度异常_错误处理
    if (sguan->status == MOTOR_STATUS_OVERTEMPERATURE){
        if (count % sguan->safe.Temp_watchdog_limit == 0){
            Status_Temp_OVERTEMPERATURE(sguan,&count);
        }
        if (count > sguan->safe.Temp_watchdog_limit*10){ 
            Status_Switch_IDLE(sguan,&count);
            // 持续自定义的周期后，自动切换退出过压保护，并进入STANDBY
        }
    }
    if (sguan->status == MOTOR_STATUS_UNDERTEMPERATURE){
        if (count % sguan->safe.Temp_watchdog_limit == 0){
            Status_Temp_UNDERTEMPERATURE(sguan,&count);
        }
        if (count > sguan->safe.Temp_watchdog_limit*10){ 
            Status_Switch_IDLE(sguan,&count);
            // 持续自定义的周期后，自动切换退出过压保护，并进入STANDBY
        }
    }
    // 4.电机过流保护OVERCURRENT错误处理
    if (sguan->status == MOTOR_STATUS_OVERCURRENT){
        if (count % sguan->safe.DQcur_watchdog_limit == 0){
            Status_Current_OVERCURRENT(sguan,&count);
        }
        if (count > sguan->safe.DQcur_watchdog_limit*10){ 
            Status_Switch_IDLE(sguan,&count);
            // 持续自定义的周期后，自动切换退出过压保护，并进入STANDBY
        }
    }
    MotorStatus_Loop(&Sguan.status);
}


// Printf电机调试信息发送
#if CONFIG_Debug
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
#else // CONFIG_Debug
// Printf电机数据正常发送
static void Printf_Normal_Loop(SguanFOC_System_STRUCT *sguan){
    // 发送数据到上位机
    Printf_TX_Loop(&Sguan.txdata);
}
#endif // CONFIG_Debug


// Set各种控制电机回零的设置
static void Positioning_Set(SguanFOC_System_STRUCT *sguan,float Ud,float Uq){
    // 电机零位Ud,Uq设计
    float duty_d = Ud/sguan->foc.Real_VBUS;
    float duty_q = Uq/sguan->foc.Real_VBUS;
    PWM_Tick(sguan,
        0.0f,
        1.0f,
        duty_d,
        duty_q);
}

// Sguan_Calculate_Loop有传感器角度和电流
static void Sguan_Calculate_Loop(SguanFOC_System_STRUCT *sguan){
    // 1.有传感器电机角度和角速度计算
    Encoder_Tick[Value_set(sguan->mode,
        Sensorless_HS_MODE,0)](sguan);
    
    // 2.电机相线和各轴电流计算
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
    Transfer_LPF_Loop(&sguan->transfer.LPF_D,
                    sguan->current.Real_Id);
    Transfer_LPF_Loop(&sguan->transfer.LPF_Q,
                    sguan->current.Real_Iq);
    sguan->current.Real_Id = sguan->transfer.LPF_D.filter.Output;
    sguan->current.Real_Iq = sguan->transfer.LPF_Q.filter.Output;
}

// Sguan_GeneratePWM_Loop计算控制量并执行电机控制
static void Sguan_GeneratePWM_Loop(SguanFOC_System_STRUCT *sguan){
    // 运算控制器输出
    Control_Tick[Value_set(sguan->mode,
        Sensorless_HS_MODE,0)](sguan);
    
    // 计算PWM调制量并执行
    PWM_Tick(sguan,
        sguan->foc.sine,        // sin正弦值给定
        sguan->foc.cosine,      // cos余弦值给定
        (sguan->foc.Ud_in/sguan->foc.Real_VBUS),
        (sguan->foc.Uq_in/sguan->foc.Real_VBUS));
}

// Sguan系统开始的核心文件，主任务初始化函数
static void Sguan_ReInit_Loop(SguanFOC_System_STRUCT *sguan){
    if (sguan->status == MOTOR_STATUS_UNINITIALIZED){
        // 用户自定义的电机参数和控制系统参数
        User_InitialInit();
        User_MotorSet();
        User_ParameterSet();
        // 各种控制系统的初始化
        sguan->status = MOTOR_STATUS_INITIALIZING;
        Transfer_Init(sguan);
        Identify_Init(&sguan->motor.identify);
        Printf_TX_Init(&sguan->txdata);
        // 读取电流偏置
        Offset_CurrentRead(sguan);
        // if ((sguan->mode >= Voltag_OPEN_MODE) && (sguan->mode <= Sensor_Hall_MODE)){            
            //电机回零操作
            Positioning_Set(sguan,0.3f*sguan->foc.Real_VBUS,0.0f);
            User_Delay(1200);
            // 读取角度偏置
            Offset_EncoderRead(sguan);
            // 电机失能并进入正常工作状态
            Positioning_Set(sguan,0.0f,0.0f);
            User_Delay(800);
        // }
    }
    if ((sguan->status == MOTOR_STATUS_INITIALIZING) || 
        (sguan->status == MOTOR_STATUS_CALIBRATING)){
        sguan->status = MOTOR_STATUS_CALIBRATING;
        // 电机校准，参数辨识，暂时还未书写
        // if (Identify_Loop(&sguan->motor.identify)){
            //正常工作中(状态机运行)
            sguan->status = MOTOR_STATUS_IDLE;
        // }
    }
}

/**
 * @description: SguanFOC核心文件，定时中断服务函数(高频率电机载波)
 * @reminder: 10Khz或者更高定时中断中调用，任务优先级“最高”
 * @return {*}
 */
void SguanFOC_High_Loop(void){
    static uint8_t PWM_watchdog_counter = 0;
    Sguan.flag.in_PWM_Calc_ISR = 1;
    if (!Sguan.flag.PWM_Calc){
        Sguan.flag.PWM_Calc = 1;
        PWM_watchdog_counter = 0;

        // (如果计算超时，会更新错误状态并停用此线程)
        // 如果在初始化完成，进入函数
        if ((Sguan.status >= MOTOR_STATUS_CALIBRATING) && 
            (Sguan.status < MOTOR_STATUS_ENCODER_ERROR)){          
            // 计算编码器和电流
            Sguan_Calculate_Loop(&Sguan);
            // 运算PID并执行SVPWM
            Sguan_GeneratePWM_Loop(&Sguan);
        }
        if ((Sguan.status >= MOTOR_STATUS_ENCODER_ERROR) || 
            (Sguan.status == 18)){
            static uint8_t status = 0xFF;
            if (status != Sguan.status){
                // 电压给定归零
                Sguan.foc.Ud_in = 0.0f;
                Sguan.foc.Uq_in = 0.0f;
                // 偏置数值归零
                Sguan.encoder.Pos_offset = 0.0f;
                Sguan.current.Current_offset0 = 0.0f;
                Sguan.current.Current_offset1 = 0.0f;
                // 清零Target数值
                Sguan.foc.Target_Id = 0.0f;
                Sguan.foc.Target_Iq = 0.0f;
                Sguan.foc.Target_Speed = 0.0f;
                Sguan.foc.Target_Pos = 0.0f;
                PWM_Tick(&Sguan,      // 清零电机状态
                    Sguan.foc.sine,
                    Sguan.foc.cosine,
                    (Sguan.foc.Ud_in/Sguan.foc.Real_VBUS),
                    (Sguan.foc.Uq_in/Sguan.foc.Real_VBUS));
                status = Sguan.status;
            }
        }

        #if CONFIG_Debug
        Printf_Debug_Loop(&Sguan);
        #endif // CONFIG_Debug
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
void SguanFOC_Low_Loop(void){
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
void SguanFOC_Printf_Loop(uint8_t *data, uint16_t length){
    // 微控制器接收来自上位机的消息
    // 解析数据的格式like：AO=16.8?
    Printf_RX_Loop(data,length);
}

/**
 * @description: SguanFOC核心文件，主循环服务函数(主循环TXdata数据更新)
 * @reminder: 主循环函数调用，任务优先级“最低”
 * @return {*}
 */
void SguanFOC_main_Loop(void){
    Sguan_ReInit_Loop(&Sguan);
    #if !CONFIG_Debug
    if ((Sguan.status >= MOTOR_STATUS_IDLE) && 
        (Sguan.status < MOTOR_STATUS_ENCODER_ERROR)){
        Printf_Normal_Loop(&Sguan);
    }
    #endif // CONFIG_Debug
}


/*
 * @Author: 星必尘Sguan
 * @GitHub: https://github.com/Sguan-ZhouQing
 * @Date: 2026-01-26 22:38:34
 * @LastEditors: 星必尘Sguan|3464647102@qq.com
 * @LastEditTime: 2026-03-20 22:58:58
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

// 电机控制核心结构体设计
SguanFOC_System_STRUCT Sguan = {0};


// =============================== float 版本代码(仅声明) =============================
#if !CONFIG_Q31
/**
 * @description: 1.Transfer传递函数的离散化运算，采用双线性变换
 * @param {PID_STRUCT} *pid (控制)PID闭环控制系统运算
 * @param {LADRC_STRUCT} *ladrc (控制)LADRC线自抗扰运算
 * @param {INTERNALMODEL_STRUCT} *im (控制)IMC内模控制
 * @param {BPF_STRUCT} *bpf (滤波)BPF二阶巴特沃斯低通滤波
 * @param {PLL_STRUCT} *pll (估算)PLL速度跟踪锁相环
 * @return {*}
 */
#if !CONFIG_PI
static void Transfer_STA_Loop(STA_STRUCT *sta,float Ref,float Fbk);
#endif // CONFIG_PI
static void Transfer_PID_Loop(PID_STRUCT *pid,float Ref,float Fbk);
static void Transfer_LPF_Loop(LPF_STRUCT *lpf,float input);
static void Transfer_PLL_Loop(PLL_STRUCT *pll,uint8_t mode,float input_Rad);
/**
 * @description: 2.Offset内部静态函数声明
 * @param {SguanFOC_System_STRUCT} *sguan
 * @return {*}
 */
static void Offset_EncoderRead(SguanFOC_System_STRUCT *sguan);
static void Offset_CurrentRead(SguanFOC_System_STRUCT *sguan);
/**
 * @description: 3.Current内部静态函数声明
 * @param {SguanFOC_System_STRUCT} *sguan
 * @return {*}
 */
static void Current_ReadIabc(SguanFOC_System_STRUCT *sguan);
/**
 * @description: 4.Sguan_Calculate_Loop内部静态函数声明
 * @param {SguanFOC_System_STRUCT} *sguan
 * @return {*}
 */
static void Sguan_Calculate_Loop(SguanFOC_System_STRUCT *sguan);
/**
 * @description: 5.Control运算及其模式切换
 * @param {SguanFOC_System_STRUCT} *sguan
 * @return {*}
 */
static void Control_Velocity_OPEN(SguanFOC_System_STRUCT *sguan);
static void Control_Current_SINGLE(SguanFOC_System_STRUCT *sguan);
static void Control_VelCur_DOUBLE(SguanFOC_System_STRUCT *sguan);
static void Control_PosVelCur_THREE(SguanFOC_System_STRUCT *sguan);
static void (*const Control_Tick[])(SguanFOC_System_STRUCT*)={
    /*这里注意“枚举变量”的边界, Control_Tick[sguan->mode](sguan)使用*/
    Control_Velocity_OPEN,      // Velocity_OPEN_MODE  = 0
    Control_Current_SINGLE,     // Current_SINGLE_MODE = 1
    Control_VelCur_DOUBLE,      // VelCur_DOUBLE_MODE  = 2
    Control_PosVelCur_THREE     // PosVelCur_THREE_MODE= 3
};
/**
 * @description: 6.SVPWM电机驱动的马鞍波生成
 * @param {SguanFOC_System_STRUCT} *sguan
 * @param {float} d
 * @param {float} q
 * @return {*}
 */
static void SVPWM_Tick(SguanFOC_System_STRUCT *sguan,
                    float sine,
                    float cosine,
                    float d_set,
                    float q_set);
/**
 * @description: 7.Sguan_GeneratePWM_Loop定时计算并执行
 * @param {SguanFOC_System_STRUCT} *sguan
 * @return {*}
 */
static void Sguan_GeneratePWM_Loop(SguanFOC_System_STRUCT *sguan);
#endif // CONFIG_Q31
/**
 * @description: 8.Data母线电压和驱动器物理温度数据更新
 * @param {SguanFOC_System_STRUCT} *sguan
 * @return {*}
 */
static void Data_Protection_Loop(SguanFOC_System_STRUCT *sguan);
/**
 * @description: 9.Status判断并切换状态机
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
 * @description: 10.Printf数据发送Debug和正常模式
 * @param {SguanFOC_System_STRUCT} *sguan
 * @return {*}
 */
#if Open_Printf_Debug
static void Printf_Debug_Loop(SguanFOC_System_STRUCT *sguan);
#else // Open_Printf_Debug
static void Printf_Normal_Loop(SguanFOC_System_STRUCT *sguan);
#endif // Open_Printf_Debug
/**
 * @description: 11.Sguan...Set各种控制电机参数的设置
 * @param {SguanFOC_System_STRUCT} *sguan
 * @return {*}
 */
static void Sguan_SystemT_Set(SguanFOC_System_STRUCT *sguan);
static void Sguan_Positioning_Set(SguanFOC_System_STRUCT *sguan,float Ud,float Uq);
/**
 * @description: 12.Sguan...Init各种控制系统的初始化
 * @param {SguanFOC_System_STRUCT} *sguan
 * @return {*}
 */
static void Sguan_LPF_Init(SguanFOC_System_STRUCT *sguan);
static void Sguan_Control_Init(SguanFOC_System_STRUCT *sguan);
static void Sguan_PLL_Init(SguanFOC_System_STRUCT *sguan);
/**
 * @description: 13.Sguan...Tick电机总初始化与校准
 * @param {SguanFOC_System_STRUCT} *sguan
 * @return {*}
 */
static void Sguan_Start_Tick(void);


// =============================== Q31 版本代码(仅声明) =============================
#if CONFIG_Q31
// 0.初始化浮点转Q31和实时浮点转定点
static void IQmath_Calculate_Init_Tick_q31(SguanFOC_System_STRUCT *sguan);
static void IQmath_Calculate_High_Tick_q31(SguanFOC_System_STRUCT *sguan);
// 1.Transfer传递函数的离散化运算，采用双线性变换
#if CONFIG_PI
static void Transfer_STA_Loop_q31(STA_STRUCT_q31 *sta,Q31_t Ref,Q31_t Fbk);
#endif // CONFIG_PI
static void Transfer_PID_Loop_q31(PID_STRUCT_q31 *pid,Q31_t Ref,Q31_t Fbk);
static void Transfer_LPF_Loop_q31(LPF_STRUCT_q31 *lpf,Q31_t input);
static void Transfer_PLL_Loop_q31(PLL_STRUCT_q31 *pll,uint8_t mode,Q31_t input_Rad);
// 2.Offset内部静态函数声明
static void Offset_EncoderRead_q31(SguanFOC_System_STRUCT *sguan);
static void Offset_CurrentRead_q31(SguanFOC_System_STRUCT *sguan);
// 3.Current内部静态函数声明
static void Current_ReadIabc_q31(SguanFOC_System_STRUCT *sguan);
// 4.Sguan_Calculate_Loop内部静态函数声明
static void Sguan_Calculate_Loop_q31(SguanFOC_System_STRUCT *sguan);
// 5.Control运算及其模式切换
static void Control_Velocity_OPEN_q31(SguanFOC_System_STRUCT *sguan);
static void Control_Current_SINGLE_q31(SguanFOC_System_STRUCT *sguan);
static void Control_VelCur_DOUBLE_q31(SguanFOC_System_STRUCT *sguan);
static void Control_PosVelCur_THREE_q31(SguanFOC_System_STRUCT *sguan);
static void (*const Control_Tick_q31[])(SguanFOC_System_STRUCT*)={
    Control_Velocity_OPEN_q31,
    Control_Current_SINGLE_q31,
    Control_VelCur_DOUBLE_q31,
    Control_PosVelCur_THREE_q31
};
// 6.SVPWM电机驱动的马鞍波生成
static void SVPWM_Tick_q31(SguanFOC_System_STRUCT *sguan,
                    Q31_t sine,
                    Q31_t cosine,
                    Q31_t d_set,
                    Q31_t q_set);
// 7.Sguan_GeneratePWM_Loop定时计算并执行
static void Sguan_GeneratePWM_Loop_q31(SguanFOC_System_STRUCT *sguan);
#endif // CONFIG_Q31


// =============================== float 版本代码(代码实现) =============================
#if !CONFIG_Q31
// Transfer运算_STA二阶滑膜控制
#if !CONFIG_PI
static void Transfer_STA_Loop(STA_STRUCT *sta,float Ref,float Fbk){
    sta->sta.Ref = Ref;
    sta->sta.Fbk = Fbk;
    STA_Loop(sta);
    // 输出sta->sta.Output;
}
#endif // CONFIG_PI

// Transfer运算_PID运算
static void Transfer_PID_Loop(PID_STRUCT *pid,float Ref,float Fbk){
    pid->run.Ref = Ref;
    pid->run.Fbk = Fbk;
    PID_Loop(pid);
    // 输出pid->run.Output;
}

// Transfer运算_二阶巴特沃斯低通滤波器
static void Transfer_LPF_Loop(LPF_STRUCT *lpf,float input){
    lpf->filter.Input = input;
    LPF_Loop(lpf);
    // 输出bpf->filter.Output;
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

// Current读取当前的电流值并更新3相电流(已滤波)
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

// Calculate有传感器角度和电流
static void Sguan_Calculate_Loop(SguanFOC_System_STRUCT *sguan){
    // 1.有传感器电机角度和角速度计算
    sguan->encoder.Real_Rad = User_Encoder_ReadRad();
    Transfer_PLL_Loop(&sguan->encoder.pll,
                    sguan->mode,
                    (sguan->encoder.Real_Rad - sguan->encoder.Pos_offset)*
                    sguan->motor.Encoder_Dir);
    Transfer_LPF_Loop(&sguan->lpf.Encoder,
                    sguan->encoder.pll.go.OutWe);
    sguan->encoder.Real_Speed = sguan->lpf.Encoder.filter.Output;
    sguan->encoder.Real_Pos = sguan->encoder.pll.go.OutRe;
    sguan->encoder.Real_Erad = Value_normalize(
                            sguan->encoder.pll.go.OutRe*
                            sguan->motor.Poles);
    sguan->encoder.Real_Espeed = sguan->encoder.Real_Speed*sguan->motor.Poles;
    fast_sin_cos(sguan->encoder.Real_Erad,&sguan->foc.sine,&sguan->foc.cosine);
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
    Transfer_LPF_Loop(&sguan->lpf.CurrentD,
                    sguan->current.Real_Id);
    Transfer_LPF_Loop(&sguan->lpf.CurrentQ,
                    sguan->current.Real_Iq);
    sguan->current.Real_Id = sguan->lpf.CurrentD.filter.Output;
    sguan->current.Real_Iq = sguan->lpf.CurrentQ.filter.Output;
}

// Control速度开环(用于直接控制Uq_in,用于电机测试)
static void Control_Velocity_OPEN(SguanFOC_System_STRUCT *sguan){
    // 如果不在正常工作状态，则赋值为零
    if (sguan->status < 4){
        sguan->foc.Ud_in = 0.0f;
        sguan->foc.Uq_in = 0.0f;
    }
}

// Control电流单环(单闭环)
static void Control_Current_SINGLE(SguanFOC_System_STRUCT *sguan){
    // 0.(控制器数据赋值)如果不在工作状态，赋值为零
    if (sguan->status < 4){
        sguan->foc.Target_Id = 0.0f;
        sguan->foc.Target_Iq = 0.0f;
        sguan->current.Real_Id = 0.0f;
        sguan->current.Real_Iq = 0.0f;
    }

    // 1.前馈计算
    float Ud_ff = -sguan->encoder.Real_Espeed*sguan->identify.Lq*sguan->current.Real_Iq; 
    float Uq_ff = sguan->encoder.Real_Espeed*sguan->identify.Ld*sguan->current.Real_Id + 
                    sguan->encoder.Real_Espeed*sguan->identify.Flux;

    // 2.弱磁控制
    #if Open_FW_Calculate
    FW_MTPA_Loop(&sguan->foc.Target_Id,
                sguan->identify.Flux,
                sguan->identify.Ld,
                sguan->current.Real_Iq,
                sguan->current.Real_Iq);
    #endif // Open_FW_Calculate

    // 3.电流环PI控制器计算
    Transfer_PID_Loop(&sguan->control.Current_D,
        sguan->foc.Target_Id,
        sguan->current.Real_Id);
    Transfer_PID_Loop(&sguan->control.Current_Q,
        sguan->foc.Target_Iq,
        sguan->current.Real_Iq);

    // 4.结果输出到Ud和Uq给定
    sguan->foc.Ud_in = sguan->control.Current_D.run.Output + Ud_ff;
    sguan->foc.Uq_in = sguan->control.Current_Q.run.Output + Uq_ff;
}

// Control速度-电流双环(双闭环)
static void Control_VelCur_DOUBLE(SguanFOC_System_STRUCT *sguan){
    static uint8_t Control_Count = 0;
    Control_Count++;
    // 0.(控制器数据赋值)如果不在工作状态，赋值为零
    if (sguan->status < 4){
        sguan->foc.Target_Speed = 0.0f;
        sguan->encoder.Real_Speed = 0.0f;
    }

    #if Open_PI_Control
    // 1.转速环PI控制器计算
    if (Control_Count >= sguan->control.Response){
        Transfer_PID_Loop(&sguan->control.Velocity,
            sguan->foc.Target_Speed,
            sguan->encoder.Real_Speed);
        Control_Count = 0;
    }

    // 2.前馈计算
    float Ud_ff = -sguan->encoder.Real_Espeed*sguan->identify.Lq*sguan->current.Real_Iq; 
    float Uq_ff = sguan->encoder.Real_Espeed*sguan->identify.Ld*sguan->current.Real_Id + 
                    sguan->encoder.Real_Espeed*sguan->identify.Flux;

    // 3.弱磁控制
    #if Open_FW_Calculate
    FW_MTPA_Loop(&sguan->foc.Target_Id,
                sguan->identify.Flux,
                sguan->identify.Ld,
                sguan->current.Real_Iq,
                sguan->current.Real_Iq);
    #endif // Open_FW_Calculate

    // 4.电流环PI控制器计算
    Transfer_PID_Loop(&sguan->control.Current_D,
        sguan->foc.Target_Id,       // 默认D轴励磁Id为0
        sguan->current.Real_Id);
    Transfer_PID_Loop(&sguan->control.Current_Q,
        sguan->control.Velocity.run.Output,
        sguan->current.Real_Iq);

    // 5.结果输出到Ud和Uq给定
    sguan->foc.Ud_in = sguan->control.Current_D.run.Output + Ud_ff;
    sguan->foc.Uq_in = sguan->control.Current_Q.run.Output + Uq_ff;
    #else // Open_PI_Control
    // 1.转速环STA二阶滑膜控制计算
    if (Control_Count >= sguan->control.Response){
        Transfer_STA_Loop(&sguan->control.Speed,
                        sguan->foc.Target_Speed,
                        sguan->encoder.Real_Speed);
        Control_Count = 0;
    }

    // 2.前馈计算
    float Ud_ff = -sguan->encoder.Real_Espeed*sguan->identify.Lq*sguan->current.Real_Iq; 
    float Uq_ff = sguan->encoder.Real_Espeed*sguan->identify.Ld*sguan->current.Real_Id + 
                    sguan->encoder.Real_Espeed*sguan->identify.Flux;

    // 3.弱磁控制
    #if Open_FW_Calculate
    FW_MTPA_Loop(&sguan->foc.Target_Id,
                sguan->identify.Flux,
                sguan->identify.Ld,
                sguan->current.Real_Iq,
                sguan->current.Real_Iq);
    #endif // Open_FW_Calculate

    // 4.电流环PI控制器计算
    Transfer_PID_Loop(&sguan->control.Current_D,
        sguan->foc.Target_Id,
        sguan->current.Real_Id);
    Transfer_PID_Loop(&sguan->control.Current_Q,
        sguan->control.Speed.sta.output,
        sguan->current.Real_Iq);

    // 5.结果输出到Ud和Uq给定
    sguan->foc.Ud_in = sguan->control.Current_D.run.Output + Ud_ff;
    sguan->foc.Uq_in = sguan->control.Current_Q.run.Output + Uq_ff;
    #endif // Open_PI_Control
}

// Control高性能伺服三环(三闭环)
static void Control_PosVelCur_THREE(SguanFOC_System_STRUCT *sguan){
    static uint8_t Control_Count = 0;
    Control_Count++;
    // 0.(控制器数据赋值)如果不在工作状态，赋值为零
    if (sguan->status < 4){
        sguan->foc.Target_Pos = 0.0f;
        sguan->encoder.Real_Pos = 0.0f;
    }

    #if Open_PI_Control
    // 1.位置环PD控制器计算
    if (Control_Count >= sguan->control.Response*sguan->control.Response){
        Transfer_PID_Loop(&sguan->control.Position,
            sguan->foc.Target_Pos,
            sguan->encoder.Real_Pos);
        Control_Count = 0;
    }

    // 2.转速环PI控制器计算
    if (Control_Count % sguan->control.Response == 0){
        Transfer_PID_Loop(&sguan->control.Velocity,
            sguan->control.Position.run.Output,
            sguan->encoder.Real_Speed);
    }

    // 3.前馈计算
    float Ud_ff = -sguan->encoder.Real_Espeed*sguan->identify.Lq*sguan->current.Real_Iq; 
    float Uq_ff = sguan->encoder.Real_Espeed*sguan->identify.Ld*sguan->current.Real_Id + 
                    sguan->encoder.Real_Espeed*sguan->identify.Flux;

    // 4.弱磁控制
    #if Open_FW_Calculate
    FW_MTPA_Loop(&sguan->foc.Target_Id,
                sguan->identify.Flux,
                sguan->identify.Ld,
                sguan->current.Real_Iq,
                sguan->current.Real_Iq);
    #endif // Open_FW_Calculate

    // 5.电流环PI控制器计算
    Transfer_PID_Loop(&sguan->control.Current_D,
        sguan->foc.Target_Id,       // 默认D轴励磁为0
        sguan->current.Real_Id);
    Transfer_PID_Loop(&sguan->control.Current_Q,
        sguan->control.Velocity.run.Output,
        sguan->current.Real_Iq);

    // 6.结果输出到Ud和Uq给定
    sguan->foc.Ud_in = sguan->control.Current_D.run.Output + Ud_ff;
    sguan->foc.Uq_in = sguan->control.Current_Q.run.Output + Uq_ff;
    #else // Open_PI_Control
    // 1.位置环PD控制器计算
    if (Control_Count >= sguan->control.Response*sguan->control.Response){
        Transfer_PID_Loop(&sguan->control.Position,
            sguan->foc.Target_Pos,
            sguan->encoder.Real_Pos);
        Control_Count = 0;
    }

    // 2.转速环STA二阶滑膜控制计算
    if (Control_Count % sguan->control.Response == 0){
        Transfer_STA_Loop(&sguan->control.Speed,
                        sguan->control.Position.run.Output,
                        sguan->encoder.Real_Speed);
    }

    // 3.前馈计算
    float Ud_ff = -sguan->encoder.Real_Espeed*sguan->identify.Lq*sguan->current.Real_Iq; 
    float Uq_ff = sguan->encoder.Real_Espeed*sguan->identify.Ld*sguan->current.Real_Id + 
                    sguan->encoder.Real_Espeed*sguan->identify.Flux;

    // 4.弱磁控制
    #if Open_FW_Calculate
    FW_MTPA_Loop(&sguan->foc.Target_Id,
                sguan->identify.Flux,
                sguan->identify.Ld,
                sguan->current.Real_Iq,
                sguan->current.Real_Iq);
    #endif // Open_FW_Calculate

    // 5.电流环PI控制器计算
    Transfer_PID_Loop(&sguan->control.Current_D,
        sguan->foc.Target_Id,
        sguan->current.Real_Id);
    Transfer_PID_Loop(&sguan->control.Current_Q,
        sguan->control.Speed.sta.output,
        sguan->current.Real_Iq);

    // 6.结果输出到Ud和Uq给定
    sguan->foc.Ud_in = sguan->control.Current_D.run.Output + Ud_ff;
    sguan->foc.Uq_in = sguan->control.Current_Q.run.Output + Uq_ff;
    #endif // Open_PI_Control
}

// SVPWM电机驱动的马鞍波生成
static void SVPWM_Tick(SguanFOC_System_STRUCT *sguan,
                    float sine,
                    float cosine,
                    float d_set,
                    float q_set){
    float U_alpha,U_beta;
    ipark(&U_alpha,&U_beta,d_set,q_set,sine,cosine);
    
    SVPWM(U_alpha,U_beta,
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

// Sguan_GeneratePWM_Loop计算PID并执行电机控制
static void Sguan_GeneratePWM_Loop(SguanFOC_System_STRUCT *sguan){
    // 用户实时控制的参数传入
    User_UserControl();
    // PID运算PWM大小并执行
    if (sguan->mode < 4){
        Control_Tick[sguan->mode](sguan);
    } else{
        // 错误处理：自动跳转到默认速度开环模式
        sguan->mode = Velocity_OPEN_MODE;
        Control_Tick[Velocity_OPEN_MODE](sguan);
    }
    SVPWM_Tick(sguan,
        sguan->foc.sine,        // sin正弦值给定
        sguan->foc.cosine,      // cos余弦值给定
        sguan->foc.Ud_in/sguan->motor.VBUS,
        sguan->foc.Uq_in/sguan->motor.VBUS);
}

// =============================== float 版本代码(代码实现) =============================
#else // CONFIG_Q31
static void IQmath_Calculate_Init_Tick_q31(SguanFOC_System_STRUCT *sguan){

}

static void IQmath_Calculate_High_Tick_q31(SguanFOC_System_STRUCT *sguan){

}

#if !CONFIG_PI
static void Transfer_STA_Loop_q31(STA_STRUCT_q31 *sta,Q31_t Ref,Q31_t Fbk){
    sta->sta.Ref = Ref;
    sta->sta.Fbk = Fbk;
    STA_Loop_q31(sta);
}
#endif // CONFIG_PI

static void Transfer_PID_Loop_q31(PID_STRUCT_q31 *pid,Q31_t Ref,Q31_t Fbk){
    pid->run.Ref = Ref;
    pid->run.Fbk = Fbk;
    PID_Loop_q31(pid);
}

static void Transfer_LPF_Loop_q31(LPF_STRUCT_q31 *lpf,Q31_t input){
    lpf->filter.Input = input;
    LPF_Loop_q31(lpf);
}

static void Transfer_PLL_Loop_q31(PLL_STRUCT_q31 *pll,uint8_t mode,Q31_t input_Rad){
    if (mode == PosVelCur_THREE_MODE){
        pll->go.Error = input_Rad - Value_normalize_q31(pll->go.OutRe);
        if (!pll->is_position_mode){
            pll->is_position_mode = 1;
        }
    }
    else{
        pll->go.Error = input_Rad - pll->go.OutRe;
        if (pll->is_position_mode){
            pll->is_position_mode = 0;
        }
    }

    if (pll->go.Error >= Value_PI_q31){
        pll->go.Error -= Value_2PI_q31;
    }
    if (pll->go.Error <= -Value_PI_q31){
        pll->go.Error += Value_2PI_q31;
    }
    PLL_Loop_q31(pll);
}

static void Offset_EncoderRead_q31(SguanFOC_System_STRUCT *sguan){
    float Pos_offset;
    for (uint8_t i = 0; i < 10; i++){
        Pos_offset += User_Encoder_ReadRad();
        User_Delay(2);
    }
    sguan->encoder.Pos_offset_q31 = IQmath_Q31_from_float(Pos_offset/10.0f,BASE_Rad);
}

static void Offset_CurrentRead_q31(SguanFOC_System_STRUCT *sguan){
    for (uint8_t i = 0; i < 24; i++){
        sguan->current.Current_offset0 += User_ReadADC_Raw(0);
        sguan->current.Current_offset1 += User_ReadADC_Raw(1);
        User_Delay(2);
    }
    sguan->current.Current_offset0 = sguan->current.Current_offset0/24;
    sguan->current.Current_offset1 = sguan->current.Current_offset1/24;
    sguan->current.Final_Gain_q31 = IQmath_Q31_from_float(sguan->motor.MCU_Voltage/
        (sguan->motor.ADC_Precision*sguan->motor.Amplifier*sguan->motor.Sampling_Rs),
        BASE_Current);
}

static void Current_ReadIabc_q31(SguanFOC_System_STRUCT *sguan){
    Q31_t I0 = (User_ReadADC_Raw(0) - sguan->current.Current_offset0)*
                            sguan->current.Final_Gain_q31*sguan->motor.Current_Dir0;
    Q31_t I1 = (User_ReadADC_Raw(1) - sguan->current.Current_offset1)*
                            sguan->current.Final_Gain_q31*sguan->motor.Current_Dir1;
    Q31_t I2 = -(I0 + I1);
    if (sguan->motor.Current_Num == 0){
        sguan->current.Real_Ia_q31 = sguan->motor.Motor_Dir == 1 ? I0 : I1;
        sguan->current.Real_Ib_q31 = sguan->motor.Motor_Dir == 1 ? I1 : I0;
        sguan->current.Real_Ic_q31 = I2;
    } else if (sguan->motor.Current_Num == 1){
        sguan->current.Real_Ia_q31 = sguan->motor.Motor_Dir == 1 ? I0 : I2;
        sguan->current.Real_Ib_q31 = sguan->motor.Motor_Dir == 1 ? I2 : I0;
        sguan->current.Real_Ic_q31 = I1;
    } else {
        sguan->current.Real_Ia_q31 = sguan->motor.Motor_Dir == 1 ? I2 : I0;
        sguan->current.Real_Ib_q31 = sguan->motor.Motor_Dir == 1 ? I0 : I2;
        sguan->current.Real_Ic_q31 = I1;
    }
}

static void Sguan_Calculate_Loop_q31(SguanFOC_System_STRUCT *sguan){
    float Rad = User_Encoder_ReadRad();
    sguan->encoder.Real_Rad_q31 = IQmath_Q31_from_float(Rad,BASE_Rad);
    Transfer_PLL_Loop_q31(&sguan->encoder.pll,
                    sguan->mode,
                    (sguan->encoder.Real_Rad_q31 - sguan->encoder.Pos_offset_q31)*
                    sguan->motor.Encoder_Dir);
    Transfer_LPF_Loop_q31(&sguan->lpf.Encoder,
                    sguan->encoder.pll.go.OutWe);
    sguan->encoder.Real_Speed_q31 = sguan->lpf.Encoder.filter.Output;
    sguan->encoder.Real_Pos_q31 = sguan->encoder.pll.go.OutRe;
    sguan->encoder.Real_Erad_q31 = Value_normalize_q31(
                            sguan->encoder.pll.go.OutRe*
                            sguan->motor.Poles);
    sguan->encoder.Real_Espeed_q31 = sguan->encoder.Real_Speed_q31*sguan->motor.Poles;
    fast_sin_cos_q31(IQmath_Q31_to_float(sguan->encoder.Real_Erad_q31,BASE_Rad),
                    &sguan->foc.sine_q31,
                    &sguan->foc.cosine_q31);
    // 2.电机相线和各轴电流计算
    Current_ReadIabc_q31(sguan);
    clarke_q31(&sguan->current.Real_Ialpha_q31,
        &sguan->current.Real_Ibeta_q31,
        sguan->current.Real_Ia_q31,
        sguan->current.Real_Ib_q31);
    park_q31(&sguan->current.Real_Id_q31,
        &sguan->current.Real_Iq_q31,
        sguan->current.Real_Ialpha_q31,
        sguan->current.Real_Ibeta_q31,
        sguan->foc.sine_q31,
        sguan->foc.cosine_q31);
    Transfer_LPF_Loop_q31(&sguan->lpf.CurrentD,
                    sguan->current.Real_Id_q31);
    Transfer_LPF_Loop_q31(&sguan->lpf.CurrentQ,
                    sguan->current.Real_Iq_q31);
    sguan->current.Real_Id_q31 = sguan->lpf.CurrentD.filter.Output;
    sguan->current.Real_Iq_q31 = sguan->lpf.CurrentQ.filter.Output;
}

static void Control_Velocity_OPEN_q31(SguanFOC_System_STRUCT *sguan){
    if (sguan->status < 4){
        sguan->foc.Ud_in_q31 = 0;
        sguan->foc.Uq_in_q31 = 0;
    }
}

static void Control_Current_SINGLE_q31(SguanFOC_System_STRUCT *sguan){
    if (sguan->status < 4){
        sguan->foc.Target_Id_q31 = 0;
        sguan->foc.Target_Iq_q31 = 0;
        sguan->current.Real_Id_q31 = 0;
        sguan->current.Real_Iq_q31 = 0;
    }

    Q31_t Ud_ff_q31 = -IQmath_Q31_mul(IQmath_Q31_mul(IQmath_Q31_mul
                    (sguan->encoder.Real_Espeed_q31,
                    sguan->identify.Lq_q31),
                    sguan->current.Real_Iq_q31),
                    Value_1_Rad_q31);
    Q31_t Uq_ff_q31 =  IQmath_Q31_mul(IQmath_Q31_mul
                    (sguan->encoder.Real_Espeed_q31,
                    (IQmath_Q31_mul(sguan->identify.Ld_q31,sguan->current.Real_Id_q31) + 
                    sguan->identify.Flux_q31)),
                    Value_1_Rad_q31);

    #if Open_FW_Calculate
    FW_MTPA_Loop(&sguan->foc.Target_Id,
                sguan->identify.Flux,
                sguan->identify.Ld,
                sguan->current.Real_Iq,
                sguan->current.Real_Iq);
    #endif // Open_FW_Calculate

    Transfer_PID_Loop_q31(&sguan->control.Current_D,
        sguan->foc.Target_Id_q31,
        sguan->current.Real_Id_q31);
    Transfer_PID_Loop_q31(&sguan->control.Current_Q,
        sguan->foc.Target_Iq_q31,
        sguan->current.Real_Iq_q31);

    sguan->foc.Ud_in_q31 = sguan->control.Current_D.run.Output + Ud_ff_q31;
    sguan->foc.Uq_in_q31 = sguan->control.Current_Q.run.Output + Uq_ff_q31;
}

static void Control_VelCur_DOUBLE_q31(SguanFOC_System_STRUCT *sguan){
    static uint8_t Control_Count = 0;
    Control_Count++;
    if (sguan->status < 4){
        sguan->foc.Target_Speed_q31 = 0;
        sguan->encoder.Real_Speed_q31 = 0;
    }

    #if Open_PI_Control
    if (Control_Count >= sguan->control.Response){
        Transfer_PID_Loop_q31(&sguan->control.Velocity,
            sguan->foc.Target_Speed_q31,
            sguan->encoder.Real_Speed_q31);
        Control_Count = 0;
    }

    Q31_t Ud_ff_q31 = -IQmath_Q31_mul(IQmath_Q31_mul(IQmath_Q31_mul
                    (sguan->encoder.Real_Espeed_q31,
                    sguan->identify.Lq_q31),
                    sguan->current.Real_Iq_q31),
                    Value_1_Rad_q31);
    Q31_t Uq_ff_q31 =  IQmath_Q31_mul(IQmath_Q31_mul
                    (sguan->encoder.Real_Espeed_q31,
                    (IQmath_Q31_mul(sguan->identify.Ld_q31,sguan->current.Real_Id_q31) + 
                    sguan->identify.Flux_q31)),
                    Value_1_Rad_q31);

    #if Open_FW_Calculate
    FW_MTPA_Loop(&sguan->foc.Target_Id,
                sguan->identify.Flux,
                sguan->identify.Ld,
                sguan->current.Real_Iq,
                sguan->current.Real_Iq);
    #endif // Open_FW_Calculate

    Transfer_PID_Loop_q31(&sguan->control.Current_D,
        sguan->foc.Target_Id_q31,
        sguan->current.Real_Id_q31);
    Transfer_PID_Loop_q31(&sguan->control.Current_Q,
        sguan->control.Velocity.run.Output,
        sguan->current.Real_Iq_q31);

    sguan->foc.Ud_in_q31 = sguan->control.Current_D.run.Output + Ud_ff_q31;
    sguan->foc.Uq_in_q31 = sguan->control.Current_Q.run.Output + Uq_ff_q31;
    #else // Open_PI_Control
    if (Control_Count >= sguan->control.Response){
        Transfer_STA_Loop_q31(&sguan->control.Speed,
                        sguan->foc.Target_Speed_q31,
                        sguan->encoder.Real_Speed_q31);
        Control_Count = 0;
    }

    Q31_t Ud_ff_q31 = -IQmath_Q31_mul(IQmath_Q31_mul(IQmath_Q31_mul
                    (sguan->encoder.Real_Espeed_q31,
                    sguan->identify.Lq_q31),
                    sguan->current.Real_Iq_q31),
                    Value_1_Rad_q31);
    Q31_t Uq_ff_q31 =  IQmath_Q31_mul(IQmath_Q31_mul
                    (sguan->encoder.Real_Espeed_q31,
                    (IQmath_Q31_mul(sguan->identify.Ld_q31,sguan->current.Real_Id_q31) + 
                    sguan->identify.Flux_q31)),
                    Value_1_Rad_q31);

    #if Open_FW_Calculate
    FW_MTPA_Loop(&sguan->foc.Target_Id,
                sguan->identify.Flux,
                sguan->identify.Ld,
                sguan->current.Real_Iq,
                sguan->current.Real_Iq);
    #endif // Open_FW_Calculate

    Transfer_PID_Loop_q31(&sguan->control.Current_D,
        sguan->foc.Target_Id_q31,
        sguan->current.Real_Id_q31);
    Transfer_PID_Loop_q31(&sguan->control.Current_Q,
        sguan->control.Speed.sta.output,
        sguan->current.Real_Iq_q31);

    sguan->foc.Ud_in_q31 = sguan->control.Current_D.run.Output + Ud_ff_q31;
    sguan->foc.Uq_in_q31 = sguan->control.Current_Q.run.Output + Uq_ff_q31;
    #endif // Open_PI_Control
}

static void Control_PosVelCur_THREE_q31(SguanFOC_System_STRUCT *sguan){
    static uint8_t Control_Count = 0;
    Control_Count++;
    if (sguan->status < 4){
        sguan->foc.Target_Pos_q31 = 0;
        sguan->encoder.Real_Pos_q31 = 0;
    }

    #if Open_PI_Control
    if (Control_Count >= sguan->control.Response*sguan->control.Response){
        Transfer_PID_Loop_q31(&sguan->control.Position,
            sguan->foc.Target_Pos_q31,
            sguan->encoder.Real_Pos_q31);
        Control_Count = 0;
    }

    if (Control_Count % sguan->control.Response == 0){
        Transfer_PID_Loop_q31(&sguan->control.Velocity,
            sguan->control.Position.run.Output,
            sguan->encoder.Real_Speed_q31);
    }

    Q31_t Ud_ff_q31 = -IQmath_Q31_mul(IQmath_Q31_mul(IQmath_Q31_mul
                    (sguan->encoder.Real_Espeed_q31,
                    sguan->identify.Lq_q31),
                    sguan->current.Real_Iq_q31),
                    Value_1_Rad_q31);
    Q31_t Uq_ff_q31 =  IQmath_Q31_mul(IQmath_Q31_mul
                    (sguan->encoder.Real_Espeed_q31,
                    (IQmath_Q31_mul(sguan->identify.Ld_q31,sguan->current.Real_Id_q31) + 
                    sguan->identify.Flux_q31)),
                    Value_1_Rad_q31);

    #if Open_FW_Calculate
    FW_MTPA_Loop(&sguan->foc.Target_Id,
                sguan->identify.Flux,
                sguan->identify.Ld,
                sguan->current.Real_Iq,
                sguan->current.Real_Iq);
    #endif // Open_FW_Calculate

    Transfer_PID_Loop_q31(&sguan->control.Current_D,
        sguan->foc.Target_Id_q31,
        sguan->current.Real_Id_q31);
    Transfer_PID_Loop_q31(&sguan->control.Current_Q,
        sguan->control.Velocity.run.Output,
        sguan->current.Real_Iq_q31);

    sguan->foc.Ud_in_q31 = sguan->control.Current_D.run.Output + Ud_ff_q31;
    sguan->foc.Uq_in_q31 = sguan->control.Current_Q.run.Output + Uq_ff_q31;
    #else // Open_PI_Control
    if (Control_Count >= sguan->control.Response*sguan->control.Response){
        Transfer_PID_Loop_q31(&sguan->control.Position,
            sguan->foc.Target_Pos_q31,
            sguan->encoder.Real_Pos_q31);
        Control_Count = 0;
    }

    if (Control_Count % sguan->control.Response == 0){
        Transfer_STA_Loop_q31(&sguan->control.Speed,
                        sguan->control.Position.run.Output,
                        sguan->encoder.Real_Speed_q31);
    }

    Q31_t Ud_ff_q31 = -IQmath_Q31_mul(IQmath_Q31_mul(IQmath_Q31_mul
                    (sguan->encoder.Real_Espeed_q31,
                    sguan->identify.Lq_q31),
                    sguan->current.Real_Iq_q31),
                    Value_1_Rad_q31);
    Q31_t Uq_ff_q31 =  IQmath_Q31_mul(IQmath_Q31_mul
                    (sguan->encoder.Real_Espeed_q31,
                    (IQmath_Q31_mul(sguan->identify.Ld_q31,sguan->current.Real_Id_q31) + 
                    sguan->identify.Flux_q31)),
                    Value_1_Rad_q31);

    #if Open_FW_Calculate
    FW_MTPA_Loop(&sguan->foc.Target_Id,
                sguan->identify.Flux,
                sguan->identify.Ld,
                sguan->current.Real_Iq,
                sguan->current.Real_Iq);
    #endif // Open_FW_Calculate

    Transfer_PID_Loop_q31(&sguan->control.Current_D,
        sguan->foc.Target_Id_q31,
        sguan->current.Real_Id_q31);
    Transfer_PID_Loop_q31(&sguan->control.Current_Q,
        sguan->control.Speed.sta.output,
        sguan->current.Real_Iq_q31);

    sguan->foc.Ud_in_q31 = sguan->control.Current_D.run.Output + Ud_ff_q31;
    sguan->foc.Uq_in_q31 = sguan->control.Current_Q.run.Output + Uq_ff_q31;
    #endif // Open_PI_Control
}

static void SVPWM_Tick_q31(SguanFOC_System_STRUCT *sguan,
                    Q31_t sine,
                    Q31_t cosine,
                    Q31_t d_set,
                    Q31_t q_set){
    Q31_t U_alpha,U_beta;
    ipark_q31(&U_alpha,&U_beta,d_set,q_set,sine,cosine);

    SVPWM_q31(U_alpha,U_beta,
        &sguan->foc.Du_q31,
        &sguan->foc.Du_q31,
        &sguan->foc.Du_q31);
    
    sguan->foc.Du = IQmath_Q31_to_float(sguan->foc.Du_q31,1.0f);
    sguan->foc.Dv = IQmath_Q31_to_float(sguan->foc.Du_q31,1.0f);
    sguan->foc.Dw = IQmath_Q31_to_float(sguan->foc.Du_q31,1.0f);

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
    if (sguan->motor.Motor_Dir == -1){
        uint16_t duty_temp = sguan->foc.Duty_u;
        sguan->foc.Duty_u = sguan->foc.Duty_v;
        sguan->foc.Duty_v = duty_temp;
    }
    User_PwmDuty_Set(sguan->foc.Duty_u,sguan->foc.Duty_v,sguan->foc.Duty_w);
}

static void Sguan_GeneratePWM_Loop_q31(SguanFOC_System_STRUCT *sguan){
    User_UserControl();
    if (sguan->mode < 4){
        Control_Tick_q31[sguan->mode](sguan);
    } else{
        sguan->mode = Velocity_OPEN_MODE;
        Control_Tick_q31[Velocity_OPEN_MODE](sguan);
    }
    Q31_t d = IQmath_Q31_from_float(sguan->foc.Ud_in/sguan->motor.VBUS,1.0f);
    Q31_t q = IQmath_Q31_from_float(sguan->foc.Uq_in/sguan->motor.VBUS,1.0f);

    SVPWM_Tick_q31(sguan,
        sguan->foc.sine_q31,
        sguan->foc.cosine_q31,
        IQmath_Q31_div(sguan->foc.Ud_in_q31,sguan->motor.VBUS_q31),
        IQmath_Q31_div(sguan->foc.Uq_in_q31,sguan->motor.VBUS_q31));
}
#endif // CONFIG_Q31


// =============================== 通用数据层(代码实现) =============================

// Data母线电压和驱动器物理温度数据更新
static void Data_Protection_Loop(SguanFOC_System_STRUCT *sguan){
    if (User_VBUS_DataGet() != -9999.0f){        
        sguan->foc.Real_VBUS = User_VBUS_DataGet();
    }
    if (User_Temperature_DataGet() != -9999.0f){        
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
    if ((sguan->status != MOTOR_STATUS_STANDBY) && 
        MOTOR_STATUS_STANDBY_Signal()){ // [重要]接收到待机信号后，解除锁定状态...进入待机状态
        sguan->status = MOTOR_STATUS_STANDBY;
    }
    if ((sguan->status == MOTOR_STATUS_EMERGENCY_STOP) || 
        (sguan->status == MOTOR_STATUS_DISABLED) || (sguan->status < 4)){
        return; // 紧急停止和失能状态优先级最高, 直接返回不执行后续状态判断
    }
    if ((sguan->status != MOTOR_STATUS_UNINITIALIZED) && 
        MOTOR_STATUS_UNINITIALIZED_Signal()){ // [重要]接收到开始信号后，切换到未初始化状态...准备初始化
        sguan->status = MOTOR_STATUS_UNINITIALIZED;
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
        (sguan->current.Real_Iq > (sguan->foc.Target_Iq - 0.2f)) && 
        (sguan->current.Real_Iq < (sguan->foc.Target_Iq + 0.2f))){
        sguan->status = MOTOR_STATUS_TORQUE_CONTROL;
    }
    if ((sguan->mode == Current_SINGLE_MODE) && 
        (sguan->status != MOTOR_STATUS_TORQUE_INCREASING) && 
        (sguan->current.Real_Iq < (sguan->foc.Target_Iq + 0.2f))){
        sguan->status = MOTOR_STATUS_TORQUE_INCREASING;
    }
    if ((sguan->mode == Current_SINGLE_MODE) && 
        (sguan->status != MOTOR_STATUS_TORQUE_DECREASING) && 
        (sguan->current.Real_Iq > (sguan->foc.Target_Iq - 0.2f))){
        sguan->status = MOTOR_STATUS_TORQUE_DECREASING;
    }
    // 2.速度模式检测
    if ((sguan->mode == VelCur_DOUBLE_MODE) && 
        (sguan->status != MOTOR_STATUS_CONST_SPEED) && 
        (sguan->encoder.Real_Speed > sguan->foc.Target_Speed - 6.0f) && 
        (sguan->encoder.Real_Speed < sguan->foc.Target_Speed + 6.0f)){
        sguan->status = MOTOR_STATUS_CONST_SPEED;
    }
    if ((sguan->mode == VelCur_DOUBLE_MODE) && 
        (sguan->status != MOTOR_STATUS_ACCELERATING) && 
        (sguan->encoder.Real_Speed < sguan->foc.Target_Speed + 6.0f)){
        sguan->status = MOTOR_STATUS_ACCELERATING;
    }
    if ((sguan->mode == VelCur_DOUBLE_MODE) && 
        (sguan->status != MOTOR_STATUS_DECELERATING) && 
        (sguan->encoder.Real_Speed > sguan->foc.Target_Speed - 6.0f)){
        sguan->status = MOTOR_STATUS_DECELERATING;
    }
    // 3.位置模式检测
    if ((sguan->mode == PosVelCur_THREE_MODE) && 
        (sguan->status != MOTOR_STATUS_POSITION_HOLD) && 
        (sguan->encoder.Real_Pos > sguan->foc.Target_Pos - 1.0f) && 
        (sguan->encoder.Real_Pos < sguan->foc.Target_Pos + 1.0f)){
        sguan->status = MOTOR_STATUS_POSITION_HOLD;
    }
    if ((sguan->mode == PosVelCur_THREE_MODE) && 
        (sguan->status != MOTOR_STATUS_POSITION_INCREASING) && 
        (sguan->encoder.Real_Pos < sguan->foc.Target_Pos + 1.0f)){
        sguan->status = MOTOR_STATUS_POSITION_INCREASING;
    }
    if ((sguan->mode == PosVelCur_THREE_MODE) && 
        (sguan->status != MOTOR_STATUS_POSITION_DECREASING) && 
        (sguan->encoder.Real_Pos > sguan->foc.Target_Pos - 1.0f)){
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
#if Open_Printf_Debug
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
#else // Open_Printf_Debug
// Printf电机数据正常发送
static void Printf_Normal_Loop(SguanFOC_System_STRUCT *sguan){
    // 用户数据填写(串口或者CAN通信)
    User_UserTX();
    // 发送数据to上位机
    Printf_Loop(&Sguan.TXdata);
}
#endif // Open_Printf_Debug


// Sguan...Set系统时钟设置(定时器中断周期)
static void Sguan_SystemT_Set(SguanFOC_System_STRUCT *sguan){
    // 1.bpf低通滤波器
    sguan->lpf.CurrentD.T = PMSM_RUN_T;
    sguan->lpf.CurrentQ.T = PMSM_RUN_T;
    sguan->lpf.Encoder.T = PMSM_RUN_T;
    // 2.control闭环控制系统
    sguan->control.Current_D.T = PMSM_RUN_T;
    sguan->control.Current_Q.T = PMSM_RUN_T;

    #if Open_PI_Control
    sguan->control.Velocity.T = PMSM_RUN_T*sguan->control.Response;
    #else // Open_PI_Control
    sguan->control.Speed.T = PMSM_RUN_T*sguan->control.Response;
    #endif // Open_PI_Control

    sguan->control.Position.T = PMSM_RUN_T*sguan->control.Response*sguan->control.Response;
    // 3.pll锁相环跟踪系统
    sguan->encoder.pll.T = PMSM_RUN_T;
}

// Sguan...Set电机零点对齐(机械角度对齐)
static void Sguan_Positioning_Set(SguanFOC_System_STRUCT *sguan,float Ud,float Uq){
    #if !CONFIG_Q31
    // 电机零位Ud,Uq设计
    SVPWM_Tick(sguan,0.0f,1.0f,
        Ud/sguan->motor.VBUS,
        Uq/sguan->motor.VBUS);
    #else // CONFIG_Q31
    Q31_t d = IQmath_Q31_from_float(Ud,BASE_Voltage);
    Q31_t q = IQmath_Q31_from_float(Uq,BASE_Voltage);
    SVPWM_Tick_q31(sguan,0,Q31_MAX,
        IQmath_Q31_div(d,sguan->motor.VBUS_q31),
        IQmath_Q31_div(q,sguan->motor.VBUS));
    #endif // CONFIG_Q31
}

// Sguan...Init巴特沃斯低通滤波器的初始化
static void Sguan_LPF_Init(SguanFOC_System_STRUCT *sguan){
    #if !CONFIG_Q31
    LPF_Init(&sguan->lpf.CurrentD);
    LPF_Init(&sguan->lpf.CurrentQ);
    LPF_Init(&sguan->lpf.Encoder);
    #else // CONFIG_Q31
    if (LPF_Init_q31(&sguan->lpf.CurrentD) || LPF_Init_q31(&sguan->lpf.CurrentQ)){
        IQmath_SetBit(&sguan->IQmath_Error,0,1);
    }
    IQmath_SetBit(&sguan->IQmath_Error,1,
                LPF_Init_q31(&sguan->lpf.Encoder));
    #endif // CONFIG_Q31
}

// Sguan...Init闭环控制算法PID的初始化
static void Sguan_Control_Init(SguanFOC_System_STRUCT *sguan){
    #if !CONFIG_Q31
    // 1.电流环PI控制器初始化
    PID_Init(&sguan->control.Current_D);
    PID_Init(&sguan->control.Current_Q);
    // 2.转速环PI控制器初始化
    #if CONFIG_PI
    PID_Init(&sguan->control.Velocity);
    #else // CONFIG_PI
    // STA_Init(&Sguan.control.Speed);
    STA_Init(&Sguan.control.Speed);
    #endif // CONFIG_PI
    // 3.位置环PD控制器初始化
    PID_Init(&sguan->control.Position);
    #else // CONFIG_Q31
    // 1.电流环PI控制器初始化
    sguan->control.Current_D.BASE_In = BASE_Current;
    sguan->control.Current_D.BASE_Out = BASE_Voltage;
    sguan->control.Current_Q.BASE_In = BASE_Current;
    sguan->control.Current_Q.BASE_Out = BASE_Voltage;
    if (PID_Init_q31(&sguan->control.Current_D) || 
        PID_Init_q31(&sguan->control.Current_Q)){
        IQmath_SetBit(&sguan->IQmath_Error,2,1);
    }
    // 2.转速环PI控制器初始化
    #if CONFIG_PI
    sguan->control.Velocity.BASE_In = BASE_Speed;
    sguan->control.Velocity.BASE_Out = BASE_Current;
    IQmath_SetBit(&sguan->IQmath_Error,3,
                PID_Init_q31(&sguan->control.Velocity));
    #else // CONFIG_PI
    IQmath_SetBit(&sguan->IQmath_Error,4,
                STA_Init_q31(&Sguan.control.Speed));
    #endif // CONFIG_PI
    // 3.位置环PD控制器初始化
    sguan->control.Position.BASE_In = BASE_Rad;
    sguan->control.Position.BASE_Out = BASE_Speed;
    IQmath_SetBit(&sguan->IQmath_Error,5,
                PID_Init_q31(&sguan->control.Position));
    #endif // CONFIG_Q31
}

// Sguan...Init锁相环PLL的初始化
static void Sguan_PLL_Init(SguanFOC_System_STRUCT *sguan){
    #if !CONFIG_Q31
    PLL_Init(&sguan->encoder.pll);
    #else // CONFIG_Q31
    IQmath_SetBit(&sguan->IQmath_Error,6,
                PLL_Init_q31(&sguan->encoder.pll));
    #endif // CONFIG_Q31
}

// Sguan...Tick系统开始的核心文件，主任务初始化函数
static void Sguan_Start_Tick(void){
    if (Sguan.status == MOTOR_STATUS_UNINITIALIZED){
        // 用户自定义的电机参数和控制系统参数
        User_InitialInit();
        User_MotorSet();
        User_ParameterSet();
        // 系统时间设定
        Sguan_SystemT_Set(&Sguan);
        // 各种控制系统的初始化
        Sguan.status = MOTOR_STATUS_INITIALIZING;
        #if CONFIG_Q31
        IQmath_Calculate_Init_Tick_q31(&Sguan);
        #endif // CONFIG_Q31
        Sguan_LPF_Init(&Sguan);
        Sguan_Control_Init(&Sguan);
        Sguan_PLL_Init(&Sguan);
        Printf_Init(&Sguan.TXdata);
        // 读取电流偏置
        Sguan.status = MOTOR_STATUS_CALIBRATING;
        #if !CONFIG_Q31
        Offset_CurrentRead(&Sguan);
        #else // CONFIG_Q31
        Offset_CurrentRead_q31(&Sguan);
        #endif // CONFIG_Q31
        //电机回零操作
        Sguan_Positioning_Set(&Sguan,0.3f*Sguan.motor.VBUS,0.0f);
        User_Delay(1200);
        // 读取角度偏置
        #if !CONFIG_Q31
        Offset_EncoderRead(&Sguan);
        #else // CONFIG_Q31
        Offset_EncoderRead_q31(&Sguan);
        #endif // CONFIG_Q31
        // 电机失能并进入正常工作状态
        Sguan_Positioning_Set(&Sguan,0.0f,0.0f);
        User_Delay(800);
        // 判断电机的极性,如果是SPMSM,D轴给定,可能停在0或者180度位置
        // 暂时还未书写
        //正常工作中(状态机运行)
        Sguan.status = MOTOR_STATUS_IDLE;
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

        // 如果在初始化完成，进入函数
        if (Sguan.status > 3 && Sguan.status < 19){       
            #if !CONFIG_Q31     
            // 计算编码器和电流
            Sguan_Calculate_Loop(&Sguan);
            // 运算PID并执行SVPWM(如果计算超时，会更新错误状态并停用此线程)
            Sguan_GeneratePWM_Loop(&Sguan);
            #else // CONFIG_Q31
            IQmath_Calculate_High_Tick_q31(&Sguan);
            Sguan_Calculate_Loop_q31(&Sguan);
            Sguan_GeneratePWM_Loop_q31(&Sguan);
            #endif // CONFIG_Q31
        }
        if (Sguan.status >= 19){
            static uint8_t status = 0xFF;
            if (status != Sguan.status){
                #if !CONFIG_Q31
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
                SVPWM_Tick(&Sguan,      // 清零电机状态
                    Sguan.foc.sine,
                    Sguan.foc.cosine,
                    Sguan.foc.Ud_in/Sguan.motor.VBUS,
                    Sguan.foc.Uq_in/Sguan.motor.VBUS);
                status = Sguan.status;
                #else // CONFIG_Q31
                Sguan.foc.Ud_in_q31 = 0;
                Sguan.foc.Uq_in_q31 = 0;

                Sguan.encoder.Pos_offset_q31 = 0;
                Sguan.current.Current_offset0 = 0;
                Sguan.current.Current_offset1 = 0;

                Sguan.foc.Target_Id_q31 = 0;
                Sguan.foc.Target_Iq_q31 = 0;
                Sguan.foc.Target_Speed_q31 = 0;
                Sguan.foc.Target_Pos_q31 = 0;
                SVPWM_Tick_q31(&Sguan,
                    Sguan.foc.sine_q31,
                    Sguan.foc.cosine_q31,
                    IQmath_Q31_div(Sguan.foc.Ud_in_q31,Sguan.motor.VBUS_q31),
                    IQmath_Q31_div(Sguan.foc.Uq_in_q31,Sguan.motor.VBUS_q31));
                status = Sguan.status;
                #endif // CONFIG_Q31
            }
        }

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
void SguanFOC_main_Loop(void){
    Sguan_Start_Tick();
    #if !Printf_Debug
    if (Sguan.status > 3 && Sguan.status < 19){
        Printf_Normal_Loop(&Sguan);
    }
    else if (Sguan.status >= 19){
        static uint8_t count = 0;
        count++;
        if (count > 10){
            Printf_Normal_Loop(&Sguan);
            count = 0;
        }
    }
    #endif // Printf_Debug
}


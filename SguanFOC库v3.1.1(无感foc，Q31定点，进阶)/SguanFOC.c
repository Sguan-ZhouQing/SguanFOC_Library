/*
 * @Author: 星必尘Sguan
 * @GitHub: https://github.com/Sguan-ZhouQing
 * @Date: 2026-01-26 22:38:34
 * @LastEditors: 星必尘Sguan|3464647102@qq.com
 * @LastEditTime: 2026-04-01 03:44:33
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

// ================================= (静态函数声明) ==================================
static Q31_t Transfer_LPF_Loop(LPF_STRUCT *lpf, Q31_t Input);
static Q31_t Transfer_PID_Loop(PID_STRUCT *pid, Q31_t Ref, Q31_t Fbk);
static void Transfer_NLFO_Loop(SguanFOC_System_STRUCT *sguan, 
                            PLL_STRUCT *pll);
static Q31_t Feedforward_CurrentD(Q31_t We,Q31_t Lq,Q31_t Iq);
static Q31_t Feedforward_CurrentQ(Q31_t We,Q31_t Ld,Q31_t Id,Q31_t Flux);
static void Float_DataSwitch_Loop(SguanFOC_System_STRUCT *sguan);
static void High_Current_Loop(SguanFOC_System_STRUCT *sguan);
static void High_Encoder_Loop(SguanFOC_System_STRUCT *sguan);
static void High_Control_Loop(SguanFOC_System_STRUCT *sguan);
static void High_PWM_Loop(SguanFOC_System_STRUCT *sguan);
static void Low_DataRead_Loop(SguanFOC_System_STRUCT *sguan);
static void Low_StatusSwitch_Loop(SguanFOC_System_STRUCT *sguan);
static void main_Transfer_Init(SguanFOC_System_STRUCT *sguan);
static void main_Current_Init(SguanFOC_System_STRUCT *sguan);
static void main_loop(SguanFOC_System_STRUCT *sguan);
// =============================== (Transfer/Feedforward) ===========================
static Q31_t Transfer_LPF_Loop(LPF_STRUCT *lpf, Q31_t Input){
    lpf->go.Input = Input;
    LPF_Loop(lpf);
    return lpf->go.Output;
}

static Q31_t Transfer_PID_Loop(PID_STRUCT *pid, Q31_t Ref, Q31_t Fbk){
    pid->go.Ref = Ref;
    pid->go.Fbk = Fbk;
    PID_Loop(pid);
    return pid->go.Output;
}

static void Transfer_NLFO_Loop(SguanFOC_System_STRUCT *sguan, 
                            PLL_STRUCT *pll){
    // 1.计算无感磁链观测器
    sguan->Transfer.NLFO.go.Input_Ialpha = sguan->Motor.Real_Ialpha;
    sguan->Transfer.NLFO.go.Input_Ibeta = sguan->Motor.Real_Ibeta;
    sguan->Transfer.NLFO.go.Input_Ualpha = sguan->Foc.Ualpha;
    sguan->Transfer.NLFO.go.Input_Ubeta = sguan->Foc.Ubeta;
    NLFO_Loop(&sguan->Transfer.NLFO);

    #if 1
    Q31_t Cosine,Sine;
    fast_sin_cos(pll->go.OutRe, &Sine, &Cosine);

    // 2.计算正交锁相环
    Q31_t Error = iqmath_sub(
        iqmath_mul(sguan->Transfer.NLFO.go.Output_Sine, Cosine),
        iqmath_mul(sguan->Transfer.NLFO.go.Output_Cosine, Sine));
    #else
    Q31_t Error = iqmath_sub(
        iqmath_mul(sguan->Transfer.NLFO.go.Output_Sine, sguan->Foc.Cosine),
        iqmath_mul(sguan->Transfer.NLFO.go.Output_Cosine, sguan->Foc.Sine));
    #endif

    pll->go.Error = (Q31_t)((uint32_t)Error << 9); // Sguan_NLFO中有缩放9倍

    PLL_Loop(pll);
}

// D轴电流前馈量，返回电压控制量
static Q31_t Feedforward_CurrentD(Q31_t We,Q31_t Lq,Q31_t Iq){
    return -iqmath_mul(iqmath_mul(Iq,Lq), We);
}

// Q轴电流前馈量，返回电压控制量
static Q31_t Feedforward_CurrentQ(Q31_t We,Q31_t Ld,Q31_t Id,Q31_t Flux){
    Q31_t temp = iqmath_mul(Ld, Id);
    return iqmath_mul(iqmath_add(temp,Flux), We);
}

static void Float_DataSwitch_Loop(SguanFOC_System_STRUCT *sguan){
    // 1.浮点数自动转定点
    sguan->Foc.Target_Speed = iqmath_from_float(sguan->Float.Target_Speed, BASE_Speed);
    sguan->Foc.Target_Uq = iqmath_from_float(sguan->Float.Target_Uq, BASE_Voltage);
    sguan->Foc.Target_VF_Uq = iqmath_from_float(sguan->Float.Target_VF_Uq, BASE_Voltage);
    sguan->Foc.Target_IF_Iq = iqmath_from_float(sguan->Float.Target_IF_Iq, BASE_Current);
    sguan->Foc.Target_Id = iqmath_from_float(sguan->Float.Target_Id, BASE_Current);
    sguan->Foc.Target_Iq = iqmath_from_float(sguan->Float.Target_Iq, BASE_Current);

    // 2.定点数自动转浮点
    sguan->Float.Real_Speed = iqmath_to_float(sguan->Motor.Real_Speed, BASE_Speed);
    sguan->Float.Real_Uq = iqmath_to_float(sguan->Foc.Uq_in, BASE_Voltage);
    sguan->Float.Real_Re = iqmath_to_float(sguan->Motor.Real_Re, BASE_Rad);
    sguan->Float.Real_Id = iqmath_to_float(sguan->Motor.Real_Id, BASE_Current);
    sguan->Float.Real_Iq = iqmath_to_float(sguan->Motor.Real_Iq, BASE_Current);
}

// ================================= (High) ===================================
static void High_Current_Loop(SguanFOC_System_STRUCT *sguan){
    // 1.读取三相原始数值
    #if CONFIG_CUR
    // 带入电流偏置计算
    if (sguan->Motor.Current_Dir == 1){        
        sguan->Motor.Real_Ia = iqmath_current_raw_to_q31(
            User_ReadADC_Raw(0) - sguan->Motor.Current_Offset0);
        sguan->Motor.Real_Ib = iqmath_current_raw_to_q31(
            User_ReadADC_Raw(1) - sguan->Motor.Current_Offset1);
        // sguan->Motor.Real_Ic = iqmath_current_raw_to_q31(
        //     User_ReadADC_Raw(2) - sguan->Motor.Current_Offset2);
        // (注释掉Ic电流的读取，是为了节省运算)
        sguan->Motor.Real_Ic = iqmath_current_raw_to_q31(
             User_ReadADC_Raw(2) - sguan->Motor.Current_Offset2);
        // (注释掉Ic电流的读取，是为了节省运算)
    }
    else{
        sguan->Motor.Real_Ia = -iqmath_current_raw_to_q31(
            User_ReadADC_Raw(0) - sguan->Motor.Current_Offset0);
        sguan->Motor.Real_Ib = -iqmath_current_raw_to_q31(
            User_ReadADC_Raw(1) - sguan->Motor.Current_Offset1);
        sguan->Motor.Real_Ic = -iqmath_current_raw_to_q31(
             User_ReadADC_Raw(2) - sguan->Motor.Current_Offset2);
    }
    #else // CONFIG_CUR
    // 不带入电流偏置计算
    if (sguan->Motor.Current_Dir == 1){
        sguan->Motor.Real_Ia = iqmath_current_raw_to_q31(
            User_ReadADC_Raw(0));
        sguan->Motor.Real_Ib = iqmath_current_raw_to_q31(
            User_ReadADC_Raw(1));
        // sguan->Motor.Real_Ic = iqmath_current_raw_to_q31(
        //     User_ReadADC_Raw(2));
        // (注释掉Ic电流的读取，是为了节省运算)
    }
    else{
        sguan->Motor.Real_Ia = -iqmath_current_raw_to_q31(
            User_ReadADC_Raw(0));
        sguan->Motor.Real_Ib = -iqmath_current_raw_to_q31(
            User_ReadADC_Raw(1));
    }
    #endif // CONFIG_CUR

    // 2.坐标变换计算
    clarke(&sguan->Motor.Real_Ialpha, 
        &sguan->Motor.Real_Ibeta, 
        sguan->Motor.Real_Ia, 
        sguan->Motor.Real_Ib);
    park(&sguan->Motor.Real_Id_temp, 
        &sguan->Motor.Real_Iq_temp, 
        sguan->Motor.Real_Ialpha, 
        sguan->Motor.Real_Ibeta, 
        sguan->Foc.Sine, 
        sguan->Foc.Cosine);

    // 3.电流DQ轴滤波
    sguan->Motor.Real_Id = Transfer_LPF_Loop(
        &sguan->Transfer.LPF_D, 
        sguan->Motor.Real_Id_temp);
    sguan->Motor.Real_Iq = Transfer_LPF_Loop(
        &sguan->Transfer.LPF_Q, 
        sguan->Motor.Real_Iq_temp);
}

static void High_Encoder_Loop(SguanFOC_System_STRUCT *sguan){
    // 1.期望数值更新
    #if CONFIG_MODE==MODE_NLFO_Voltag
    sguan->Foc.Uq_in = Transfer_LPF_Loop(
        &sguan->Transfer.LPF_Ltd, 
        sguan->Foc.Target_Uq);
    #else // CONFIG_MODE
    sguan->Foc.Speed_in = Transfer_LPF_Loop(
        &sguan->Transfer.LPF_Ltd, 
        sguan->Foc.Target_Speed);
    #endif // CONFIG_MODE

    // 2.运行开环强拖角度生成
    #if CONFIG_MODE<=MODE_IF_Only
    sguan->Motor.Real_Speed = sguan->Foc.Speed_in;
    sguan->Motor.Real_We = 
        sguan->Motor.Real_Speed*
        sguan->Motor.Poles;

    iqmath_rad_loop(
        &sguan->Motor.Real_Re, 
        sguan->Motor.Real_We, 
        PMSM_RUN_T_q31);

    // // .........................................................
    Transfer_NLFO_Loop(sguan, &sguan->Transfer.PLL);
    
    // (电机角速度滤波)
    Transfer_LPF_Loop(
        &sguan->Transfer.LPF_Speed, 
        sguan->Transfer.PLL.go.OutWe);
    // // .........................................................
    #else // CONFIG_MODE
    
    // 2.运行无感foc控制算法
    // (运行无感NLFO磁链观测器)
    Transfer_NLFO_Loop(sguan, &sguan->Transfer.PLL);

    // (电机角速度滤波)
    sguan->Motor.Real_We = Transfer_LPF_Loop(
        &sguan->Transfer.LPF_Speed, 
        sguan->Transfer.PLL.go.OutWe);
    sguan->Motor.Real_Speed = iqmath_speed_div5_fast(sguan->Motor.Real_We);

    // (刷新电机角度相关信息)
    sguan->Motor.Real_Re = sguan->Transfer.PLL.go.OutRe;
    #endif // CONFIG_MODE

    // 3.电机三角函数求解
    fast_sin_cos(
        sguan->Motor.Real_Re, 
        &sguan->Foc.Sine, 
        &sguan->Foc.Cosine);
}

static void High_Control_Loop(SguanFOC_System_STRUCT *sguan){
    #if CONFIG_MODE==MODE_VF_Only
    sguan->Foc.Uq_in = sguan->Foc.Target_VF_Uq;
    #elif CONFIG_MODE==MODE_IF_Only
    // 1.更新IF控制的控制量
    sguan->Foc.Target_Iq = sguan->Foc.Target_IF_Iq;

    // 2.电流环控制器求解
    sguan->Foc.Ud_in = Transfer_PID_Loop(
        &sguan->Transfer.PID_D, 
        sguan->Foc.Target_Id, 
        sguan->Motor.Real_Id);
    sguan->Foc.Uq_in = Transfer_PID_Loop(
        &sguan->Transfer.PID_Q, 
        sguan->Foc.Target_Iq, 
        sguan->Motor.Real_Iq);

    // 3.电流前馈计算
    sguan->Foc.Ud_in += Feedforward_CurrentD(
        sguan->Motor.Real_We, 
        sguan->Motor.Lq, 
        sguan->Motor.Real_Iq);
    sguan->Foc.Uq_in += Feedforward_CurrentQ(
        sguan->Motor.Real_We, 
        sguan->Motor.Ld, 
        sguan->Motor.Real_Id, 
        sguan->Motor.Flux);
    #elif CONFIG_MODE>=MODE_NLFO_Vel
    // 1.转速环控制器求解
    if (sguan->Status == STATUS_COM){
        static uint8_t count = 0;
        count++;
        if (count >= sguan->Transfer.Response){
            sguan->Foc.Target_Iq = Transfer_PID_Loop(
                &sguan->Transfer.PID_Speed, 
                sguan->Foc.Speed_in, 
                sguan->Motor.Real_Speed);

            count = 0;
        }
    }

    // 2.电流环控制器求解
    sguan->Foc.Ud_in = Transfer_PID_Loop(
        &sguan->Transfer.PID_D, 
        sguan->Foc.Target_Id, 
        sguan->Motor.Real_Id);
    sguan->Foc.Uq_in = Transfer_PID_Loop(
        &sguan->Transfer.PID_Q, 
        sguan->Foc.Target_Iq, 
        sguan->Motor.Real_Iq);

    // 3.电流前馈计算
    sguan->Foc.Ud_in += Feedforward_CurrentD(
        sguan->Motor.Real_We, 
        sguan->Motor.Lq, 
        sguan->Motor.Real_Iq);
    sguan->Foc.Uq_in += Feedforward_CurrentQ(
        sguan->Motor.Real_We, 
        sguan->Motor.Ld, 
        sguan->Motor.Real_Id, 
        sguan->Motor.Flux);
    #endif // CONGIG_MODE
}

static void High_PWM_Loop(SguanFOC_System_STRUCT *sguan){
    // 1.Park逆变换
    ipark(&sguan->Foc.Ualpha,
        &sguan->Foc.Ubeta,
        sguan->Foc.Ud_in,
        sguan->Foc.Uq_in, 
        sguan->Foc.Sine, 
        sguan->Foc.Cosine);


    // 2.运行SVPWM函数
    SVPWM(iqmath_div(sguan->Foc.Ualpha, sguan->Foc.VBUS), 
        iqmath_div(sguan->Foc.Ubeta, sguan->Foc.VBUS), 
        &sguan->Foc.Du, 
        &sguan->Foc.Dv, 
        &sguan->Foc.Dw);

    // 3.计算比较器数值
    sguan->Foc.Duty_u = (uint16_t)(iqmath_mul(sguan->Foc.Du,(Q31_t)sguan->Motor.Duty));
    sguan->Foc.Duty_v = (uint16_t)(iqmath_mul(sguan->Foc.Dv,(Q31_t)sguan->Motor.Duty));
    sguan->Foc.Duty_w = (uint16_t)(iqmath_mul(sguan->Foc.Dw,(Q31_t)sguan->Motor.Duty));
    
    // 4.输出限幅并执行
    User_PwmDuty_Set(sguan->Foc.Duty_u,sguan->Foc.Duty_v,sguan->Foc.Duty_w);
}

// ================================= (Low) ===================================
static void Low_DataRead_Loop(SguanFOC_System_STRUCT *sguan){
    sguan->Safe.Vbus_Real = User_VBUS_DataGet();
    sguan->Safe.Temp_Real = User_Temperature_DataGet();
    sguan->Safe.Ibus_Real = User_ReadADC_Raw(0); // 象征性Ibus读取
}

static void Low_StatusSwitch_Loop(SguanFOC_System_STRUCT *sguan){
    if (sguan->Status == STATUS_COM){
        // 1.过压保护
        if (sguan->Safe.Vbus_Real >= sguan->Safe.Vbus_Max){
            sguan->Status = STATUS_Standby;
            sguan->Error_Code = ERROR_OverVoltage;
        }

        // 2.欠压保护
        if (sguan->Safe.Vbus_Real <= sguan->Safe.Vbus_Min){
            sguan->Status = STATUS_Standby;
            sguan->Error_Code = ERROR_UnderVoltage;
        }

        // 3.过温保护
        if (sguan->Safe.Temp_Real >= sguan->Safe.Temp_Max){
            sguan->Status = STATUS_Standby;
            sguan->Error_Code = ERROR_OverTemp;
        }

        // 4.低温保护
        if (sguan->Safe.Temp_Real <= sguan->Safe.Temp_Min){
            sguan->Status = STATUS_Standby;
            sguan->Error_Code = ERROR_UnderTemp;
        }

        // 5.过流保护
        if (sguan->Safe.Ibus_Real >= sguan->Safe.Ibus_Max){
            sguan->Status = STATUS_Standby;
            sguan->Error_Code = ERROR_OverCurrent;
        }
    }
}

// ================================= (main) ===================================
static void main_Transfer_Init(SguanFOC_System_STRUCT *sguan){
    // 1.用户数据初始化
    User_MotorSet();
    User_ParameterSet();

    // 2.Transfer传递函数初始化
    // (LPF)
    sguan->Transfer.LPF_D.T = PMSM_RUN_T;
    LPF_Init(&sguan->Transfer.LPF_D);

    sguan->Transfer.LPF_Q.T = PMSM_RUN_T;
    LPF_Init(&sguan->Transfer.LPF_Q);

    sguan->Transfer.LPF_Speed.T = PMSM_RUN_T;
    LPF_Init(&sguan->Transfer.LPF_Speed);

    // (LTD)
    sguan->Transfer.LPF_Ltd.T = PMSM_RUN_T;
    LPF_Init(&sguan->Transfer.LPF_Ltd);

    // (PID)
    sguan->Transfer.PID_D.id = 0;
    sguan->Transfer.PID_D.T = PMSM_RUN_T;
    sguan->Transfer.PID_D.IntMax = 
        (sguan->Transfer.PID_D.OutMax/sguan->Transfer.PID_D.Ki);
    sguan->Transfer.PID_D.IntMin = 
        (sguan->Transfer.PID_D.OutMin/sguan->Transfer.PID_D.Ki);
    PID_Init(&sguan->Transfer.PID_D);

    sguan->Transfer.PID_Q.id = 0;
    sguan->Transfer.PID_Q.T = PMSM_RUN_T;
    sguan->Transfer.PID_Q.IntMax = 
        (sguan->Transfer.PID_Q.OutMax/sguan->Transfer.PID_Q.Ki);
    sguan->Transfer.PID_Q.IntMin = 
        (sguan->Transfer.PID_Q.OutMin/sguan->Transfer.PID_Q.Ki);
    PID_Init(&sguan->Transfer.PID_Q);

    sguan->Transfer.PID_Speed.id = 1;
    sguan->Transfer.PID_Speed.T = PMSM_RUN_T*sguan->Transfer.Response;
    sguan->Transfer.PID_Speed.IntMax = 
        (sguan->Transfer.PID_Speed.OutMax/sguan->Transfer.PID_Speed.Ki);
    sguan->Transfer.PID_Speed.IntMin = 
        (sguan->Transfer.PID_Speed.OutMin/sguan->Transfer.PID_Speed.Ki);
    PID_Init(&sguan->Transfer.PID_Speed);

    // (PLL)
    sguan->Transfer.PLL.T = PMSM_RUN_T;
    PLL_Init(&sguan->Transfer.PLL);
        
    // (NLFO)
    sguan->Transfer.NLFO.T = PMSM_RUN_T;
    sguan->Transfer.NLFO.Rs = sguan->Motor.Rs;
    sguan->Transfer.NLFO.Ls = (sguan->Motor.Ld + sguan->Motor.Lq)/2.0f;
    sguan->Transfer.NLFO.Flux = sguan->Motor.Flux;
    NLFO_Init(&sguan->Transfer.NLFO);

    // 3.消除编译警告
    (void)Transfer_PID_Loop;
    (void)Transfer_NLFO_Loop;
    (void)Feedforward_CurrentD;
    (void)Feedforward_CurrentQ;
    (void)Float_DataSwitch_Loop;
}

static void main_Current_Init(SguanFOC_System_STRUCT *sguan){
    #if CONFIG_CUR
    // 1.读取电流偏置
    uint32_t sum0 = 0, sum1 = 0, sum2 = 0;
    uint8_t n = 8;   // 外层：8 组
    uint8_t m = 8;   // 内层：每组 8 次
    for (uint8_t i = 0; i < n; i++) {
        for (uint8_t j = 0; j < m; j++) {
            sum0 += User_ReadADC_Raw(0);
            sum1 += User_ReadADC_Raw(1);
            sum2 += User_ReadADC_Raw(2);
        }
        User_Delay(1);   // 每组之间延时
    }

    // 2.计算电流偏置
    sguan->Motor.Current_Offset0 = (int16_t)(((float)sum0)/((float)(n*m)));
    sguan->Motor.Current_Offset1 = (int16_t)(((float)sum1)/((float)(n*m)));
    sguan->Motor.Current_Offset2 = (int16_t)(((float)sum2)/((float)(n*m)));
    #else // CONFIG_CUR
    sguan->Motor.Current_Offset0 = 0;
    sguan->Motor.Current_Offset1 = 0;
    sguan->Motor.Current_Offset2 = 0;
    #endif // CONFIG_CUR
}

static void main_loop(SguanFOC_System_STRUCT *sguan){
    if (sguan->Status == STATUS_Ready){
        // 1.用户初始化程序运行
        // (并立马切换状态机到初始化位)
        sguan->Status = STATUS_Initializing0;
        User_StartInit();

        // 2.初始化模块
        main_Transfer_Init(sguan);

        // 3.电流偏置计算
        main_Current_Init(sguan);

        // 4.状态机切换到无感观测器启动
        // (但是控制器"速度环"暂时不启动)
        // (控制器电流环启动)
        sguan->Status = STATUS_Initializing1;
        User_Delay(500);
        
        // 5.所有控制器全部启动，电机正常工作
        sguan->Status = STATUS_COM;
        sguan->Error_Code = ERROR_Zreo;
    }
    if (sguan->Status == STATUS_COM){
        Printf_TX_Loop(&sguan->Printf);
    }
}


// ================================= (SguanFOC) ===================================
void SguanFOC_High_Loop(void){
    if (Sguan.Status >= STATUS_Initializing1){
        // 1.Q31/Flaot数据实时转换
        #if CONFIG_Float
        Float_DataSwitch_Loop(&Sguan);
        #endif // CONFIG_Float

        // 2.电流计算任务
        High_Current_Loop(&Sguan);

        // 3.角度计算任务
        High_Encoder_Loop(&Sguan);

        // 4.控制器计算任务
        High_Control_Loop(&Sguan);
        
        // 5.占空比计算任务
        High_PWM_Loop(&Sguan);
    }
}

void SguanFOC_Low_Loop(void){
    // 1.读取实时数据
    Low_DataRead_Loop(&Sguan);

    // 2.状态机切换做保护
    Low_StatusSwitch_Loop(&Sguan);
}

void SguanFOC_Printf_Loop(uint8_t *data, uint16_t length){
    // 微控制器接收来自上位机的消息
    // 解析数据的格式like：AO=16.8?
    Printf_RX_Loop(data,length);
}

void SguanFOC_main_Loop(void){
    static uint8_t count = 0;
    if (count == 0){
        User_InitialInit();
        Printf_TX_Init(&Sguan.Printf);
        Printf_RX_Init();

        count = 1;
    }
    main_loop(&Sguan);
}


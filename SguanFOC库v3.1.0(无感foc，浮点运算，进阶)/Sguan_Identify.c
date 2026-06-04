/*
 * @Author: 星必尘Sguan
 * @GitHub: https://github.com/Sguan-ZhouQing
 * @Date: 2026-04-22 01:53:19
 * @LastEditors: 星必尘Sguan|3464647102@qq.com
 * @LastEditTime: 2026-06-05 03:46:51
 * @FilePath: \SguanFOC_Debug\SguanFOC\Sguan_Identify.c
 * @Description: SguanFOC库的“电机参数辨识”实现
 * @Reminder: 针对Rs、Ls(Ld，Lq)和Flux
 * 
 * Copyright (c) 2026 by $星必尘Sguan, All Rights Reserved. 
 */
#include "Sguan_Identify.h"

/* UserData配置文件声明 */
#include "UserData_Function.h"

/**
 * @description: 电机参数辨识放到初始化中运行的函数
 * @reminder: (读取电机相电阻(Rs = ΔU / ΔI))
 * @param {IDENTIFY_STRUCT} *identify
 * @return {*}
 */
void Identify_ReadRs_Init(IDENTIFY_STRUCT *identify){
    // 电机相电阻参数辨识
    // (需要SVPWM/SPWM使能，电流计算使能)
    // (需要编码器使能，Init_Tick()运行会刷新掉角度值)
    if ((CONFIG_Identify == 1) || (CONFIG_Identify == 2)){
        // 1.D轴注入一半Uh并等待电机稳定
        // （需提前定位好电机角度到零位）
        identify->go.Output_Ud = identify->go.Set_Uh/2.0f;
        User_Delay((uint32_t)identify->go.Set_Delay);

        // 2.采集此时的D轴电流
        float Current = identify->go.Input_Id;

        // 3.D轴注入全部Uh并等待电机稳定
        identify->go.Output_Ud = identify->go.Set_Uh;
        User_Delay((uint32_t)identify->go.Set_Delay);

        // 4.采集此时的D轴电流
        identify->go.Current_h = identify->go.Input_Id;

        // 5.计算电机相电阻并切换Step标志位
        identify->Rs = identify->go.Set_Uh/
            (2.0f*(identify->go.Current_h - Current));
        identify->go.Step = 1;

        // 6.延时为电感参数辨识提高时间
        User_Delay((uint32_t)(identify->go.Set_Delay*10.0f));
    }
}

/**
 * @description: 电机参数辨识放到初始化中运行的函数
 * @reminder: (读取电机磁链(空载测E0/We,E0=Uq-Iq*Rs))
 * @param {IDENTIFY_STRUCT} *identify
 * @return {*}
 */
void Identify_ReadFlux_Init(IDENTIFY_STRUCT *identify){
    // 电机Flux磁链参数
    // (需要SVPWM/SPWM使能，电流计算使能)
    // (需要编码器使能，Init_Tick()运行不刷新角度值)
    if (CONFIG_Identify == 3){
        // 1.Q轴注入全部Us并等待电机稳定
        identify->go.Output_Uq = identify->go.Set_Us;

        // 2.延时10倍的Set_Delay的时间
        User_Delay((uint32_t)(identify->go.Set_Delay*10.0f));

        // 3.此时计算磁链大小
        // （此时电机稳定）
        // （辨识途中需要保持PWM计算使能，控制器失能）
        identify->Flux = 
            (identify->go.Output_Uq - 
            identify->go.Input_Iq*identify->Rs)/
            identify->go.Input_We;
    }
}

/**
 * @description: 电机参数辨识放到离散周期中运行的函数
 * @reminder: (读取电机相电感(L = R * t / ln(20)))
 * @param {IDENTIFY_STRUCT} *identify
 * @return {*}
 */
void Identify_ReadLs_Loop(IDENTIFY_STRUCT *identify){
    // 1.如果无电机极性辨识进程则直接不参与下面计算
    if ((identify->go.Step >= 2) || (!CONFIG_Identify)){
        return;
    }

    // 2.电机电感参数辨识
    // (需要SVPWM/SPWM使能，电流计算使能)
    // (需要编码器使能，Init_Tick()运行会刷新掉角度值)
    if (identify->go.Step == 1){
        if (CONFIG_Identify == 1){
            // 1.D轴注入全部Uh并记录电流达到时间
            identify->go.Output_Ud = identify->go.Set_Uh;
            identify->go.Time++;

            // 2.若达95%预定电流则计算Ls
            if (identify->go.Input_Id >= identify->go.Current_h*0.95f){
                identify->Ld = identify->Rs*
                            identify->go.Time*
                            (float)PMSM_RUN_T*
                            (float)Value_INV_ln20;
                identify->Lq = identify->Ld;
                identify->go.Output_Ud = 0.0f;
                identify->go.Time = 0;
                identify->go.Step = 2;
            }
        }
        else{
            static uint8_t Ls_Flag = 0;
            if (Ls_Flag == 0){
                // 1.D轴注入全部Uh并记录电流达到时间
                identify->go.Output_Ud = identify->go.Set_Uh;
                identify->go.Time++;

                // 2.若达95%预定电流则计算Ld
                if (identify->go.Input_Id >= identify->go.Current_h*0.95f){
                    identify->Ld = identify->Rs*
                                identify->go.Time*
                                (float)PMSM_RUN_T*
                                (float)Value_INV_ln20;
                    identify->go.Output_Ud = 0.0f;
                    identify->go.Time = 0;
                    Ls_Flag = !Ls_Flag;
                }
            }
            else{
                // 3.Q轴注入全部Uh并记录电流达到时间
                // （使用当前锁定的位置记录电流）
                identify->go.Output_Uq = identify->go.Set_Uh;
                identify->go.Time++;

                // 4.若达95%预定电流则计算Lq
                if (identify->go.Input_Iq >= identify->go.Current_h*0.95f){
                    identify->Lq = identify->Rs*
                                identify->go.Time*
                                (float)PMSM_RUN_T*
                                (float)Value_INV_ln20;
                    identify->go.Output_Uq = 0.0f;
                    identify->go.Time = 0;
                    identify->go.Step = 2;
                }
            }
        }
    }
}


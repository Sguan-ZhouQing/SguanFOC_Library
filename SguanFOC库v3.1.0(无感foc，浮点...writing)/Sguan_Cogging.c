/*
 * @Author: 星必尘Sguan
 * @GitHub: https://github.com/Sguan-ZhouQing
 * @Date: 2026-04-30 01:37:34
 * @LastEditors: 星必尘Sguan|3464647102@qq.com
 * @LastEditTime: 2026-04-30 01:56:37
 * @FilePath: \SguanFOC_Debug\SguanFOC\Sguan_Cogging.c
 * @Description: SguanFOC库的“抗齿槽算法(离线标定int16_t的1800点位)”实现
 * 
 * Copyright (c) 2026 by $星必尘Sguan, All Rights Reserved. 
 */
#include "Sguan_Cogging.h"

/* UserData配置文件声明 */
#include "UserData_Function.h"

// 离线标定的1800点位抗齿槽电流表
#define Cogging_N   1800
static int16_t iq_tab[Cogging_N] = {0};
// static const int16_t iq_tab[Cogging_N] = {0};

/**
 * @description: 抗齿槽离线标定的初始化函数
 * @param {COGGING_STRUCT} *cogging
 * @return {*}
 */
void Cogging_SetIq_Init(COGGING_STRUCT *cogging){
    for (int j = 0; j < (cogging->Count); j++){
        for (int i = 0 ; i < Cogging_N; i++){
            iq_tab[i] += IQmath_Q15_from_float(
                cogging->go.Input_Init_Iq/CONFIG_BASE);
            cogging->go.Output_Init_Rad += 
                (((float)Value_2PI)*i)/((float)Cogging_N);
            User_Delay((uint32_t)(cogging->Cycle/((float)Cogging_N)));
        }
    }
    for (int k = 0; k < Cogging_N; k++){
        iq_tab[k] /= cogging->Count;
    }
}

/**
 * @description: 超螺旋滑模DOB的离散运行函数
 * @reminder: (离线标定法是通过预先测量和记录电机在不同位置的齿槽效应数据)
 * @reminder: (在运行时，根据当前电机位置从查找表中读取对应的补偿电流值)
 * @param {COGGING_STRUCT} *cogging
 * @return {*}
 */
void Cogging_ReadIq_Loop(COGGING_STRUCT *cogging){
    uint16_t K = cogging->go.Input_Loop_Rad*
                ((float)Cogging_N)/((float)Value_2PI);

    cogging->go.Output_Loop_Iq = 
                IQmath_Q15_to_float(iq_tab[K])*CONFIG_BASE;
}



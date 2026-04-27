/*
 * @Author: 星必尘Sguan
 * @GitHub: https://github.com/Sguan-ZhouQing
 * @Date: 2026-02-26 22:37:25
 * @LastEditors: 星必尘Sguan|3464647102@qq.com
 * @LastEditTime: 2026-04-19 01:00:46
 * @FilePath: \SguanFOC_Debug\SguanFOC\Sguan_Optimize.c
 * @Description: SguanFOC库的“MTPA、FW弱磁和死区补偿的电机优化算法”实现
 * 
 * Copyright (c) 2026 by $星必尘Sguan, All Rights Reserved. 
 */
#include "Sguan_Optimize.h"

/**
 * @description: 经典最大转矩电流id计算公式
 * @param {float} *Target_id
 * @param {float} flux
 * @param {float} Ld
 * @param {float} Lq
 * @param {float} iq
 * @return {*}
 */
void MTPA_Loop(float *Target_id, 
            float flux, 
            float Ld, 
            float Lq, 
            float iq){
    // 1.运行中间关键变量
    float num = Ld - Lq;
    float temp = Value_sqrtf(flux*flux + 
                8*num*num*iq*iq);
    
    // 2.输出MTPA的目标D轴电流
    *Target_id = -((flux + temp)/
                (num*4));
}


void FW_Loop(void){
    
}

void DeadZone_Loop(float *Ua_error, 
                float *Ub_error, 
                float *Uc_error, 
                float Ia, 
                float Ib, 
                float Ic, 
                float VUBS,
                float Dead_Time){
    // 1.计算补偿增益的大小
    static float value = 0.0f;
    if (!value){
        value = VUBS*(Dead_Time/((float)PMSM_RUN_T));
    }
    
    // 2.输出三相补偿量
    *Ua_error = Value_Sign(Ia)*value;
    *Ub_error = Value_Sign(Ib)*value;
    *Uc_error = Value_Sign(Ic)*value;
}


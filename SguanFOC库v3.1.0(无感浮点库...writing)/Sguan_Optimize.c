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
#include "Sguan_PID.h"

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
            float iq) {
    static float delta_L = 0.0f;
    if (!delta_L){
        delta_L = Lq - Ld;
    }
    float temp = Value_sqrtf(flux*flux + 4.0f*delta_L*delta_L*iq*iq);
    
    // 计算标准MTPA公式并输出
    *Target_id = (flux - temp) / (2.0f * delta_L);
    if (*Target_id > 0) *Target_id = 0;
}

/**
 * @description: 弱磁控制的PI控制器运算
 * @param {void} *fw
 * @param {float} Ud
 * @param {float} Uq
 * @param {float} Percentage
 * @param {float} Vbus
 * @return {*}
 */
float FW_Loop(void *fw, 
            float Ud, 
            float Uq, 
            float Percentage, 
            float Vbus){
    static float fbk = 0.0f;
    if (!fbk){
        fbk = Percentage*Vbus*Value_SQRT3_2;
    }

    PID_STRUCT *p = (PID_STRUCT*)fw;
    p->run.Ref = Value_sqrtf(Ud*Ud + Uq*Uq);
    p->run.Fbk = fbk;
    PID_Loop(p);
    return -p->run.Output;
}

/**
 * @description: 简化后的死区补偿算法
 * @param {float} *Ua_duty
 * @param {float} *Ub_duty
 * @param {float} *Uc_duty
 * @param {float} Ia
 * @param {float} Ib
 * @param {float} Ic
 * @param {float} Current_Min
 * @param {float} Dead_Time
 * @return {*}
 */
void DeadZone_Loop(float *Ua_duty, 
            float *Ub_duty, 
            float *Uc_duty, 
            float Ia, 
            float Ib, 
            float Ic, 
            float Current_Min, 
            float Dead_Time){
    // 1.计算补偿增益的大小
    static float value = 0.0f;
    if (!value){
        value = Dead_Time/((float)PMSM_RUN_T);
    }
    
    // 2.输出三相死区补偿量
    // 过零点附近不补偿，避免误判
    if ((Ia < -Current_Min) || 
        (Ia > Current_Min)){
        *Ua_duty += Value_Sign(Ia)*value;
    }
    if ((Ib < -Current_Min) || 
        (Ib > Current_Min)){
        *Ub_duty += Value_Sign(Ib)*value;
    }
    if ((Ic < -Current_Min) || 
        (Ic > Current_Min)){
        *Uc_duty += Value_Sign(Ic)*value;
    }
}


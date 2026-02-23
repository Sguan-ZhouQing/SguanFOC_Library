/*
 * @Author: 星必尘Sguan
 * @GitHub: https://github.com/Sguan-ZhouQing
 * @Date: 2026-01-26 22:38:09
 * @LastEditors: 星必尘Sguan|3464647102@qq.com
 * @LastEditTime: 2026-02-16 00:39:48
 * @FilePath: \stm_SguanFOCtest\SguanFOC\Sguan_PID.c
 * @Description: SguanFOC库的“电机极性辨识NSD算法”实现
 * 
 * Copyright (c) 2026 by $星必尘Sguan, All Rights Reserved. 
 */
#include "Sguan_NSD.h"

/**
 * @description: 电机转子极性辨识NSD函数实现
 * @param {float} input 当前d轴电流高频分量Id_h
 * @param {float} *Ud   用于输出的d轴电压偏置
 * @reminder:0 - 转子在0度电角度处；1 - 转子在180度电角度处
 * @return {*}
 */
uint8_t NSD_Loop(float input, float Ud_Bias, float *Ud){
    static uint16_t NSD_Count = 0;
    static float sum1 = 0, sum2 = 0;
    static uint8_t init_done = 0;
    static uint8_t state = 0;

    // 如果已经完成极性辨识，直接返回结果
    if(init_done == 1){
        return init_done - 1; // 0或1
    }
    
    NSD_Count++;
    switch(state){
        case 0: // 第一阶段：0电压注入 (0-399)
            if(NSD_Count < 400){
                *Ud = 0.0f;
            }
            else{
                state = 1;
                NSD_Count = 400;
            }
            break;
        case 1: // 第二阶段：正电压注入 (400-599)
            if(NSD_Count < 600){
                *Ud = Ud_Bias;
            }
            else{
                state = 2;
                NSD_Count = 600;
            }
            break;
        case 2: // 第三阶段：正电压注入+采样 (600-609)
            if(NSD_Count < 610){
                *Ud = Ud_Bias;
                sum1 += fabs(input * 4.0f);
            }
            else{
                state = 3;
                NSD_Count = 610;
            }
            break;
        case 3: // 第四阶段：0电压注入 (610-809)
            if(NSD_Count < 810){
                *Ud = 0.0f;
            }
            else{
                state = 4;
                NSD_Count = 810;
            }
            break;
        case 4: // 第五阶段：负电压注入 (810-1009)
            if(NSD_Count < 1010){
                *Ud = -Ud_Bias;
            }
            else{
                state = 5;
                NSD_Count = 1010;
            }
            break;
        case 5: // 第六阶段：负电压注入+采样 (1010-1019)
            if(NSD_Count < 1020){
                *Ud = -Ud_Bias;
                sum2 += fabs(input * 4.0f);
            }
            else{
                state = 6;
                NSD_Count = 1020;
            }
            break;
        case 6: // 第七阶段：回归0电压，判断极性 (1020+)
            *Ud = 0.0f;
            if(sum2 > sum1){
                // 转子在180度电角度处
                init_done = 2; // 返回1
            }
            else{
                // 转子在0度电角度处
                init_done = 1; // 返回0
            }
            break;
        default:
            *Ud = 0.0f;
            break;
    }
    
    if(init_done != 0){
        return init_done - 1;
    }
    else{
        return 2; // 正在辨识中
    }
}


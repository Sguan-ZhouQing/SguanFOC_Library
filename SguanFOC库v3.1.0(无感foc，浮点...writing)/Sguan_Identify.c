/*
 * @Author: 星必尘Sguan
 * @GitHub: https://github.com/Sguan-ZhouQing
 * @Date: 2026-04-22 01:53:19
 * @LastEditors: 星必尘Sguan|3464647102@qq.com
 * @LastEditTime: 2026-04-30 01:57:17
 * @FilePath: \SguanFOC_Debug\SguanFOC\Sguan_Identify.c
 * @Description: SguanFOC库的“电机参数辨识”实现
 * @Reminder: 针对Encoder_Dir、Rs、Ls和B、J、Flux
 * 
 * Copyright (c) 2026 by $星必尘Sguan, All Rights Reserved. 
 */
#include "Sguan_Identify.h"

// 判断电机参数辨识的函数
static uint8_t Identify_RETURN_STATUS(IDENTIFY_STRUCT *identify){
    if (identify->pmsm.Status == IDENTIFY_FINISHED){
        return 1;
    }
    return 0;
}

// 参数辨识Part(包含Encoder_Dir、Rs、Ls)
static uint8_t Identify_READING_Part(IDENTIFY_STRUCT *identify){

    return Identify_RETURN_STATUS(identify);
}

// 参数辨识All(所有参数一起辨识，时间长)
static uint8_t Identify_READING_All(IDENTIFY_STRUCT *identify){


    return Identify_RETURN_STATUS(identify);
}

/**
 * @description: MOTOR参数辨识的初始化函数
 * @param {IDENTIFY_STRUCT} *identify
 * @return {*}
 */
void Identify_Init(IDENTIFY_STRUCT *identify){
    
}


/**
 * @description: 电机参数辨识离散运行函数
 * @param {IDENTIFY_STRUCT} *identify
 * @return {*}
 */
uint8_t Identify_Loop(IDENTIFY_STRUCT *identify){
    switch (CONFIG_Identify){
    case 0:
        identify->pmsm.Status = IDENTIFY_FINISHED;
        return 1;
    case 1:
        if (Identify_READING_Part(identify)){
            return 1;
        }
    case 2:
        if (Identify_READING_All(identify)){
            return 1;
        }
    default:
        break;
    }
    return 0;
}


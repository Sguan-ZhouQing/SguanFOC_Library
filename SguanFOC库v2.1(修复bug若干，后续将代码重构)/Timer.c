/*
 * @Author: 星必尘Sguan
 * @Date: 2025-05-08 19:26:48
 * @LastEditors: 星必尘Sguan|3464647102@qq.com
 * @LastEditTime: 2026-01-25 22:13:37
 * @FilePath: \demo_SguanFOCCode\Hardware\Timer.c
 * @Description: TIM定时中断统一管理函数;
 * 
 * Copyright (c) 2025 by $JUST, All Rights Reserved. 
 */
#include "Timer.h"
#include "adc.h"
#include "SguanFOC.h"

uint32_t ADC_InjectedValues[4];
uint8_t flag = 0;

/**
 * @description: TIM2中断回调函数，1ms的定时器定时中断;
 * @param {TIM_HandleTypeDef*} htim
 * @return {*}
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim == &htim1) {  // 20kHz中断
        HAL_ADCEx_InjectedStart_IT(&hadc2);
        // HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_14);
    }
    if (htim == &htim2) {  // 1kHz中断
        SguanFOC_Run_Tick();
        HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_15);
    }
}

void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc)
{		
	ADC_InjectedValues[0] = ADC2->JDR1;           //母线电压
	ADC_InjectedValues[1] = ADC2->JDR2;           //获取A相电流
	ADC_InjectedValues[2] = ADC2->JDR3;          //获取C相电流
 	ADC_InjectedValues[3] = ADC2->JDR4;//使用波轮电位器给电机目标转速
    if (flag)
    {
        /* code */
        SguanFOC_GeneratePWM_Loop();
    }
    
}

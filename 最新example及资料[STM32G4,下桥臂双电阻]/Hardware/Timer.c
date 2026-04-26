/*
 * @Author: 星必尘Sguan
 * @Date: 2025-05-08 19:26:48
 * @LastEditors: 星必尘Sguan|3464647102@qq.com
 * @LastEditTime: 2026-02-21 01:14:10
 * @FilePath: \stm_SguanFOCtest\Hardware\Timer.c
 * @Description: TIM定时中断统一管理函数;
 * 
 * Copyright (c) 2025 by $JUST, All Rights Reserved. 
 */
#include "Timer.h"
#include "adc.h"
#include "SguanFOC.h"
#include "main.h"

extern UART_HandleTypeDef huart1;
volatile uint32_t ADC_InjectedValues[4] = {0};

/**
 * @description: TIM中断回调函数，1ms和100us的定时器定时中断;
 * @param {TIM_HandleTypeDef*} htim
 * @return {*}
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim == &htim1){  // 10kHz中断
        HAL_ADCEx_InjectedStart_IT(&hadc2);
    }
    if (htim == &htim2){  // 1kHz中断
        HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_15);
        SguanFOC_Low_Loop();
	}
}

/**
 * @description: ADCz注入转换完成回调函数，1ms的定时器定时中断;
 * @param {ADC_HandleTypeDef} *hadc
 * @return {*}
 */
void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc){		
	ADC_InjectedValues[0] = ADC2->JDR1;           //母线电压
	ADC_InjectedValues[1] = ADC2->JDR2;           //获取A相电流
	ADC_InjectedValues[2] = ADC2->JDR3;          //获取C相电流
 	ADC_InjectedValues[3] = ADC2->JDR4;//使用波轮电位器给电机目标转速
	// Sguan.foc.Uq_in = ADC_InjectedValues[3]*10.0f/4096.0f; // 这里直接把波轮电位器的值映射到Uq_in上
	// Sguan.foc.Target_Iq = ADC_InjectedValues[3]*6.5f/4096.0f; // 这里直接把波轮电位器的值映射到Uq_in上
	SguanFOC_High_Loop();
    HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_3);
}

/**
 * @description: UART接收完成回调函数，用于处理串口接收的数据;
 * @param {UART_HandleTypeDef} *huart
 * @param {uint16_t} Size
 * @return {*}
 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
    if (huart->Instance == USART1) {
        // 直接调用Printf_ProcessData处理接收到的数据
        SguanFOC_Printf_Loop(Sguan_PrintfBuff, Size);
        
        // 重新启动DMA接收
        HAL_UARTEx_ReceiveToIdle_DMA(&huart1, Sguan_PrintfBuff, sizeof(Sguan_PrintfBuff));
        __HAL_DMA_DISABLE_IT(huart1.hdmarx, DMA_IT_HT);
    }
}

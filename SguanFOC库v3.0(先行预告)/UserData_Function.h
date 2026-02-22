#ifndef __USERDATA_FUNCTION_H
#define __USERDATA_FUNCTION_H
#include <stdint.h>
/* 电机控制User用户设置·功能接口 */
/* 用户自己的CODE BEGIN Includes */
#include "SguanFOC.h"
#include "main.h"

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern volatile uint32_t ADC_InjectedValues[4];
/* 用户自己的CODE END Includes */

static inline void User_InitialInit(void){
    /* Your code for initing TIM and gate driver and encoder and ADC here */
    // 开启SD使能栅极驱动器
    HAL_GPIO_WritePin(SD1_GPIO_Port,SD1_Pin,GPIO_PIN_SET);
    HAL_GPIO_WritePin(SD2_GPIO_Port,SD2_Pin,GPIO_PIN_SET);
    HAL_GPIO_WritePin(SD3_GPIO_Port,SD3_Pin,GPIO_PIN_SET);
    // 开启PWM输出
    HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_3);
    HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
    // 设置TIM3计数器初始值
    __HAL_TIM_SET_COUNTER(&htim3, 0);
}

static inline void User_Delay(uint32_t ms){
    /* Your code for Delay_ms here */
    HAL_Delay(ms);
}

static inline int32_t User_ReadADC_Raw(uint8_t Current_CH){
    // 电流采样通道0->AB相，1->AC相，2->BC相
    // Sguan.motor.Current_Num值自定义采样通道
    // 用户在UserData_Motor.h中定义“这个值”
    int32_t ADC_num = 0;
    switch (Current_CH){
    case 0:
        /* Your code for Motor CH0 raw */
        ADC_num = (int32_t)ADC_InjectedValues[1];
        break;
    case 1:
        /* Your code for Motor CH1 raw */
        ADC_num = (int32_t)ADC_InjectedValues[2];
        break;
    default:
        break;
    }
    return ADC_num;
}

static inline float User_Encoder_ReadRad(void){
    float Rad_num = 0.0f;
    /* Your code for encoder radian position (0-2pi) if you use SensorFOC */
    Rad_num = SS_Encoder_GetRad();
    return Rad_num;
}

static inline void User_PwmDuty_Set(uint16_t Duty_u,uint16_t Duty_v,uint16_t Duty_w){
    /* Your code for Motor PWM_CH0~2 duty set */
    /* 若是使用正常6相，设置死区补偿，可在此设计 */
    __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,Duty_u);
    __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,Duty_v);
    __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,Duty_w);
}

static inline float User_VBUS_DataGet(void){
    // float VBUS_num = 0.0f;
    /* Your code for motor VBUS_Voltage Data return if you use it */
    
    // 如果不使用电压功能，返回-9999.0f（正常电压不会是负数）
    return -9999.0f;
}

static inline float User_Temperature_DataGet(void){
    // float Temp_num = 0.0f;
    /* Your code for motor Temperature Data return if you use it */
    
    // 如果不使用温度功能，返回-9999.0f（正常温度不会是这么大的负数）
    return -9999.0f;
}


#endif // USERDATA_FUNCTION_H

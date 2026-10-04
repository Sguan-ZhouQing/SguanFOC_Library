#ifndef __USERDATA_FUNCTION_H
#define __USERDATA_FUNCTION_H
#include <stdint.h>
/* 电机控制User用户设置·功能接口 */

// .................................................................
#include "main.h"
#include "SguanFOC.h"

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern UART_HandleTypeDef huart1;
extern volatile uint32_t ADC_InjectedValues[4];
// .................................................................


static inline void User_InitialInit(void){
    /* Your code for SguanFOC here */

    // ..............................................................
    // 初始化定时器中断
    HAL_TIM_Base_Start_IT(&htim1);
    HAL_TIM_Base_Start_IT(&htim3);
    // 启用串口DMA接收
    HAL_UARTEx_ReceiveToIdle_DMA(&huart1, Sguan_PrintfBuff, sizeof(Sguan_PrintfBuff));
    __HAL_DMA_DISABLE_IT(huart1.hdmarx, DMA_IT_HT);
    // ..............................................................
}

static inline void User_StartInit(void){
    /* Your code for SguanFOC here */

    // ..............................................................
    // 开启SD使能栅极驱动器
    HAL_GPIO_WritePin(SD_GPIO_Port,SD_Pin,GPIO_PIN_SET);
    // 开启PWM输出
    HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_3);
    HAL_TIMEx_PWMN_Start(&htim1,TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Start(&htim1,TIM_CHANNEL_2);
    HAL_TIMEx_PWMN_Start(&htim1,TIM_CHANNEL_3);
    __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,3000);
    __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,3000);
    __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,3000);

    HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
    // 设置TIM3计数器初始值
    __HAL_TIM_SET_COUNTER(&htim2, 0);
    // ..............................................................
}

static inline void User_Delay(uint32_t ms){
    /* Your code for SguanFOC here */
    
    // .............................................................
    HAL_Delay(ms);
    // .............................................................
}

static inline uint16_t User_ReadADC_Raw(uint8_t Current_CH){
    /* Your code for SguanFOC here */
    uint16_t ADC_num = 0;
    switch (Current_CH){
    case 0:

        // ........................................................
        ADC_num = (int32_t)ADC_InjectedValues[0];
        // ........................................................
        break;
    case 1:

        // ........................................................
        ADC_num = (int32_t)ADC_InjectedValues[1];
        // ........................................................
        break;
    case 2:

        // ........................................................
        ADC_num = (int32_t)ADC_InjectedValues[2];
        // ........................................................
        break;
    default:
        break;
    }
    return ADC_num;
}

static inline uint16_t User_VBUS_DataGet(void){
    /* Your code for SguanFOC here */
    
    return 10;
}

static inline uint16_t User_Temperature_DataGet(void){
    /* Your code for SguanFOC here */

    return 10;
}

static inline void User_PwmDuty_Set(uint16_t Duty_u,
                                uint16_t Duty_v,
                                uint16_t Duty_w){
    /* Your code for SguanFOC here */

    // ............................................................
    __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,Duty_u);
    __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,Duty_v);
    __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,Duty_w);
    // ............................................................
}

/* ================= 驱动代码(驱动层) ================= */
static inline void User_CorrespondSet(unsigned char *ch, unsigned short int size){
    /* Your code for SguanFOC here */

    // ................................................
    HAL_UART_Transmit(&huart1, ch, size, 0xFFFF);
    // ................................................
}


#endif // USERDATA_FUNCTION_H

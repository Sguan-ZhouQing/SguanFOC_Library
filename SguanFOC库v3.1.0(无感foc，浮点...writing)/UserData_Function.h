#ifndef __USERDATA_FUNCTION_H
#define __USERDATA_FUNCTION_H
#include <stdint.h>
/* 电机控制User用户设置·功能接口 */
/* 用户自己的CODE BEGIN Includes */
#include "main.h"
#include "SguanFOC.h"

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern UART_HandleTypeDef huart1;
extern volatile uint32_t ADC_InjectedValues[4];
/* 用户自己的CODE END Includes */

/**
 * @description: 1.电机上电即初始化的函数接口
 * @reminder: (此方函数->填入一些最先初始化的代码)
 * @reminder: (比如定时器中断开启、串口接收开启)
 * @return {*}
 */
static inline void User_Initial_Init(void){
    /* Your code for Initializing immediately upon powering on here */
    // 初始化定时器中断
    HAL_TIM_Base_Start_IT(&htim1);
    HAL_TIM_Base_Start_IT(&htim2);
    // 启用串口DMA接收
    HAL_UARTEx_ReceiveToIdle_DMA(&huart1, Sguan_PrintfBuff, sizeof(Sguan_PrintfBuff));
    __HAL_DMA_DISABLE_IT(huart1.hdmarx, DMA_IT_HT);
}

/**
 * @description: 2.有启动信号后才初始化的函数
 * @reminder: (此方函数->填入电机启动后才初始化的代码)
 * @reminder: (比如使能栅极驱动、开启PWM输出)
 * @return {*}
 */
static inline void User_StartMotor_Init(void){
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

/**
 * @description: 3.用户系统的Delay函数对接
 * @reminder: (此方函数->填入你自己单片机的延时函数)
 * @reminder: (用于某些初始化场景的时机等待)
 * @param {unsigned int} ms
 * @return {*}
 */
static inline void User_Delay(uint32_t ms){
    /* Your code for Delay_ms here */
    HAL_Delay(ms);
}

/**
 * @description: 4.用户读取的ADC原始数据传入函数
 * @reminder: (此方函数->填入你电机采样的ADC原始数据)
 * @reminder: (比如12位数据就是0-4095)
 * @param {int32_t} Current_CH
 * @return {*}
 */
static inline int32_t User_ReadADC_Raw(int32_t Current_CH){
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

/**
 * @description: 5.用户的高精度编码器函数接口
 * @reminder: (此方函数->填入你自己的编码器原始数据函数)
 * @reminder: (原始数据是略微处理后，输出0-2pi的数值)
 * @reminder: (如果你使用的霍尔模式或者无感，就无需填写)
 * @return {*}
 */
static inline float User_Encoder_ReadRad(void){
    float Rad_num = 0.0f;
    /* Your code for encoder radian position (0-2pi) if you use Absolute SensorFOC */
    Rad_num = SguanDemo_Encoder_GetRad();
    return Rad_num;
}

/**
 * @description: 6.用户的三霍尔编码器接口
 * @reminder: (此方函数->填入你自己的霍尔信号读取函数)
 * @reminder: (数据是三个电平的高低)
 * @reminder: (如果你使用的高精度编码器或者无感，就无需填写)
 * @param {uint8_t} CH
 * @return {*}
 */
static inline uint8_t User_Encoder_ReadHall(uint8_t CH){
    uint8_t Signal = 0;
    /* Your code for encoder UVW Signal if you use Hall SensorFOC */
    switch (CH){
    case 0:
        /* Your code for Hall CH0 raw */
        // like：Signal = HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_0);
        break;
    case 1:
        /* Your code for Hall CH1 raw */
        // like：Signal = HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_1);
        break;
    case 2:
        /* Your code for Hall CH2 raw */
        // like：Signal = HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_2);
        break;
    
    default:
        break;
    }
    return Signal;
}

/**
 * @description: 7.用户的PWM驱动接口函数
 * @reminder: (此方函数->填写你自己的驱动器PWM占空比)
 * @return {*}
 */
static inline void User_PwmDuty_Set(uint32_t Duty_u,
                                uint32_t Duty_v,
                                uint32_t Duty_w){
    /* Your code for Motor PWM_CH0~2 duty set */
    __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,Duty_u);
    __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,Duty_v);
    __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,Duty_w);
}

/**
 * @description: 8.用户的驱动器母线电压读取接口
 * @reminder: (此方函数->填写驱动器母线电压滤波后的数值)
 * @return {*}
 */
static inline float User_VBUS_DataGet(void){
    // float VBUS_num = 0.0f;
    /* Your code for motor VBUS_Voltage Data return if you use it */
    
    // 如果不使用电压功能，返回0xFF800000（正常数值不会是负无穷）
    // return ADC_InjectedValues[0]*0.01795;
    return 0xFF800000;
}

/**
 * @description: 9.用户的驱动器温度读取接口
 * @reminder: (此方函数->填写驱动器温度滤波后的数值)
 * @return {*}
 */
static inline float User_Temperature_DataGet(void){
    // float Temp_num = 0.0f;
    /* Your code for motor Temperature Data return if you use it */
    
    // 如果不使用温度功能，返回0xFF800000（正常数值不会是负无穷）
    return 0xFF800000;
}

/**
 * @description: 10.用户的通信接口设计
 * @reminder: (此方函数->填写串口或者CAN的对应接口函数)
 * @param {unsigned char} *ch
 * @param {unsigned short int} size
 * @return {*}
 */
static inline void User_CorrespondSet(uint8_t *ch, uint16_t size){
    /* Your code for UART or CAN Signal Transmit Driver */
    HAL_UART_Transmit(&huart1, ch, size, 0xFFFF);
}


#endif // USERDATA_FUNCTION_H

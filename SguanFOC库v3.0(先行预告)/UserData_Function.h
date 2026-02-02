#ifndef __USERDATA_FUNCTION_H
#define __USERDATA_FUNCTION_H
#include <stdint.h>
/* 电机控制User用户设置·功能接口 */

static inline void User_InitialInit(void){
    /* Your code for initing TIM and gate driver and encoder and ADC here */
}

static inline void User_Delay(uint32_t ms){
    /* Your code for Delay_ms here */
}

static inline uint32_t User_ReadADC_Raw(uint8_t Current_CH){
    // 电流采样通道0->AB相，1->AC相，2->BC相
    // Sguan.motor.Current_Num值自定义采样通道
    // 用户在UserData_Motor.h中定义“这个值”
    uint32_t ADC_num;
    switch (Current_CH){
    case 0:
        /* Your code for Motor CH0 raw */
        break;
    case 1:
        /* Your code for Motor CH1 raw */
        break;
    default:
        break;
    }
    return ADC_num;
}

static inline float User_Encoder_ReadRad(void){
    float Rad_num;
    /* Your code for encoder radian position (0-2pi) if you use SensorFOC */
    return Rad_num;
}

static inline void User_PwmDuty_Set(uint16_t Duty_u,uint16_t Duty_v,uint16_t Duty_w){
    /* Your code for Motor PWM_CH0~2 duty set */
}

static inline float User_VBUS_DataGet(void){
    float VBUS_num;
    /* Your code for motor VBUS_Voltage Data return if you use it */
    return VBUS_num;
}

static inline float User_Temperature_DataGet(void){
    float Temp_num;
    /* Your code for motor Temperature Data return if you use it */
    return Temp_num;
}


#endif // USERDATA_FUNCTION_H

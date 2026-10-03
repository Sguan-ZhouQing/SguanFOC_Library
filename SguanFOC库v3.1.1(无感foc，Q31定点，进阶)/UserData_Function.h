#ifndef __USERDATA_FUNCTION_H
#define __USERDATA_FUNCTION_H
#include <stdint.h>
/* 电机控制User用户设置·功能接口 */


static inline void User_InitialInit(void){
    /* Your code for initing TIM and gate driver and encoder and ADC here */
}

static inline void User_StartInit(void){
    /* Your code for initing TIM and gate driver and encoder and ADC here */
}

static inline void User_Delay(uint32_t ms){
    // Delay_Ms(ms);
}

static inline uint16_t User_ReadADC_Raw(uint8_t Current_CH){
    uint16_t ADC_num = 0;
    switch (Current_CH){
    case 0:

        break;
    case 1:

        break;
    case 2:

        break;
    default:
        break;
    }
    return ADC_num;
}

static inline void User_PwmDuty_Set(uint16_t Duty_u,
                                uint16_t Duty_v,
                                uint16_t Duty_w){
    uint16_t u_ch0 = Duty_u;
    uint16_t u_ch1 = Duty_u;
    uint16_t v_ch0 = Duty_v;
    uint16_t v_ch1 = Duty_v;
    uint16_t w_ch0 = Duty_w;
    uint16_t w_ch1 = Duty_w;
    #if 0
    if (ADC_Yes == 0){
        // Du
        if ((Sguan.foc.Du < Sguan.foc.Dv) && (Sguan.foc.Du < Sguan.foc.Dw) && (Duty_n <= Min_Duty)){
            u_ch0 = Duty_u + (uint16_t)((1.0f*Min_Duty - Duty_n)*Sguan.motor.Duty);
            u_ch1 = Duty_u - (uint16_t)((1.0f*Min_Duty - Duty_n)*Sguan.motor.Duty);
        }
        if ((Sguan.foc.Du > Sguan.foc.Dv) && (Sguan.foc.Du > Sguan.foc.Dw) && (Duty_m <= Min_Duty)){
            u_ch0 = Duty_u - (uint16_t)((1.0f*Min_Duty - Duty_m)*Sguan.motor.Duty);
            u_ch1 = Duty_u + (uint16_t)((1.0f*Min_Duty - Duty_m)*Sguan.motor.Duty);
        }

        // Dv
        if ((Sguan.foc.Dv < Sguan.foc.Du) && (Sguan.foc.Dv < Sguan.foc.Dw) && (Duty_n <= Min_Duty)){
            v_ch0 = Duty_v + (uint16_t)((1.0f*Min_Duty - Duty_n)*Sguan.motor.Duty);
            v_ch1 = Duty_v - (uint16_t)((1.0f*Min_Duty - Duty_n)*Sguan.motor.Duty);
        }
        if ((Sguan.foc.Dv > Sguan.foc.Du) && (Sguan.foc.Dv > Sguan.foc.Dw) && (Duty_m <= Min_Duty)){
            v_ch0 = Duty_v - (uint16_t)((1.0f*Min_Duty - Duty_m)*Sguan.motor.Duty);
            v_ch1 = Duty_v + (uint16_t)((1.0f*Min_Duty - Duty_m)*Sguan.motor.Duty);
        }

        // Dw
        if ((Sguan.foc.Dw < Sguan.foc.Dv) && (Sguan.foc.Dw < Sguan.foc.Du) && (Duty_n <= Min_Duty)){
            w_ch0 = Duty_w + (uint16_t)((1.0f*Min_Duty - Duty_n)*Sguan.motor.Duty);
            w_ch1 = Duty_w - (uint16_t)((1.0f*Min_Duty - Duty_n)*Sguan.motor.Duty);
        }
        if ((Sguan.foc.Dw > Sguan.foc.Dv) && (Sguan.foc.Dw > Sguan.foc.Du) && (Duty_m <= Min_Duty)){
            w_ch0 = Duty_w - (uint16_t)((1.0f*Min_Duty - Duty_m)*Sguan.motor.Duty);
            w_ch1 = Duty_w + (uint16_t)((1.0f*Min_Duty - Duty_m)*Sguan.motor.Duty);
        }
    }
    #endif

    // Sguan_TIM_SetDuty(u_ch0,u_ch1, v_ch0,v_ch1, w_ch0,w_ch1);
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

/* ================= 驱动代码(驱动层) ================= */
static inline void User_CorrespondSet(unsigned char *ch, unsigned short int size){
    // Sguan_UART_SendData(ch,size,1000);
}


#endif // USERDATA_FUNCTION_H

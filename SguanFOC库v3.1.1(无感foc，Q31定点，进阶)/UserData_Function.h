#ifndef __USERDATA_FUNCTION_H
#define __USERDATA_FUNCTION_H
#include <stdint.h>
/* 电机控制User用户设置·功能接口 */

// .................................................................
#include "debug.h"
#include "Sguan_GPIO.h"
#include "Sguan_UART.h"
#include "Sguan_ADCcom.h"
#include "Sguan_ADC.h"
#include "Sguan_TIM.h"
#include "SguanFOC.h"
// .................................................................


static inline void User_InitialInit(void){
    /* Your code for SguanFOC here */

    // ..............................................................
    Sguan_GPIO_Init();
    Sguan_UART_Init();

    // ..............................................................
}

static inline void User_StartInit(void){
    /* Your code for SguanFOC here */

    // ..............................................................
    /* ADC 采样硬件在初始阶段就绪（与 FOC 状态机无关）：
     * 单电阻采样的 CH4 触发 + JEOC 八状态机由 TIM1 峰顶中断驱动，
     * 若等 FOC 状态机走到 STATUS_Ready 才初始化（原 User_StartInit），
     * 关闭 FOC 状态机（Status 不置 1）做纯 PWM 测试时，
     * u0（上升段比较值）将无人写入、移相功能失效。 */
    Sguan_ADCcom_Init();
    Sguan_ADC_Init();

    Sguan_TIM_Init();

    GPIO_SetBits(GPIOC, GPIO_Pin_1);
    GPIO_SetBits(GPIOD, GPIO_Pin_7);

    Sguan_ADC_SetDuty(2500,1400);
    // ..............................................................
}

static inline void User_Delay(uint32_t ms){
    /* Your code for SguanFOC here */
    
    // .............................................................
    Delay_Ms(ms);
    // .............................................................
}

static inline int16_t User_ReadADC_Raw(uint8_t Current_CH){
    /* Your code for SguanFOC here */
    int16_t ADC_num = 0;
    switch (Current_CH){
    case 0:
        // ........................................................
        ADC_num = MOTOR_Ix[0];
        // ........................................................
        break;
    case 1:
        // ........................................................
        ADC_num = MOTOR_Ix[1];
        // ........................................................
        break;
    case 2:
        // ........................................................
        ADC_num = 4096;
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
    uint16_t u_ch0 = Duty_u;
    uint16_t u_ch1 = Duty_u;
    uint16_t v_ch0 = Duty_v;
    uint16_t v_ch1 = Duty_v;
    uint16_t w_ch0 = Duty_w;
    uint16_t w_ch1 = Duty_w;

    #if 1
    if (ADC_Yes == 0){
        // Du
        if ((Sguan.Foc.Du < Sguan.Foc.Dv) && (Sguan.Foc.Du < Sguan.Foc.Dw) && (Duty_n <= Min_Duty)){
            u_ch0 = Duty_u + (uint16_t)(iqmath_mul((Min_Duty - Duty_n),(Q31_t)Sguan.Motor.Duty));
            u_ch1 = Duty_u - (uint16_t)(iqmath_mul((Min_Duty - Duty_n),(Q31_t)Sguan.Motor.Duty));
        }
        if ((Sguan.Foc.Du > Sguan.Foc.Dv) && (Sguan.Foc.Du > Sguan.Foc.Dw) && (Duty_m <= Min_Duty)){
            u_ch0 = Duty_u - (uint16_t)(iqmath_mul((1.0f*Min_Duty - Duty_m),(Q31_t)Sguan.Motor.Duty));
            u_ch1 = Duty_u + (uint16_t)(iqmath_mul((1.0f*Min_Duty - Duty_m),(Q31_t)Sguan.Motor.Duty));
        }

        // Dv
        if ((Sguan.Foc.Dv < Sguan.Foc.Du) && (Sguan.Foc.Dv < Sguan.Foc.Dw) && (Duty_n <= Min_Duty)){
            v_ch0 = Duty_v + (uint16_t)(iqmath_mul((1.0f*Min_Duty - Duty_n),(Q31_t)Sguan.Motor.Duty));
            v_ch1 = Duty_v - (uint16_t)(iqmath_mul((1.0f*Min_Duty - Duty_n),(Q31_t)Sguan.Motor.Duty));
        }
        if ((Sguan.Foc.Dv > Sguan.Foc.Du) && (Sguan.Foc.Dv > Sguan.Foc.Dw) && (Duty_m <= Min_Duty)){
            v_ch0 = Duty_v - (uint16_t)(iqmath_mul((1.0f*Min_Duty - Duty_m),(Q31_t)Sguan.Motor.Duty));
            v_ch1 = Duty_v + (uint16_t)(iqmath_mul((1.0f*Min_Duty - Duty_m),(Q31_t)Sguan.Motor.Duty));
        }

        // Dw
        if ((Sguan.Foc.Dw < Sguan.Foc.Dv) && (Sguan.Foc.Dw < Sguan.Foc.Du) && (Duty_n <= Min_Duty)){
            w_ch0 = Duty_w + (uint16_t)(iqmath_mul((1.0f*Min_Duty - Duty_n),(Q31_t)Sguan.Motor.Duty));
            w_ch1 = Duty_w - (uint16_t)(iqmath_mul((1.0f*Min_Duty - Duty_n),(Q31_t)Sguan.Motor.Duty));
        }
        if ((Sguan.Foc.Dw > Sguan.Foc.Dv) && (Sguan.Foc.Dw > Sguan.Foc.Du) && (Duty_m <= Min_Duty)){
            w_ch0 = Duty_w - (uint16_t)(iqmath_mul((1.0f*Min_Duty - Duty_m),(Q31_t)Sguan.Motor.Duty));
            w_ch1 = Duty_w + (uint16_t)(iqmath_mul((1.0f*Min_Duty - Duty_m),(Q31_t)Sguan.Motor.Duty));
        }
    }
    #endif
    Sguan_TIM_SetDuty(u_ch0,u_ch1, v_ch0,v_ch1, w_ch0,w_ch1);
    // ............................................................
}

/* ================= 驱动代码(驱动层) ================= */
static inline void User_CorrespondSet(unsigned char *ch, unsigned short int size){
    /* Your code for SguanFOC here */

    // ................................................
    Sguan_UART_SendData(ch,size,1000);
    // ................................................
}


#endif // USERDATA_FUNCTION_H

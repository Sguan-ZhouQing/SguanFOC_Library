#ifndef __USERDATA_FUNCTION_H
#define __USERDATA_FUNCTION_H
#include <stdint.h>
/* 电机控制User用户设置·功能接口 */

// .................................................................

// .................................................................


static inline void User_InitialInit(void){
    /* Your code for SguanFOC here */

    // ..............................................................

    // ..............................................................
}

static inline void User_StartInit(void){
    /* Your code for SguanFOC here */

    // ..............................................................

    // ..............................................................
}

static inline void User_Delay(uint32_t ms){
    /* Your code for SguanFOC here */
    
    // .............................................................

    // .............................................................
}

static inline int16_t User_ReadADC_Raw(uint8_t Current_CH){
    /* Your code for SguanFOC here */
    int16_t ADC_num = 0;
    switch (Current_CH){
    case 0:
        // ........................................................

        // ........................................................
        break;
    case 1:
        // ........................................................

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

    // ............................................................
}

/* ================= 驱动代码(驱动层) ================= */
static inline void User_CorrespondSet(unsigned char *ch, unsigned short int size){
    /* Your code for SguanFOC here */

    // ................................................

    // ................................................
}


#endif // USERDATA_FUNCTION_H

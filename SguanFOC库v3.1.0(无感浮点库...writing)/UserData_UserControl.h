#ifndef __USERDATA_USERCONTROL_H
#define __USERDATA_USERCONTROL_H
#include "SguanFOC.h"
/* 电机控制User用户设置·实时参数控制页面 */
#include "main.h"
/* 用户自己的CODE BEGIN Includes */
extern volatile uint32_t ADC_InjectedValues[4];

/* 用户自己的CODE END Includes */

static inline void User_AO_Adjust(float AO){
    /* Your code for Parameter set */
    // 接收到串口或者CAN的数据是AO=xx?
    switch (Sguan.mode){
    case 0x02:
        Sguan.foc.Uq_in = AO;
        break;
    case 0x03:
        Sguan.foc.Target_Iq = AO;
        break;
    case 0x04:
        Sguan.foc.Target_Speed = AO;
        break;
    case 0x05:
        Sguan.foc.Target_Pos = AO;
        break;
    default:
        break;
    }
}

static inline void User_BO_Adjust(float BO){
    /* Your code for Parameter set */
    // 接收到串口或者CAN的数据是BO=xx?
    if ((0.0f < BO) && (BO < 1.0f)){
        Sguan.mode = 0x02;
    }
    else if ((1.0f <= BO) && (BO < 2.0f)){
        Sguan.mode = 0x03;
    }
    else if ((2.0f <= BO) && (BO < 3.0f)){
        Sguan.mode = 0x04;
    }
    else if ((3.0f <= BO) && (BO < 10.0f)){
        Sguan.mode = 0x05;
    }
}

static inline void User_CO_Adjust(float CO){
    /* Your code for Parameter set */
    // 接收到串口或者CAN的数据是CO=xx?
    if (CO < 0.5f){
        Sguan.status = 0x16;
    }
    else{
        Sguan.status = 0x01;
    }
}

static inline void User_UserTX(void){
    /* 仅传入主循环printf发送的数据，如TXdata.fdata[0],默认最多12个 */
    Sguan.txdata.fdata[0] = Sguan.status;
    Sguan.txdata.fdata[1] = Sguan.encoder.Real_Speed;
    Sguan.txdata.fdata[2] = Sguan.foc.Target_Speed;
    Sguan.txdata.fdata[3] = Sguan.current.Real_Id;
    Sguan.txdata.fdata[4] = Sguan.current.Real_Iq;
    Sguan.txdata.fdata[5] = Sguan.foc.Du;
    Sguan.txdata.fdata[6] = Sguan.foc.Target_Iq;
    Sguan.txdata.fdata[7] = Sguan.foc.Uq_in;
    Sguan.txdata.fdata[8] = Sguan.current.Real_Ia;
    Sguan.txdata.fdata[9] = Sguan.foc.Dv;
    Sguan.txdata.fdata[10] = Sguan.foc.Dw;
    Sguan.txdata.fdata[11] = Sguan.mode;
}


#endif // USERDATA_USERCONTROL_H

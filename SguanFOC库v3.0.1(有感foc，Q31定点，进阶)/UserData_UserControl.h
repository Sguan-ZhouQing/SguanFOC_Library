#ifndef __USERDATA_USERCONTROL_H
#define __USERDATA_USERCONTROL_H
#include "SguanFOC.h"
/* 电机控制User用户设置·实时参数控制页面 */

/* 用户自己的CODE BEGIN Includes */

/* 用户自己的CODE END Includes */

static inline void User_UserControl(void){
    /* 仅传入需要实时控制的数据，如Target_Speed */
    // like: Sguan.foc.Target_Speed = 0.0f;
}

static inline void User_AO_Adjust(float AO){
    /* Your code for Parameter set */
    /* like: 
    switch (Sguan.mode){
    case 0x00:
        Sguan.foc.Uq_in = AO;
        break;
    case 0x01:
        Sguan.foc.Target_Iq = AO;
        break;
    case 0x02:
        Sguan.foc.Target_Speed = AO;
        break;
    case 0x03:
        Sguan.foc.Target_Pos = AO;
        break;
    default:
        break;
    }
    */
}

static inline void User_BO_Adjust(float BO){
    /* Your code for Parameter set */
    /* like: 
    if ((0.0f < BO) && (BO < 1.0f)){
        Sguan.mode = 0x00;
    }
    else if ((1.0f <= BO) && (BO < 2.0f)){
        Sguan.mode = 0x01;
    }
    else if ((2.0f <= BO) && (BO < 3.0f)){
        Sguan.mode = 0x02;
    }
    else if ((3.0f <= BO) && (BO < 10.0f)){
        Sguan.mode = 0x03;
    }
    */
}

static inline void User_CO_Adjust(float CO){
    /* Your code for Parameter set */
    /* like: 
    if (CO < 0.5f){
        Sguan.status = 0x16;
    }
    else{
        Sguan.status = 0x01;
    }
    */
}

static inline void User_UserTX(void){
    /* 仅传入主循环printf发送的数据，如TXdata.fdata[0],默认最多12个 */
    /* like:
    Sguan.TXdata.fdata[0] = Sguan.status;
    Sguan.TXdata.fdata[1] = Sguan.encoder.Real_Speed;
    Sguan.TXdata.fdata[2] = Sguan.foc.Target_Speed;
    Sguan.TXdata.fdata[3] = Sguan.current.Real_Id;
    Sguan.TXdata.fdata[4] = Sguan.current.Real_Iq;
    Sguan.TXdata.fdata[5] = Sguan.foc.Target_Id;
    Sguan.TXdata.fdata[6] = Sguan.foc.Target_Iq;
    Sguan.TXdata.fdata[7] = Sguan.foc.Uq_in;
    Sguan.TXdata.fdata[8] = Sguan.current.Real_Ia;
    Sguan.TXdata.fdata[9] = Sguan.encoder.Real_Pos;
    Sguan.TXdata.fdata[10] = Sguan.encoder.Pos_offset;
    Sguan.TXdata.fdata[11] = Sguan.mode;
    */
}


#endif // USERDATA_USERCONTROL_H

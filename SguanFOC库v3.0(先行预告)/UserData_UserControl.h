#ifndef __USERDATA_USERCONTROL_H
#define __USERDATA_USERCONTROL_H
#include "SguanFOC.h"
/* 电机控制User用户设置·实时参数控制页面 */
/* 用户自己的CODE BEGIN Includes */

/* 用户自己的CODE END Includes */

static inline void User_UserControl(void){
    /* 仅传入需要实时控制的数据，如Target_Speed */
    // Sguan.foc.Target_Speed = 0.0f;
}

static inline void User_AO_Adjust(float AO){
    /* Your code for Parameter set */
    Sguan.foc.Target_Pos = AO;
}

static inline void User_BO_Adjust(float BO){
    /* Your code for Parameter set */
}

static inline void User_CO_Adjust(float CO){
    /* Your code for Parameter set */
}

static inline void User_UserTX(void){
    /* 仅传入主循环printf发送的数据，如TXdata.fdata[0],默认最多12个 */
    Sguan.TXdata.fdata[0] = Sguan.status;
    Sguan.TXdata.fdata[1] = Sguan.encoder.Real_Speed;
    Sguan.TXdata.fdata[2] = Sguan.foc.Target_Speed;
    Sguan.TXdata.fdata[3] = Sguan.current.Real_Id;
    Sguan.TXdata.fdata[4] = Sguan.current.Real_Iq;
    Sguan.TXdata.fdata[5] = Sguan.foc.Target_Iq;
    Sguan.TXdata.fdata[6] = Sguan.foc.Uq_in;
    Sguan.TXdata.fdata[7] = Sguan.current.Real_Ia;
    Sguan.TXdata.fdata[8] = Sguan.encoder.Real_Pos;
    Sguan.TXdata.fdata[9] = Sguan.foc.Target_Pos;
    Sguan.TXdata.fdata[10] = Sguan.pid.PosVelCur_p.run.Output;
    Sguan.TXdata.fdata[11] = Sguan.pid.PosVelCur_v.run.Output;
}


#endif // USERDATA_USERCONTROL_H

#ifndef __USERDATA_USERCONTROL_H
#define __USERDATA_USERCONTROL_H
#include "SguanFOC.h"
/* 电机控制User用户设置·实时参数控制页面 */
/* 用户自己的CODE BEGIN Includes */

/* 用户自己的CODE END Includes */

static inline void User_UserControl(void){
    /* 仅传入需要实时控制的数据，如Target_Speed */
    Sguan.foc.Target_Speed = 0.0f;
}

static inline void User_UserTX(void){
    /* 仅传入主循环printf发送的数据，如TXdata.fdata[0] */
    Sguan.TXdata.fdata[0] = Sguan.encoder.Real_Speed;
}


#endif // USERDATA_USERCONTROL_H

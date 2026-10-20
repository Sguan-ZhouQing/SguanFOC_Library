#ifndef __USERDATA_USERCONTROL_H
#define __USERDATA_USERCONTROL_H
#include "SguanFOC.h"
/* 电机控制User用户设置·实时参数控制页面 */
#include "Sguan_ADC.h"

static inline void User_AO_Adjust(float AO){

}

static inline void User_BO_Adjust(float BO){

}

static inline void User_CO_Adjust(float CO){

}

static inline void User_UserTX(void){
    Sguan.Printf.fdata[0] = Sguan.Float.Target_Speed;
    Sguan.Printf.fdata[1] = Sguan.Float.Real_Speed;

    Sguan.Printf.fdata[2] = Sguan.Float.Target_VF_Uq;
    Sguan.Printf.fdata[3] = Sguan.Float.Real_Uq;
}


#endif // USERDATA_USERCONTROL_H

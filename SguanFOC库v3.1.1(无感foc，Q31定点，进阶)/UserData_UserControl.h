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
    Sguan.Printf.fdata[0] = Sguan.Motor.Real_Speed*(1.907349e-6f);
    Sguan.Printf.fdata[1] = Sguan.Motor.Real_Re*(3.72529e-9f);

    Sguan.Printf.fdata[2] = Sguan.Motor.Real_Id;
    Sguan.Printf.fdata[3] = Sguan.Motor.Real_Iq;

    Sguan.Printf.fdata[4] = ADC_Value[0];
    Sguan.Printf.fdata[5] = ADC_Value[1];
    





    // Sguan.Printf.fdata[0] = Sguan.Float.Real_Speed;
    // Sguan.Printf.fdata[1] = Sguan.Float.Real_Re;
    // Sguan.Printf.fdata[2] = Sguan.Float.Real_Id;
    // Sguan.Printf.fdata[3] = Sguan.Float.Real_Iq;
    // Sguan.Printf.fdata[4] = Sguan.Motor.Real_Ialpha*(2.980232e-8);
    // Sguan.Printf.fdata[5] = Sguan.Motor.Real_Ibeta*(2.980232e-8);

    // Sguan.Printf.fdata[6] = Sguan.Motor.Real_Ia*(2.980232e-8);
    // Sguan.Printf.fdata[7] = Sguan.Motor.Real_Ib*(2.980232e-8);
    // Sguan.Printf.fdata[8] = Sguan.Transfer.LPF_Speed.go.Output*1.907349e-6;
    // Sguan.Printf.fdata[9] = Sguan.Transfer.PLL.go.OutRe*3.72529e-9;

    // Sguan.Printf.fdata[10] = Sguan.Float.Real_Uq;
    // Sguan.Printf.fdata[11] = Sguan.Transfer.NLFO.go.Input_Ibeta_f;
    // Sguan.Printf.fdata[12] = Sguan.Transfer.NLFO.go.Input_Ualpha_f;
    // Sguan.Printf.fdata[13] = Sguan.Transfer.NLFO.go.Input_Ubeta_f;
    // Sguan.Printf.fdata[14] = Sguan.Transfer.NLFO.go.Output_Sine_f;
    // Sguan.Printf.fdata[15] = Sguan.Transfer.NLFO.go.Output_Cosine_f;

    // Sguan.Printf.fdata[16] = iqmath_to_float(Sguan.Transfer.NLFO.go.Output_Sine, 1.0f);
    // Sguan.Printf.fdata[17] = Sguan.Transfer.PLL.go.Error*(4.656613e-10);
}


#endif // USERDATA_USERCONTROL_H

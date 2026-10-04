#ifndef __USERDATA_MOTOR_H
#define __USERDATA_MOTOR_H
#include "SguanFOC.h"
// 电机实体参数设置(根据实际需要填写)

static inline void User_MotorSet(void){
    // 1.最开始初始化的Target期望数值
    Sguan.Foc.Target_Speed = 0;
    Sguan.Foc.Target_Uq = 0;

    Sguan.Foc.Target_VF_Uq = 0;
    Sguan.Foc.Target_IF_Iq = 0;

    Sguan.Foc.Target_Id = 16777216;
    Sguan.Foc.Target_Iq = 0;

    // 2.电机工作电压设定（固定则自己填写）
    // (0x03000000 · 实际表示 ≈ 24 · 归一化 0.0234375)
    Sguan.Foc.VBUS = 50331648;                  // (Q31)电机当前电压

    // 3.电机本体参数设定
    Sguan.Motor.Poles = 5;                      // (uint8_t)电机极对数
    Sguan.Motor.Duty = 5999;                    // (uint16_t)PWM满计数值

    Sguan.Motor.Rs = 1.12f;                     // (float)电机相电阻
    Sguan.Motor.Ld = 0.00282163f;               // (float)电机D轴电感
    Sguan.Motor.Lq = 0.00385370f;               // (float)电机Q轴电感
    Sguan.Motor.Flux = 0.082f;                  // (float)电机磁链

    Sguan.Motor.Current_Dir = 1;                // (uint8_t)电流方向

    // 4.电机安全设置
    Sguan.Safe.Vbus_Max = 4096;                 // (Q31)Vbus阈值
    Sguan.Safe.Vbus_Min = 0;                    // (Q31)Vbus阈值

    Sguan.Safe.Temp_Max = 4096;                 // (Q31)Temp阈值
    Sguan.Safe.Temp_Min = 0;                    // (Q31)Temp阈值

    Sguan.Safe.Ibus_Max = 4096;                 // (Q31)Ibus阈值
}


#endif // USERDATA_MOTOR_H

#ifndef __USERDATA_MOTOR_H
#define __USERDATA_MOTOR_H
#include "SguanFOC.h"
// 电机实体参数设置(根据实际需要填写)

static inline void User_MotorSet(void){
    // 1.最开始初始化的Target期望数值
    // (32Rad/s)
    // Sguan.Foc.Target_Speed = 16777216;
    // (25Rad/s)
    // Sguan.Foc.Target_Speed = 13107200;
    // (15Rad/s)
    Sguan.Foc.Target_Speed = 7864320;
    Sguan.Foc.Target_Uq = 0;

    // (12V)
    // Sguan.Foc.Target_VF_Uq = 25165824;
    // (9V)
    Sguan.Foc.Target_VF_Uq = 18874368;
    // (8V)
    // Sguan.Foc.Target_VF_Uq = 16777216;
    Sguan.Foc.Target_IF_Iq = 0;

    Sguan.Foc.Target_Id = 0;
    Sguan.Foc.Target_Iq = 0;

    // ...............................................................
    Sguan.Float.Target_Speed = 15.0f;
    Sguan.Float.Target_Uq = 0.0f;

    Sguan.Float.Target_VF_Uq = 9.0f;
    Sguan.Float.Target_IF_Iq = 0.0f;

    Sguan.Float.Target_Id = 0.0f;
    Sguan.Float.Target_Iq = 0.0f;

    // 2.电机工作电压设定（固定则自己填写）
    // (0x06000000 · 实际表示 ≈ 48 · 归一化 0.046875)
    Sguan.Foc.VBUS = 100663296;                  // (Q31)电机当前电压

    // 3.电机本体参数设定
    Sguan.Motor.Poles = 5;                      // (uint8_t)电机极对数
    // (8kHz PWM: ARR=3000，2999 = 100% 占空比，跑满 Duty = 母线电压跑满。
    //  注意：u1 > 2900 时（Du > 96.7% 的顶端）峰顶直写落后于下降段匹配点，
    //  该下降段最多丢约 100 tick（≈2us）导通时间——顶端畸变，由用户在
    //  输出部分自行限幅控制。)
    Sguan.Motor.Duty = 2999;                    // (uint16_t)PWM满计数值

    Sguan.Motor.Rs = 1.12f;                     // (float)电机相电阻
    Sguan.Motor.Ld = 0.00282163f;               // (float)电机D轴电感
    Sguan.Motor.Lq = 0.00385370f;               // (float)电机Q轴电感
    Sguan.Motor.Flux = 0.082f;                  // (float)电机磁链

    Sguan.Motor.Current_Dir = 1;               // (uint8_t)电流方向

    // 4.电机安全设置
    Sguan.Safe.Vbus_Max = 4096;                 // (Q31)Vbus阈值
    Sguan.Safe.Vbus_Min = 0;                    // (Q31)Vbus阈值

    Sguan.Safe.Temp_Max = 4096;                 // (Q31)Temp阈值
    Sguan.Safe.Temp_Min = 0;                    // (Q31)Temp阈值

    Sguan.Safe.Ibus_Max = 4096;                 // (Q31)Ibus阈值
}


#endif // USERDATA_MOTOR_H

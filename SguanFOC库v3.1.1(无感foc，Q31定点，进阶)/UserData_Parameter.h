#ifndef __USERDATA_PARAMETER_H
#define __USERDATA_PARAMETER_H
#include "SguanFOC.h"
/* 电机控制User用户设置·BPF和PID和PLL运行参数 */

static inline void User_ParameterSet(void){
    // 1.低通滤波LPF参数
    Sguan.Transfer.LPF_D.Wc = 31415.96f;        // (float)截止频率
    Sguan.Transfer.LPF_Q.Wc = 31415.96f;        // (float)截止频率
    Sguan.Transfer.LPF_Speed.Wc = 2000.0f;      // (float)截止频率

    // 2.最速控制LTD参数
    Sguan.Transfer.LPF_Ltd.Wc = 5.0f;           // (float)截止频率

    // 3.闭环控制器PID参数
    Sguan.Transfer.PID_D.Kp = 8.8599182f;       // (float)Kp比例项增益
    Sguan.Transfer.PID_D.Ki = 3516.8f;          // (float)Ki积分项增益
    Sguan.Transfer.PID_D.OutMax = 24.0f;        // (float)输出上限限幅
    Sguan.Transfer.PID_D.OutMin = -24.0f;       // (float)输出下限限幅

    Sguan.Transfer.PID_Q.Kp = 12.100618f;       // (float)Kp比例项增益
    Sguan.Transfer.PID_Q.Ki = 3516.8f;          // (float)Ki积分项增益
    Sguan.Transfer.PID_Q.OutMax = 24.0f;        // (float)输出上限限幅
    Sguan.Transfer.PID_Q.OutMin = -24.0f;       // (float)输出下限限幅

    Sguan.Transfer.PID_Speed.Kp = 0.42f;        // (float)Kp比例项增益
    Sguan.Transfer.PID_Speed.Ki = 6.85f;        // (float)Ki积分项增益
    Sguan.Transfer.PID_Speed.OutMax = 10.0f;    // (float)输出上限限幅
    Sguan.Transfer.PID_Speed.OutMin = -10.0f;   // (float)输出下限限幅

    Sguan.Transfer.Response = 5;                // (uint8_t)内外环控制倍率

    // 4.锁相环PLL参数
    Sguan.Transfer.PLL.Kp = 650.0f;             // (float)Kp比例项增益
    Sguan.Transfer.PLL.Ki = 210000.0f;          // (float)Ki积分项增益

    // 5.无感磁链NLFO参数
    Sguan.Transfer.NLFO.Gain = 3520.0f;         // (float)磁链观测解调增益
}


#endif // USERDATA_PARAMETER_H

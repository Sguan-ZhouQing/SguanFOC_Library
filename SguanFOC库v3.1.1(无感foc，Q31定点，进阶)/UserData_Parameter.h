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
    Sguan.Transfer.LPF_Ltd.Wc = 0.2f;           // (float)截止频率

    // 3.闭环控制器PID参数
    Sguan.Transfer.PID_D.Kp = 8.8599182/4.25f;  // (float)Kp比例项增益
    Sguan.Transfer.PID_D.Ki = 3516.8f/4.25f;    // (float)Ki积分项增益
    Sguan.Transfer.PID_D.OutMax = 290.0f;        // (float)输出上限限幅
    Sguan.Transfer.PID_D.OutMin = -290.0f;       // (float)输出下限限幅

    Sguan.Transfer.PID_Q.Kp = 12.100618f/4.25f; // (float)Kp比例项增益
    Sguan.Transfer.PID_Q.Ki = 3516.8f/4.25f;    // (float)Ki积分项增益
    Sguan.Transfer.PID_Q.OutMax = 290.0f;        // (float)输出上限限幅
    Sguan.Transfer.PID_Q.OutMin = -290.0f;       // (float)输出下f限幅

    Sguan.Transfer.PID_Speed.Kp = 0.2f;         // (float)Kp比例项增益
    Sguan.Transfer.PID_Speed.Ki = 7.88f;        // (float)Ki积分项增益
    Sguan.Transfer.PID_Speed.OutMax = 12.5f;    // (float)输出上限限幅
    Sguan.Transfer.PID_Speed.OutMin = -12.5f;   // (float)输出下限限幅

    Sguan.Transfer.Response = 5;                // (uint8_t)内外环控制倍率

    // 4.锁相环PLL参数
    // // (机械280rad/s)
    // Sguan.Transfer.PLL.Kp = 1979.898987322f; // (float)Kp比例项增益
    // Sguan.Transfer.PLL.Ki = 1960000.0f;      // (float)Ki积分项增益
    // (机械250rad/s)
    Sguan.Transfer.PLL.Kp = 1767.766952966f;    // (float)Kp比例项增益
    Sguan.Transfer.PLL.Ki = 1562500.0f;         // (float)Ki积分项增益
    // (机械230rad/s)
    // Sguan.Transfer.PLL.Kp = 1626.34559673f;  // (float)Kp比例项增益
    // Sguan.Transfer.PLL.Ki = 1322500.0f;      // (float)Ki积分项增益
    // (更低角频率)
    // Sguan.Transfer.PLL.Kp = 650.0f;          // (float)Kp比例项增益
    // Sguan.Transfer.PLL.Ki = 210000.0f;       // (float)Ki积分项增益

    // 5.无感磁链NLFO参数
    Sguan.Transfer.NLFO.Gain = 3520.0f;         // (float)磁链观测解调增益

    // 6.无感切换相关参数设计
    Sguan.Value.Sensorless_Start = 0;           // (Q31)需要填入5rad/s的定点数值
    Sguan.Value.Sensorless_Switch = 15005760;   // (Q31)需要填入30rad/s的定点数值
}


#endif // USERDATA_PARAMETER_H

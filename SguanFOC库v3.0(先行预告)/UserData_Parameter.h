#ifndef __USERDATA_PARAMETER_H
#define __USERDATA_PARAMETER_H
#include "SguanFOC.h"
/* 电机控制User用户设置·BPF和PID和PLL运行参数 */

static inline void User_ParameterSet(void){
    // 1.bpf低通滤波器设计
    Sguan.bpf.CurrentD.Wc = 31415.96f;          // 电机D轴电流滤波->截止频率(默认参数)
    Sguan.bpf.CurrentQ.Wc = 31415.96f;          // 电机Q轴电流滤波->截止频率(默认参数)
    Sguan.bpf.Encoder.Wc = 314.1596f;           // 速度信号滤波->截止频率(默认参数)
    // 2.pid闭环控制系统设计
    #if !Open_PI_Control

    #else // Open_PI_Control
    Sguan.control.Current_D.Wc = 100.0f;            // PID电流环D轴参数->截止频率(默认参数)
    Sguan.control.Current_D.Kp = 0.261f;            // PID电流环D轴参数->Kp【PID参数自适应】
    Sguan.control.Current_D.Ki = 958.4111f;         // PID电流环D轴参数->Ki【PID参数自适应】
    Sguan.control.Current_D.Kd = 0.0f;              // PID电流环D轴参数->Kd【PID参数自适应】
    Sguan.control.Current_D.OutMax = 12.0f;         // PID电流环D轴参数->最大限幅(默认参数)
    Sguan.control.Current_D.OutMin = -12.0f;        // PID电流环D轴参数->最小限幅(默认参数)
    Sguan.control.Current_D.IntMax = 150.0f;        // PID电流环D轴参数->积分项上限(默认参数)
    Sguan.control.Current_D.IntMin = -150.0f;       // PID电流环D轴参数->积分项下限(默认参数)
    /* =========================== 分割线 ========================== */
    Sguan.control.Current_Q.Wc = 100.0f;            // PID电流环Q轴参数->截止频率(默认参数)
    Sguan.control.Current_Q.Kp = 0.261f;            // PID电流环Q轴参数->Kp【PID参数自适应】
    Sguan.control.Current_Q.Ki = 958.4111f;         // PID电流环Q轴参数->Ki【PID参数自适应】
    Sguan.control.Current_Q.Kd = 0.0f;              // PID电流环Q轴参数->Kd【PID参数自适应】
    Sguan.control.Current_Q.OutMax = 12.0f;         // PID电流环Q轴参数->最大限幅(默认参数)
    Sguan.control.Current_Q.OutMin = -12.0f;        // PID电流环Q轴参数->最小限幅(默认参数)
    Sguan.control.Current_Q.IntMax = 150.0f;        // PID电流环Q轴参数->积分项上限(默认参数)
    Sguan.control.Current_Q.IntMin = -150.0f;       // PID电流环Q轴参数->积分项下限(默认参数)

    Sguan.control.Velocity.Wc = 100.0f;             // 双PID速度外环参数(默认参数)
    Sguan.control.Velocity.Kp = 0.06f;              // 双PID速度外环参数【PID参数自适应】
    Sguan.control.Velocity.Ki = 0.4f;               // 双PID速度外环参数【PID参数自适应】
    Sguan.control.Velocity.Kd = 0.0f;               // 双PID速度外环参数【PID参数自适应】
    Sguan.control.Velocity.OutMax = 10.5f;          // 双PID速度外环参数(默认参数)
    Sguan.control.Velocity.OutMin = -10.5f;         // 双PID速度外环参数(默认参数)
    Sguan.control.Velocity.IntMax = 15000.0f;       // 双PID速度外环参数->积分项上限(默认参数)
    Sguan.control.Velocity.IntMin = -15000.0f;      // 双PID速度外环参数->积分项下限(默认参数)
    #endif // Open_PI_Control
    Sguan.control.Position.Wc = 18.0f;           // 高性能伺服三环pos(默认参数)
    Sguan.control.Position.Kp = 12.0f;           // 高性能伺服三环pos【PID参数自适应】
    Sguan.control.Position.Ki = 0.0f;            // 高性能伺服三环pos【PID参数自适应】
    Sguan.control.Position.Kd = 0.0f;            // 高性能伺服三环pos【PID参数自适应】
    Sguan.control.Position.OutMax = 150.0f;      // 高性能伺服三环pos(默认参数)
    Sguan.control.Position.OutMin = -150.0f;     // 高性能伺服三环pos(默认参数)
    Sguan.control.Position.IntMax = 150.0f;      // 高性能伺服三环pos->积分项上限(默认参数)
    Sguan.control.Position.IntMin = -150.0f;     // 高性能伺服三环pos->积分项下限(默认参数)

    Sguan.control.Response = 5;                     // (uint8_t)响应带宽倍数
    // 3.pll锁相环跟踪系统
    Sguan.encoder.pll.Kp = 650.0f;                // 锁相环比例项增益【PID参数自适应】
    Sguan.encoder.pll.Ki = 210000.0f;                // 锁相环积分项增益【PID参数自适应】
}


#endif // USERDATA_PARAMETER_H

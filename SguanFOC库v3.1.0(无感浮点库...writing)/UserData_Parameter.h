#ifndef __USERDATA_PARAMETER_H
#define __USERDATA_PARAMETER_H
#include "SguanFOC.h"
/* 电机控制User用户设置·BPF和PID和PLL运行参数 */

static inline void User_ParameterSet(void){
    // 1.bpf低通滤波器设计
    Sguan.lpf.CurrentD.Wc = 31415.96f;              // 电机D轴电流滤波->截止频率(默认参数)
    Sguan.lpf.CurrentQ.Wc = 31415.96f;              // 电机Q轴电流滤波->截止频率(默认参数)
    Sguan.lpf.Encoder.Wc = 300.0f;                  // 速度信号滤波->截止频率(默认参数)
    // 2.pid闭环控制系统设计
    Sguan.control.Current_D.Wc = 100.0f;            // PID电流环D轴参数->截止频率(默认参数)
    Sguan.control.Current_D.Kp = 0.3425f;           // PID电流环D轴参数->Kp(核心参数)
    Sguan.control.Current_D.Ki = 1257.27f;          // PID电流环D轴参数->Ki(核心参数)
    Sguan.control.Current_D.Kd = 0.0f;              // PID电流环D轴参数->Kd(参数)
    Sguan.control.Current_D.OutMax = 12.0f;         // PID电流环D轴参数->最大限幅(默认参数)
    Sguan.control.Current_D.OutMin = -12.0f;        // PID电流环D轴参数->最小限幅(默认参数)
    Sguan.control.Current_D.IntMax = 50.0f;         // PID电流环D轴参数->积分项上限(默认参数)
    Sguan.control.Current_D.IntMin = -50.0f;        // PID电流环D轴参数->积分项下限(默认参数)
    /* =========================== 分割线 ========================== */
    Sguan.control.Current_Q.Wc = 100.0f;            // PID电流环Q轴参数->截止频率(默认参数)
    Sguan.control.Current_Q.Kp = 0.3425f;           // PID电流环Q轴参数->Kp(核心参数)
    Sguan.control.Current_Q.Ki = 1257.27f;          // PID电流环Q轴参数->Ki(核心参数)
    Sguan.control.Current_Q.Kd = 0.0f;              // PID电流环Q轴参数->Kd(参数)
    Sguan.control.Current_Q.OutMax = 12.0f;         // PID电流环Q轴参数->最大限幅(默认参数)
    Sguan.control.Current_Q.OutMin = -12.0f;        // PID电流环Q轴参数->最小限幅(默认参数)
    Sguan.control.Current_Q.IntMax = 50.0f;         // PID电流环Q轴参数->积分项上限(默认参数)
    Sguan.control.Current_Q.IntMin = -50.0f;        // PID电流环Q轴参数->积分项下限(默认参数)

    #if Open_PI_Control
    Sguan.control.Velocity.Wc = 100.0f;             // 双PID速度外环参数(默认参数)
    Sguan.control.Velocity.Kp = 0.06f;              // 双PID速度外环参数(核心参数)
    Sguan.control.Velocity.Ki = 0.4f;               // 双PID速度外环参数(核心参数)
    Sguan.control.Velocity.Kd = 0.0f;               // 双PID速度外环参数(参数)
    Sguan.control.Velocity.OutMax = 10.5f;          // 双PID速度外环参数(默认参数)
    Sguan.control.Velocity.OutMin = -10.5f;         // 双PID速度外环参数(默认参数)
    Sguan.control.Velocity.IntMax = 15000.0f;       // 双PID速度外环参数->积分项上限(默认参数)
    Sguan.control.Velocity.IntMin = -15000.0f;      // 双PID速度外环参数->积分项下限(默认参数)
    #else // Open_PI_Control
    Sguan.control.Speed.boundary = 12.0f;           // STA二阶滑膜转速环->边界厚度
    Sguan.control.Speed.k1 = 0.36f;                 // STA二阶滑膜转速环->比例项增益
    Sguan.control.Speed.k2 = 52.0f;                 // STA二阶滑膜转速环->积分项增益
    Sguan.control.Speed.OutMax = 10.5f;             // STA二阶滑膜转速环->输出限幅
    Sguan.control.Speed.OutMin = -10.5f;            // STA二阶滑膜转速环->输出限幅
    Sguan.control.Speed.IntMax = 50.0f;             // STA二阶滑膜转速环->积分限幅
    Sguan.control.Speed.IntMin = -50.0f;            // STA二阶滑膜转速环->积分限幅
    #endif // Open_PI_Control
    Sguan.control.Position.Wc = 100.0f;             // 高性能伺服三环pos(默认参数)
    Sguan.control.Position.Kp = 7.0f;               // 高性能伺服三环pos(核心参数)
    Sguan.control.Position.Ki = 0.0f;               // 高性能伺服三环pos(参数)
    Sguan.control.Position.Kd = 0.0f;               // 高性能伺服三环pos(核心参数)
    Sguan.control.Position.OutMax = 230.0f;         // 高性能伺服三环pos(默认参数)
    Sguan.control.Position.OutMin = -230.0f;        // 高性能伺服三环pos(默认参数)
    Sguan.control.Position.IntMax = 150.0f;         // 高性能伺服三环pos->积分项上限(默认参数)
    Sguan.control.Position.IntMin = -150.0f;        // 高性能伺服三环pos->积分项下限(默认参数)

    Sguan.control.Response = 5;                     // (uint8_t)响应带宽倍数(核心参数)
    // 3.pll锁相环跟踪系统
    Sguan.encoder.pll.Kp = 650.0f;                  // 锁相环比例项增益(核心参数)
    Sguan.encoder.pll.Ki = 210000.0f;               // 锁相环积分项增益(核心参数)
}


#endif // USERDATA_PARAMETER_H

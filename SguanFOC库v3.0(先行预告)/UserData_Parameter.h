#ifndef __USERDATA_PARAMETER_H
#define __USERDATA_PARAMETER_H
#include "SguanFOC.h"
/* 电机控制User用户设置·BPF和PID和PLL运行参数 */

static inline void User_ParameterSet(void){
    // 1.bpf低通滤波器设计
    Sguan.bpf.VBUS.Wc = 10.0f;              // (float)母线电压滤波->截止频率
    Sguan.bpf.Current0.Wc = 10.0f;          // (float)电机电流滤波->截止频率
    Sguan.bpf.Current1.Wc = 10.0f;          // (float)电机电流滤波->截止频率
    Sguan.bpf.Encoder.Wc = 10.0f;           // (float)速度信号滤波->截止频率
    Sguan.bpf.Thermistor.Wc = 10.0f;        // (float)热敏电阻滤波->截止频率
    Sguan.bpf.HFI.Wc = 10.0f;               // (float)HFI滤波->截止频率
    Sguan.bpf.SMO.Wc = 10.0f;               // (float)SMO滤波->截止频率
    // 2.pid闭环控制系统设计
    Sguan.pid.Current_D.Wc = 100.0f;          // (float)PID电流环D轴参数->截止频率
    Sguan.pid.Current_D.Kp = 1.5f;            // (float)PID电流环D轴参数->Kp
    Sguan.pid.Current_D.Ki = 0.02f;           // (float)PID电流环D轴参数->Ki
    Sguan.pid.Current_D.Kd = 0.0f;            // (float)PID电流环D轴参数->Kd
    Sguan.pid.Current_D.OutMax = 10.0f;       // (float)PID电流环D轴参数->最大限幅
    Sguan.pid.Current_D.OutMin = -10.0f;      // (float)PID电流环D轴参数->最小限幅

    Sguan.pid.Current_Q.Wc = 100.0f;          // (float)PID电流环Q轴参数->截止频率
    Sguan.pid.Current_Q.Kp = 1.5f;            // (float)PID电流环Q轴参数->Kp
    Sguan.pid.Current_Q.Ki = 0.02f;           // (float)PID电流环Q轴参数->Ki
    Sguan.pid.Current_Q.Kd = 0.0f;            // (float)PID电流环Q轴参数->Kd
    Sguan.pid.Current_Q.OutMax = 10.0f;       // (float)PID电流环Q轴参数->最大限幅
    Sguan.pid.Current_Q.OutMin = -10.0f;      // (float)PID电流环Q轴参数->最小限幅

    Sguan.pid.Velocity.Wc = 100.0f;         // (float)PID速度环参数
    Sguan.pid.Velocity.Kp = 1.5f;           // (float)PID速度环参数
    Sguan.pid.Velocity.Ki = 0.02f;          // (float)PID速度环参数
    Sguan.pid.Velocity.Kd = 0.0f;           // (float)PID速度环参数
    Sguan.pid.Velocity.OutMax = 10.0f;      // (float)PID速度环参数
    Sguan.pid.Velocity.OutMin = -10.0f;     // (float)PID速度环参数

    Sguan.pid.Position.Wc = 100.0f;         // (float)PID位置环参数
    Sguan.pid.Position.Kp = 1.5f;           // (float)PID位置环参数
    Sguan.pid.Position.Ki = 0.02f;          // (float)PID位置环参数
    Sguan.pid.Position.Kd = 0.0f;           // (float)PID位置环参数
    Sguan.pid.Position.OutMax = 10.0f;      // (float)PID位置环参数
    Sguan.pid.Position.OutMin = -10.0f;     // (float)PID位置环参数

    Sguan.pid.VelCur_v.Wc = 100.0f;         // (float)双PID速度外环参数
    Sguan.pid.VelCur_v.Kp = 1.5f;           // (float)双PID速度外环参数
    Sguan.pid.VelCur_v.Ki = 0.02f;          // (float)双PID速度外环参数
    Sguan.pid.VelCur_v.Kd = 0.0f;           // (float)双PID速度外环参数
    Sguan.pid.VelCur_v.OutMax = 10.0f;      // (float)双PID速度外环参数
    Sguan.pid.VelCur_v.OutMin = -10.0f;     // (float)双PID速度外环参数

    Sguan.pid.VelCur_D.Wc = 100.0f;         // (float)双PID电流内环D轴参数
    Sguan.pid.VelCur_D.Kp = 1.5f;           // (float)双PID电流内环D轴参数
    Sguan.pid.VelCur_D.Ki = 0.02f;          // (float)双PID电流内环D轴参数
    Sguan.pid.VelCur_D.Kd = 0.0f;           // (float)双PID电流内环D轴参数
    Sguan.pid.VelCur_D.OutMax = 10.0f;      // (float)双PID电流内环D轴参数
    Sguan.pid.VelCur_D.OutMin = -10.0f;     // (float)双PID电流内环D轴参数

    Sguan.pid.VelCur_Q.Wc = 100.0f;         // (float)双PID电流内环Q轴参数
    Sguan.pid.VelCur_Q.Kp = 1.5f;           // (float)双PID电流内环Q轴参数
    Sguan.pid.VelCur_Q.Ki = 0.02f;          // (float)双PID电流内环Q轴参数
    Sguan.pid.VelCur_Q.Kd = 0.0f;           // (float)双PID电流内环Q轴参数
    Sguan.pid.VelCur_Q.OutMax = 10.0f;      // (float)双PID电流内环Q轴参数
    Sguan.pid.VelCur_Q.OutMin = -10.0f;     // (float)双PID电流内环Q轴参数

    Sguan.pid.PosVel_p.Wc = 100.0f;         // (float)位置-速度双环p
    Sguan.pid.PosVel_p.Kp = 1.5f;           // (float)位置-速度双环p
    Sguan.pid.PosVel_p.Ki = 0.02f;          // (float)位置-速度双环p
    Sguan.pid.PosVel_p.Kd = 0.0f;           // (float)位置-速度双环p
    Sguan.pid.PosVel_p.OutMax = 10.0f;      // (float)位置-速度双环p
    Sguan.pid.PosVel_p.OutMin = -10.0f;     // (float)位置-速度双环p

    Sguan.pid.PosVel_v.Wc = 100.0f;         // (float)位置-速度双环v
    Sguan.pid.PosVel_v.Kp = 1.5f;           // (float)位置-速度双环v
    Sguan.pid.PosVel_v.Ki = 0.02f;          // (float)位置-速度双环v
    Sguan.pid.PosVel_v.Kd = 0.0f;           // (float)位置-速度双环v
    Sguan.pid.PosVel_v.OutMax = 10.0f;      // (float)位置-速度双环v
    Sguan.pid.PosVel_v.OutMin = -10.0f;     // (float)位置-速度双环v

    Sguan.pid.PosVelCur_p.Wc = 100.0f;      // (float)高性能伺服三环pos
    Sguan.pid.PosVelCur_p.Kp = 1.5f;        // (float)高性能伺服三环pos
    Sguan.pid.PosVelCur_p.Ki = 0.02f;       // (float)高性能伺服三环pos
    Sguan.pid.PosVelCur_p.Kd = 0.0f;        // (float)高性能伺服三环pos
    Sguan.pid.PosVelCur_p.OutMax = 10.0f;   // (float)高性能伺服三环pos
    Sguan.pid.PosVelCur_p.OutMin = -10.0f;  // (float)高性能伺服三环pos

    Sguan.pid.PosVelCur_v.Wc = 100.0f;      // (float)高性能伺服三环vel
    Sguan.pid.PosVelCur_v.Kp = 1.5f;        // (float)高性能伺服三环vel
    Sguan.pid.PosVelCur_v.Ki = 0.02f;       // (float)高性能伺服三环vel
    Sguan.pid.PosVelCur_v.Kd = 0.0f;        // (float)高性能伺服三环vel
    Sguan.pid.PosVelCur_v.OutMax = 10.0f;   // (float)高性能伺服三环vel
    Sguan.pid.PosVelCur_v.OutMin = -10.0f;  // (float)高性能伺服三环vel

    Sguan.pid.PosVelCur_D.Wc = 100.0f;      // (float)高性能伺服三环D轴
    Sguan.pid.PosVelCur_D.Kp = 1.5f;        // (float)高性能伺服三环D轴
    Sguan.pid.PosVelCur_D.Ki = 0.02f;       // (float)高性能伺服三环D轴
    Sguan.pid.PosVelCur_D.Kd = 0.0f;        // (float)高性能伺服三环D轴
    Sguan.pid.PosVelCur_D.OutMax = 10.0f;   // (float)高性能伺服三环D轴
    Sguan.pid.PosVelCur_D.OutMin = -10.0f;  // (float)高性能伺服三环D轴

    Sguan.pid.PosVelCur_Q.Wc = 100.0f;      // (float)高性能伺服三环Q轴
    Sguan.pid.PosVelCur_Q.Kp = 1.5f;        // (float)高性能伺服三环Q轴
    Sguan.pid.PosVelCur_Q.Ki = 0.02f;       // (float)高性能伺服三环Q轴
    Sguan.pid.PosVelCur_Q.Kd = 0.0f;        // (float)高性能伺服三环Q轴
    Sguan.pid.PosVelCur_Q.OutMax = 10.0f;   // (float)高性能伺服三环Q轴
    Sguan.pid.PosVelCur_Q.OutMin = -10.0f;  // (float)高性能伺服三环Q轴

    Sguan.pid.Response = 10;                // (uint8_t)响应带宽倍数
    // 3.pll锁相环跟踪系统

}


#endif // USERDATA_PARAMETER_H

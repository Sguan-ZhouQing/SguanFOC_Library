#ifndef __USERDATA_PARAMETER_H
#define __USERDATA_PARAMETER_H
#include "SguanFOC.h"
/* 电机控制User用户设置·BPF和PID和PLL运行参数 */

static inline void User_ParameterSet(void){
    // 1.bpf低通滤波器设计
    Sguan.Transfer.CurrentD.Wc = 31415.96f;         // 电机D轴电流滤波->截止频率(默认参数)
    Sguan.Transfer.CurrentQ.Wc = 31415.96f;         // 电机Q轴电流滤波->截止频率(默认参数)
    Sguan.Transfer.Encoder.Wc = 300.0f;             // 速度信号滤波->截止频率(默认参数)
    // 2.pid闭环控制系统设计
    Sguan.Transfer.Current_D.Wc = 100.0f;           // PID电流环D轴参数->截止频率(默认参数)
    Sguan.Transfer.Current_D.Kp = 0.3425f;          // PID电流环D轴参数->Kp(核心参数)
    Sguan.Transfer.Current_D.Ki = 1257.27f;         // PID电流环D轴参数->Ki(核心参数)
    Sguan.Transfer.Current_D.Kd = 0.0f;             // PID电流环D轴参数->Kd(参数)
    Sguan.Transfer.Current_D.OutMax = 12.0f;        // PID电流环D轴参数->最大限幅(默认参数)
    Sguan.Transfer.Current_D.OutMin = -12.0f;       // PID电流环D轴参数->最小限幅(默认参数)
    Sguan.Transfer.Current_D.IntMax = 50.0f;        // PID电流环D轴参数->积分项上限(默认参数)
    Sguan.Transfer.Current_D.IntMin = -50.0f;       // PID电流环D轴参数->积分项下限(默认参数)
    /* ============================== 分割线 ============================ */
    Sguan.Transfer.Current_Q.Wc = 100.0f;           // PID电流环Q轴参数->截止频率(默认参数)
    Sguan.Transfer.Current_Q.Kp = 0.3425f;          // PID电流环Q轴参数->Kp(核心参数)
    Sguan.Transfer.Current_Q.Ki = 1257.27f;         // PID电流环Q轴参数->Ki(核心参数)
    Sguan.Transfer.Current_Q.Kd = 0.0f;             // PID电流环Q轴参数->Kd(参数)
    Sguan.Transfer.Current_Q.OutMax = 12.0f;        // PID电流环Q轴参数->最大限幅(默认参数)
    Sguan.Transfer.Current_Q.OutMin = -12.0f;       // PID电流环Q轴参数->最小限幅(默认参数)
    Sguan.Transfer.Current_Q.IntMax = 50.0f;        // PID电流环Q轴参数->积分项上限(默认参数)
    Sguan.Transfer.Current_Q.IntMin = -50.0f;       // PID电流环Q轴参数->积分项下限(默认参数)

    #if CONFIG_CtrlVel==1
    Sguan.Transfer.Velocity.r = 50.0f;              // 双环速度外环speed的LADRC->速度因子
    Sguan.Transfer.Velocity.b0 = 1.2e6;             // 双环速度外环speed的LADRC->控制量增益
    Sguan.Transfer.Velocity.wc = 200.0f;            // 双环速度外环speed的LADRC->控制器带宽
    Sguan.Transfer.Velocity.OutMax = 10.5f;         // 双环速度外环speed的LADRC->输出上限
    Sguan.Transfer.Velocity.OutMin = -10.5f;        // 双环速度外环speed的LADRC->输出下限
    #elif CONFIG_CtrlVel==2
    Sguan.Transfer.Velocity.Wc = 100.0f;            // 双环速度外环speed的SMC->微分环节滤波
    Sguan.Transfer.Velocity.miu = 200.0f;           // 双环速度外环speed的SMC->不连续控制增益
    Sguan.Transfer.Velocity.q = 30.0f;              // 双环速度外环speed的SMC->切换项增益
    Sguan.Transfer.Velocity.C = 2000.0f;            // 双环速度外环speed的SMC->动态响应增益
    Sguan.Transfer.Velocity.Gain = 30.0f;           // 双环速度外环speed的SMC->电机输入增益
    Sguan.Transfer.Velocity.IntMax = 10.5;          // 双环速度外环speed的SMC->积分上限
    Sguan.Transfer.Velocity.IntMin = -10.5f;        // 双环速度外环speed的SMC->积分下限
    #elif CONFIG_CtrlVel==3
    Sguan.Transfer.Velocity.boundary = 12.0f;       // 双环速度外环speed的STA->边界厚度
    Sguan.Transfer.Velocity.k1 = 0.36f;             // 双环速度外环speed的STA->比例项增益
    Sguan.Transfer.Velocity.k2 = 52.0f;             // 双环速度外环speed的STA->积分项增益
    Sguan.Transfer.Velocity.OutMax = 10.5f;         // 双环速度外环speed的STA->输出限幅
    Sguan.Transfer.Velocity.OutMin = -10.5f;        // 双环速度外环speed的STA->输出限幅
    Sguan.Transfer.Velocity.IntMax = 50.0f;         // 双环速度外环speed的STA->积分限幅
    Sguan.Transfer.Velocity.IntMin = -50.0f;        // 双环速度外环speed的STA->积分限幅
    #else // CONFIG_CtrlVel
    Sguan.Transfer.Velocity.Wc = 100.0f;            // 双环速度外环speed的PID
    Sguan.Transfer.Velocity.Kp = 0.06f;             // 双环速度外环speed的PID
    Sguan.Transfer.Velocity.Ki = 0.4f;              // 双环速度外环speed的PID
    Sguan.Transfer.Velocity.Kd = 0.0f;              // 双环速度外环speed的PID
    Sguan.Transfer.Velocity.OutMax = 10.5f;         // 双环速度外环speed的PID
    Sguan.Transfer.Velocity.OutMin = -10.5f;        // 双环速度外环speed的PID
    Sguan.Transfer.Velocity.IntMax = 15000.0f;      // 双环速度外环speed的PID
    Sguan.Transfer.Velocity.IntMin = -15000.0f;     // 双环速度外环speed的PID
    #endif // CONFIG_CtrlVel

    #if CONFIG_CtrlPos==1
    Sguan.Transfer.Velocity.r = 50.0f;              // 高性能伺服三环pos的LADRC->速度因子
    Sguan.Transfer.Velocity.b0 = 1.2e6;             // 高性能伺服三环pos的LADRC->控制量增益
    Sguan.Transfer.Velocity.wc = 200.0f;            // 高性能伺服三环pos的LADRC->控制器带宽
    Sguan.Transfer.Velocity.OutMax = 10.5f;         // 高性能伺服三环pos的LADRC->输出上限
    Sguan.Transfer.Velocity.OutMin = -10.5f;        // 高性能伺服三环pos的LADRC->输出下限
    #elif CONFIG_CtrlPos==2
    Sguan.Transfer.Velocity.Wc = 100.0f;            // 高性能伺服三环pos的SMC->微分环节滤波
    Sguan.Transfer.Velocity.miu = 200.0f;           // 高性能伺服三环pos的SMC->不连续控制增益
    Sguan.Transfer.Velocity.q = 30.0f;              // 高性能伺服三环pos的SMC->切换项增益
    Sguan.Transfer.Velocity.C = 2000.0f;            // 高性能伺服三环pos的SMC->动态响应增益
    Sguan.Transfer.Velocity.Gain = 30.0f;           // 高性能伺服三环pos的SMC->电机输入增益
    Sguan.Transfer.Velocity.IntMax = 10.5;          // 高性能伺服三环pos的SMC->积分上限
    Sguan.Transfer.Velocity.IntMin = -10.5f;        // 高性能伺服三环pos的SMC->积分下限
    #elif CONFIG_CtrlPos==3
    Sguan.Transfer.Velocity.boundary = 12.0f;       // 高性能伺服三环pos的STA->边界厚度
    Sguan.Transfer.Velocity.k1 = 0.36f;             // 高性能伺服三环pos的STA->比例项增益
    Sguan.Transfer.Velocity.k2 = 52.0f;             // 高性能伺服三环pos的STA->积分项增益
    Sguan.Transfer.Velocity.OutMax = 10.5f;         // 高性能伺服三环pos的STA->输出限幅
    Sguan.Transfer.Velocity.OutMin = -10.5f;        // 高性能伺服三环pos的STA->输出限幅
    Sguan.Transfer.Velocity.IntMax = 50.0f;         // 高性能伺服三环pos的STA->积分限幅
    Sguan.Transfer.Velocity.IntMin = -50.0f;        // 高性能伺服三环pos的STA->积分限幅
    #else // CONFIG_CtrlPos
    Sguan.Transfer.Position.Wc = 100.0f;            // 高性能伺服三环pos的PID
    Sguan.Transfer.Position.Kp = 7.0f;              // 高性能伺服三环pos的PID
    Sguan.Transfer.Position.Ki = 0.0f;              // 高性能伺服三环pos的PID
    Sguan.Transfer.Position.Kd = 0.0f;              // 高性能伺服三环pos的PID
    Sguan.Transfer.Position.OutMax = 230.0f;        // 高性能伺服三环pos的PID
    Sguan.Transfer.Position.OutMin = -230.0f;       // 高性能伺服三环pos的PID
    Sguan.Transfer.Position.IntMax = 150.0f;        // 高性能伺服三环pos的PID
    Sguan.Transfer.Position.IntMin = -150.0f;       // 高性能伺服三环pos的PID
    #endif // CONFIG_CtrlPos

    Sguan.Transfer.Response = 5;                    // (uint8_t)响应带宽倍数(核心参数)
    // 3.pll锁相环跟踪系统
    Sguan.Transfer.PLL.Kp = 650.0f;                 // 锁相环比例项增益(核心参数)
    Sguan.Transfer.PLL.Ki = 210000.0f;              // 锁相环积分项增益(核心参数)
    // 4.DOB
    Sguan.Transfer.DOB.K1 = 2.0f;                   // 扰动观测器
    Sguan.Transfer.DOB.K2 = 3.0f;                   // 扰动观测器
    Sguan.Transfer.DOB.OutMax = 10.5f;              // 扰动观测器
    Sguan.Transfer.DOB.OutMin = -10.5f;             // 扰动观测器
}


#endif // USERDATA_PARAMETER_H

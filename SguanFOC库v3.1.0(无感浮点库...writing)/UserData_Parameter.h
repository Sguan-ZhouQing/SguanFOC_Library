#ifndef __USERDATA_PARAMETER_H
#define __USERDATA_PARAMETER_H
#include "SguanFOC.h"
/* 电机控制User用户设置·BPF和PID和PLL运行参数 */

static inline void User_ParameterSet(void){
    // 1.bpf低通滤波器设计
    Sguan.transfer.LPF_D.Wc = 31415.96f;         // 电机D轴电流滤波->截止频率(默认参数)
    Sguan.transfer.LPF_Q.Wc = 31415.96f;         // 电机Q轴电流滤波->截止频率(默认参数)
    Sguan.transfer.LPF_encoder.Wc = 300.0f;             // 速度信号滤波->截止频率(默认参数)
    // 2.pid闭环控制系统设计
    Sguan.transfer.Current_D.Wc = 100.0f;           // PID电流环D轴参数->截止频率(默认参数)
    Sguan.transfer.Current_D.Kp = 0.3425f;          // PID电流环D轴参数->Kp(核心参数)
    Sguan.transfer.Current_D.Ki = 1257.27f;         // PID电流环D轴参数->Ki(核心参数)
    Sguan.transfer.Current_D.Kd = 0.0f;             // PID电流环D轴参数->Kd(参数)
    Sguan.transfer.Current_D.OutMax = 12.0f;        // PID电流环D轴参数->最大限幅(默认参数)
    Sguan.transfer.Current_D.OutMin = -12.0f;       // PID电流环D轴参数->最小限幅(默认参数)
    Sguan.transfer.Current_D.IntMax = 50.0f;        // PID电流环D轴参数->积分项上限(默认参数)
    Sguan.transfer.Current_D.IntMin = -50.0f;       // PID电流环D轴参数->积分项下限(默认参数)
    /* ============================== 分割线 ============================ */
    Sguan.transfer.Current_Q.Wc = 100.0f;           // PID电流环Q轴参数->截止频率(默认参数)
    Sguan.transfer.Current_Q.Kp = 0.3425f;          // PID电流环Q轴参数->Kp(核心参数)
    Sguan.transfer.Current_Q.Ki = 1257.27f;         // PID电流环Q轴参数->Ki(核心参数)
    Sguan.transfer.Current_Q.Kd = 0.0f;             // PID电流环Q轴参数->Kd(参数)
    Sguan.transfer.Current_Q.OutMax = 12.0f;        // PID电流环Q轴参数->最大限幅(默认参数)
    Sguan.transfer.Current_Q.OutMin = -12.0f;       // PID电流环Q轴参数->最小限幅(默认参数)
    Sguan.transfer.Current_Q.IntMax = 50.0f;        // PID电流环Q轴参数->积分项上限(默认参数)
    Sguan.transfer.Current_Q.IntMin = -50.0f;       // PID电流环Q轴参数->积分项下限(默认参数)

    #if Switch_Control_Velocity==1
    Sguan.transfer.Velocity.r = 50.0f;              // 双环速度外环speed的LADRC->速度因子
    Sguan.transfer.Velocity.b0 = 1.2e6;             // 双环速度外环speed的LADRC->控制量增益
    Sguan.transfer.Velocity.wc = 200.0f;            // 双环速度外环speed的LADRC->控制器带宽
    Sguan.transfer.Velocity.OutMax = 10.5f;         // 双环速度外环speed的LADRC->输出上限
    Sguan.transfer.Velocity.OutMin = -10.5f;        // 双环速度外环speed的LADRC->输出下限
    #elif Switch_Control_Velocity==2
    Sguan.transfer.Velocity.Wc = 100.0f;            // 双环速度外环speed的SMC->微分环节滤波
    Sguan.transfer.Velocity.miu = 200.0f;           // 双环速度外环speed的SMC->不连续控制增益
    Sguan.transfer.Velocity.q = 30.0f;              // 双环速度外环speed的SMC->切换项增益
    Sguan.transfer.Velocity.C = 2000.0f;            // 双环速度外环speed的SMC->动态响应增益
    Sguan.transfer.Velocity.Gain = 30.0f;           // 双环速度外环speed的SMC->电机输入增益
    Sguan.transfer.Velocity.IntMax = 10.5;          // 双环速度外环speed的SMC->积分上限
    Sguan.transfer.Velocity.IntMin = -10.5f;        // 双环速度外环speed的SMC->积分下限
    #elif Switch_Control_Velocity==3
    Sguan.transfer.Velocity.boundary = 12.0f;       // 双环速度外环speed的STA->边界厚度
    Sguan.transfer.Velocity.k1 = 0.36f;             // 双环速度外环speed的STA->比例项增益
    Sguan.transfer.Velocity.k2 = 52.0f;             // 双环速度外环speed的STA->积分项增益
    Sguan.transfer.Velocity.OutMax = 10.5f;         // 双环速度外环speed的STA->输出限幅
    Sguan.transfer.Velocity.OutMin = -10.5f;        // 双环速度外环speed的STA->输出限幅
    Sguan.transfer.Velocity.IntMax = 50.0f;         // 双环速度外环speed的STA->积分限幅
    Sguan.transfer.Velocity.IntMin = -50.0f;        // 双环速度外环speed的STA->积分限幅
    #else // Switch_Control_Velocity
    Sguan.transfer.Velocity.Wc = 100.0f;            // 双环速度外环speed的PID
    Sguan.transfer.Velocity.Kp = 0.06f;             // 双环速度外环speed的PID
    Sguan.transfer.Velocity.Ki = 0.4f;              // 双环速度外环speed的PID
    Sguan.transfer.Velocity.Kd = 0.0f;              // 双环速度外环speed的PID
    Sguan.transfer.Velocity.OutMax = 10.5f;         // 双环速度外环speed的PID
    Sguan.transfer.Velocity.OutMin = -10.5f;        // 双环速度外环speed的PID
    Sguan.transfer.Velocity.IntMax = 15000.0f;      // 双环速度外环speed的PID
    Sguan.transfer.Velocity.IntMin = -15000.0f;     // 双环速度外环speed的PID
    #endif // Switch_Control_Velocity

    #if Switch_Control_Position==1
    Sguan.transfer.Velocity.r = 50.0f;              // 高性能伺服三环pos的LADRC->速度因子
    Sguan.transfer.Velocity.b0 = 1.2e6;             // 高性能伺服三环pos的LADRC->控制量增益
    Sguan.transfer.Velocity.wc = 200.0f;            // 高性能伺服三环pos的LADRC->控制器带宽
    Sguan.transfer.Velocity.OutMax = 10.5f;         // 高性能伺服三环pos的LADRC->输出上限
    Sguan.transfer.Velocity.OutMin = -10.5f;        // 高性能伺服三环pos的LADRC->输出下限
    #elif Switch_Control_Position==2
    Sguan.transfer.Velocity.Wc = 100.0f;            // 高性能伺服三环pos的SMC->微分环节滤波
    Sguan.transfer.Velocity.miu = 200.0f;           // 高性能伺服三环pos的SMC->不连续控制增益
    Sguan.transfer.Velocity.q = 30.0f;              // 高性能伺服三环pos的SMC->切换项增益
    Sguan.transfer.Velocity.C = 2000.0f;            // 高性能伺服三环pos的SMC->动态响应增益
    Sguan.transfer.Velocity.Gain = 30.0f;           // 高性能伺服三环pos的SMC->电机输入增益
    Sguan.transfer.Velocity.IntMax = 10.5;          // 高性能伺服三环pos的SMC->积分上限
    Sguan.transfer.Velocity.IntMin = -10.5f;        // 高性能伺服三环pos的SMC->积分下限
    #elif Switch_Control_Position==3
    Sguan.transfer.Velocity.boundary = 12.0f;       // 高性能伺服三环pos的STA->边界厚度
    Sguan.transfer.Velocity.k1 = 0.36f;             // 高性能伺服三环pos的STA->比例项增益
    Sguan.transfer.Velocity.k2 = 52.0f;             // 高性能伺服三环pos的STA->积分项增益
    Sguan.transfer.Velocity.OutMax = 10.5f;         // 高性能伺服三环pos的STA->输出限幅
    Sguan.transfer.Velocity.OutMin = -10.5f;        // 高性能伺服三环pos的STA->输出限幅
    Sguan.transfer.Velocity.IntMax = 50.0f;         // 高性能伺服三环pos的STA->积分限幅
    Sguan.transfer.Velocity.IntMin = -50.0f;        // 高性能伺服三环pos的STA->积分限幅
    #else // Switch_Control_Position
    Sguan.transfer.Position.Wc = 100.0f;            // 高性能伺服三环pos的PID
    Sguan.transfer.Position.Kp = 7.0f;              // 高性能伺服三环pos的PID
    Sguan.transfer.Position.Ki = 0.0f;              // 高性能伺服三环pos的PID
    Sguan.transfer.Position.Kd = 0.0f;              // 高性能伺服三环pos的PID
    Sguan.transfer.Position.OutMax = 230.0f;        // 高性能伺服三环pos的PID
    Sguan.transfer.Position.OutMin = -230.0f;       // 高性能伺服三环pos的PID
    Sguan.transfer.Position.IntMax = 150.0f;        // 高性能伺服三环pos的PID
    Sguan.transfer.Position.IntMin = -150.0f;       // 高性能伺服三环pos的PID
    #endif // Switch_Control_Position

    Sguan.transfer.Response = 5;                    // (uint8_t)响应带宽倍数(核心参数)
    // 3.pll锁相环跟踪系统
    Sguan.transfer.PLL.Kp = 650.0f;                 // 锁相环比例项增益(核心参数)
    Sguan.transfer.PLL.Ki = 210000.0f;              // 锁相环积分项增益(核心参数)
    // 4.DOB
    Sguan.transfer.DOB.K1 = 2.0f;                   // 扰动观测器
    Sguan.transfer.DOB.K2 = 3.0f;                   // 扰动观测器
    Sguan.transfer.DOB.OutMax = 10.5f;              // 扰动观测器
    Sguan.transfer.DOB.OutMin = -10.5f;             // 扰动观测器
}


#endif // USERDATA_PARAMETER_H

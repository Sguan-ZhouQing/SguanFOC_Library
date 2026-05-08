#ifndef __USERDATA_PARAMETER_H
#define __USERDATA_PARAMETER_H
#include "SguanFOC.h"
/* 电机控制User用户设置，Sguan.transfer数据 */

static inline void User_Parameter_Init(SguanFOC_System_STRUCT *user){    
    // 1.电流环参数
    user->transfer.Current_D.Wc = 100.0f;           // PID电流环D轴参数->微分滤波
    user->transfer.Current_D.Kp = 0.3425f;          // PID电流环D轴参数->Kp
    user->transfer.Current_D.Ki = 1257.27f;         // PID电流环D轴参数->Ki
    user->transfer.Current_D.Kd = 0.0f;             // PID电流环D轴参数->Kd
    user->transfer.Current_D.OutMax = 12.0f;        // PID电流环D轴参数->最大限幅
    user->transfer.Current_D.OutMin = -12.0f;       // PID电流环D轴参数->最小限幅
    user->transfer.Current_D.IntMax = 50.0f;        // PID电流环D轴参数->积分项上限
    user->transfer.Current_D.IntMin = -50.0f;       // PID电流环D轴参数->积分项下限
    /* =================================== 分割线 ================================= */
    user->transfer.Current_Q.Wc = 100.0f;           // PID电流环Q轴参数->微分滤波
    user->transfer.Current_Q.Kp = 0.3425f;          // PID电流环Q轴参数->Kp
    user->transfer.Current_Q.Ki = 1257.27f;         // PID电流环Q轴参数->Ki
    user->transfer.Current_Q.Kd = 0.0f;             // PID电流环Q轴参数->Kd
    user->transfer.Current_Q.OutMax = 12.0f;        // PID电流环Q轴参数->最大限幅
    user->transfer.Current_Q.OutMin = -12.0f;       // PID电流环Q轴参数->最小限幅
    user->transfer.Current_Q.IntMax = 50.0f;        // PID电流环Q轴参数->积分项上限
    user->transfer.Current_Q.IntMin = -50.0f;       // PID电流环Q轴参数->积分项下限

    // 2.转速环参数
    #if CONFIG_CtrlVel==Control_LADRC
    user->transfer.Velocity.r = 50.0f;              // 双环速度外环speed的LADRC->速度因子
    user->transfer.Velocity.b0 = 12e5;              // 双环速度外环speed的LADRC->控制量增益
    user->transfer.Velocity.wc = 200.0f;            // 双环速度外环speed的LADRC->控制器带宽
    user->transfer.Velocity.OutMax = 10.5f;         // 双环速度外环speed的LADRC->输出上限
    user->transfer.Velocity.OutMin = -10.5f;        // 双环速度外环speed的LADRC->输出下限
    #elif CONFIG_CtrlVel==Control_SMC
    user->transfer.Velocity.miu = 200.0f;           // 双环速度外环speed的SMC->不连续控制增益
    user->transfer.Velocity.q = 30.0f;              // 双环速度外环speed的SMC->切换项增益
    user->transfer.Velocity.C = 2000.0f;            // 双环速度外环speed的SMC->动态响应增益
    user->transfer.Velocity.Gain = 30.0f;           // 双环速度外环speed的SMC->电机输入增益
    user->transfer.Velocity.IntMax = 10.5;          // 双环速度外环speed的SMC->积分上限
    user->transfer.Velocity.IntMin = -10.5f;        // 双环速度外环speed的SMC->积分下限
    #elif CONFIG_CtrlVel==Control_STA
    user->transfer.Velocity.boundary = 12.0f;       // 双环速度外环speed的STA->边界厚度
    user->transfer.Velocity.k1 = 0.36f;             // 双环速度外环speed的STA->比例项增益
    user->transfer.Velocity.k2 = 52.0f;             // 双环速度外环speed的STA->积分项增益
    user->transfer.Velocity.OutMax = 10.5f;         // 双环速度外环speed的STA->输出限幅
    user->transfer.Velocity.OutMin = -10.5f;        // 双环速度外环speed的STA->输出限幅
    user->transfer.Velocity.IntMax = 50.0f;         // 双环速度外环speed的STA->积分限幅
    user->transfer.Velocity.IntMin = -50.0f;        // 双环速度外环speed的STA->积分限幅
    #else // CONFIG_CtrlVel
    user->transfer.Velocity.Wc = 100.0f;            // 双环速度外环speed的PID->微分滤波
    user->transfer.Velocity.Kp = 0.06f;             // 双环速度外环speed的PID->Kp
    user->transfer.Velocity.Ki = 0.4f;              // 双环速度外环speed的PID->Ki
    user->transfer.Velocity.Kd = 0.0f;              // 双环速度外环speed的PID->Kd
    user->transfer.Velocity.OutMax = 10.5f;         // 双环速度外环speed的PID->最大限幅
    user->transfer.Velocity.OutMin = -10.5f;        // 双环速度外环speed的PID->最小限幅
    user->transfer.Velocity.IntMax = 15000.0f;      // 双环速度外环speed的PID->积分项上限
    user->transfer.Velocity.IntMin = -15000.0f;     // 双环速度外环speed的PID->积分项下限
    #endif // CONFIG_Control

    // 3.位置环参数
    #if CONFIG_CtrlPos==Control_LADRC
    user->transfer.Position.r = 50.0f;              // 高性能伺服三环pos的LADRC->速度因子
    user->transfer.Position.b0 = 1.2e6;             // 高性能伺服三环pos的LADRC->控制量增益
    user->transfer.Position.wc = 200.0f;            // 高性能伺服三环pos的LADRC->控制器带宽
    user->transfer.Position.OutMax = 10.5f;         // 高性能伺服三环pos的LADRC->输出上限
    user->transfer.Position.OutMin = -10.5f;        // 高性能伺服三环pos的LADRC->输出下限
    #elif CONFIG_CtrlPos==Control_SMC
    user->transfer.Position.miu = 200.0f;           // 高性能伺服三环pos的SMC->不连续控制增益
    user->transfer.Position.q = 30.0f;              // 高性能伺服三环pos的SMC->切换项增益
    user->transfer.Position.C = 2000.0f;            // 高性能伺服三环pos的SMC->动态响应增益
    user->transfer.Position.Gain = 30.0f;           // 高性能伺服三环pos的SMC->电机输入增益
    user->transfer.Position.IntMax = 10.5;          // 高性能伺服三环pos的SMC->积分上限
    user->transfer.Position.IntMin = -10.5f;        // 高性能伺服三环pos的SMC->积分下限
    #elif CONFIG_CtrlPos==Control_STA
    user->transfer.Position.boundary = 12.0f;       // 高性能伺服三环pos的STA->边界厚度
    user->transfer.Position.k1 = 0.36f;             // 高性能伺服三环pos的STA->比例项增益
    user->transfer.Position.k2 = 52.0f;             // 高性能伺服三环pos的STA->积分项增益
    user->transfer.Position.OutMax = 10.5f;         // 高性能伺服三环pos的STA->输出限幅
    user->transfer.Position.OutMin = -10.5f;        // 高性能伺服三环pos的STA->输出限幅
    user->transfer.Position.IntMax = 50.0f;         // 高性能伺服三环pos的STA->积分限幅
    user->transfer.Position.IntMin = -50.0f;        // 高性能伺服三环pos的STA->积分限幅
    #else // CONFIG_CtrlPos
    user->transfer.Position.Wc = 100.0f;            // 高性能伺服三环pos的PID->截止频率
    user->transfer.Position.Kp = 7.0f;              // 高性能伺服三环pos的PID->Kp
    user->transfer.Position.Ki = 0.0f;              // 高性能伺服三环pos的PID->Ki
    user->transfer.Position.Kd = 0.0f;              // 高性能伺服三环pos的PID->Kd
    user->transfer.Position.OutMax = 230.0f;        // 高性能伺服三环pos的PID->最大限幅
    user->transfer.Position.OutMin = -230.0f;       // 高性能伺服三环pos的PID->最小限幅
    user->transfer.Position.IntMax = 150.0f;        // 高性能伺服三环pos的PID->积分项上限
    user->transfer.Position.IntMin = -150.0f;       // 高性能伺服三环pos的PID->积分项下限
    #endif // CONFIG_CtrlPos

    // 4.响应倍数参数
    user->transfer.Response = 5;                    // (uint8_t)响应带宽倍数(核心参数)

    // 5.滤波器参数
    user->transfer.LPF_D.Wc = 31415.96f;            // 电机D轴电流滤波->截止频率
    user->transfer.LPF_Q.Wc = 31415.96f;            // 电机Q轴电流滤波->截止频率
    user->transfer.LPF_encoder.Wc = 300.0f;         // 速度信号滤波->截止频率

    // 6.霍尔模式参数
    #if CONFIG_MODE==MODE_Sensor_Hall
    user->transfer.Hall.Wc = 100.0f;                // 霍尔信号处理->滤波截止频率
    user->transfer.Hall.Hall_High = 0.6f;           // 霍尔信号处理->信号上边界
    user->transfer.Hall.Hall_Low = 0.4f;            // 霍尔信号处理->信号下边界
    #endif // CONFIG_MODE

    // 7.锁相环参数
    user->transfer.PLL_encoder.Kp = 650.0f;         // 锁相环->比例项增益
    user->transfer.PLL_encoder.Ki = 210000.0f;      // 锁相环->积分项增益

    // 8.扰动观测器参数
    #if CONFIG_DOB
    user->transfer.DOB.K1 = 2.0f;                   // 扰动观测器->比例项增益
    user->transfer.DOB.K2 = 3.0f;                   // 扰动观测器->积分项增益
    user->transfer.DOB.OutMax_Fd = 10.5f;           // 扰动观测器->输出限幅
    user->transfer.DOB.OutMin_Fd = -10.5f;          // 扰动观测器->输出限幅
    user->transfer.DOB.OutMax_Wm = 10.5f;           // 扰动观测器->输出限幅
    user->transfer.DOB.OutMin_Wm = -10.5f;          // 扰动观测器->输出限幅
    #endif // CONFIG_DOB    

    // 9.弱磁控制参数
    #if CONFIG_FW
    user->transfer.FW.Wc = 100.0f;                  // 弱磁控制的PI控制器->截止频率
    user->transfer.FW.Kp = 7.0f;                    // 弱磁控制的PI控制器->Kp
    user->transfer.FW.Ki = 0.0f;                    // 弱磁控制的PI控制器->Ki
    user->transfer.FW.Kd = 0.0f;                    // 弱磁控制的PI控制器->Kd
    user->transfer.FW.OutMax = 230.0f;              // 弱磁控制的PI控制器->最大限幅
    user->transfer.FW.OutMin = -230.0f;             // 弱磁控制的PI控制器->最小限幅
    user->transfer.FW.IntMax = 150.0f;              // 弱磁控制的PI控制器->积分项上限
    user->transfer.FW.IntMin = -150.0f;             // 弱磁控制的PI控制器->积分项下限

    user->transfer.BaseSpeed_fw = 300.0f;           // 弱磁基速设计->MTPA转变区
    user->transfer.Percentage_fw = 0.92f;           // 弱磁调制占比->0.92工程经验
    #endif // CONFIG_FW

    // 10.速度前馈的参数
    #if CONFIG_VelFF
    user->transfer.Beta_ff = 62.8f;                 // (float)转速环角频率(前馈补偿参数)
    #endif // CONFIG_VelFF

    // 11.死区补偿参数
    #if CONFIG_DeadZone
    user->transfer.DeadTime = 1e-10f;               // (float)死区时间填写(死区补偿参数)
    user->transfer.Dead_CurMin = 0.1f;              // (float)补偿最小相电流(死区补偿参数)
    #endif // CONFIG_DeadZone

    // 12.无感控制算法参数
    #if CONFIG_MODE==MODE_Sensorless_HFI
    user->transfer.HFI.Percentage_hfi = 0.1f;       // 高频注入->电压占比
    #elif CONFIG_MODE==MODE_Sensorless_SMO
    user->transfer.SMO.Wc = 100.0f;                 // 滑模->滤波截止频率
    user->transfer.SMO.h = 50.0f;                   // 滑模->观测器增益
    user->transfer.SMO.IntMax = 5000.0f;            // 滑模->积分项上限
    user->transfer.SMO.IntMin = 5000.0f;            // 滑模->积分项下限
    #elif CONFIG_MODE==MODE_Sensorless_HS
    user->transfer.HFI.Percentage_hfi = 0.1f;       // 高频注入->电压占比

    user->transfer.SMO.Wc = 100.0f;                 // 滑模->滤波截止频率
    user->transfer.SMO.h = 50.0f;                   // 滑模->观测器增益
    user->transfer.SMO.IntMax = 5000.0f;            // 滑模->积分项上限
    user->transfer.SMO.IntMin = 5000.0f;            // 滑模->积分项下限

    user->transfer.PLL_another.Kp = 650.0f;         // 锁相环->比例项增益
    user->transfer.PLL_another.Ki = 210000.0f;      // 锁相环->积分项增益
    #endif // CONFIG_MODE
}


#endif // USERDATA_PARAMETER_H

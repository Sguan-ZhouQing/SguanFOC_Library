#ifndef __USERDATA_PARAMETER_H
#define __USERDATA_PARAMETER_H
#include "SguanFOC.h"
/* 电机控制User用户设置，Sguan.transfer数据 */

/**
 * @description: 控制器参数设计的初始化代码
 * @reminder: (此方函数->填写你设定好的控制器参数)
 * @param {SguanFOC_System_STRUCT} *user
 * @return {*}
 */
static inline void User_Parameter_Init(SguanFOC_System_STRUCT *user){
    // +---------------------------------------------------------+
    // |                控制器参数Parameter设计                   |
    // +---------------------------------------------------------+
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
    user->transfer.Velocity.Wc = 200.0f;            // 双环速度外环speed的LADRC->控制器带宽
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
    user->transfer.Position.Wc = 200.0f;            // 高性能伺服三环pos的LADRC->控制器带宽
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
    user->value.Response = 5;                       // (uint8_t)响应带宽倍数(核心参数)

    // 5.滤波器参数
    user->transfer.LPF_D.Wc = 31415.96f;            // 电机D轴电流滤波->截止频率
    user->transfer.LPF_Q.Wc = 31415.96f;            // 电机Q轴电流滤波->截止频率
    user->transfer.LPF_encoder.Wc = 300.0f;         // 速度信号滤波->截止频率

    // 6.锁相环参数
    user->transfer.PLL_encoder.Kp = 650.0f;         // 锁相环->比例项增益
    user->transfer.PLL_encoder.Ki = 210000.0f;      // 锁相环->积分项增益
    #if IS_COM_MODE
    user->transfer.PLL_another.Kp = 650.0f;         // 锁相环->比例项增益
    user->transfer.PLL_another.Ki = 210000.0f;      // 锁相环->积分项增益
    #endif // IS_COM_MODE

    // 7.Debug模式参数
    #if IS_DEBUG_MODE
    user->transfer.PLL_Debug.Kp = 650.0f;           // 锁相环->比例项增益
    user->transfer.PLL_Debug.Ki = 210000.0f;        // 锁相环->积分项增益

    user->transfer.LPF_Debug.Wc = 300.0f;           // 速度信号滤波->截止频率
    #endif // IS_DEBUG_MODE

    // 8.扰动观测器参数
    #if CONFIG_DOB
    user->transfer.DOB.K1 = 2.0f;                   // 扰动观测器->比例项增益
    user->transfer.DOB.K2 = 3.0f;                   // 扰动观测器->积分项增益
    user->transfer.DOB.OutMax_Fd = 10.5f;           // 扰动观测器->输出限幅
    user->transfer.DOB.OutMin_Fd = -10.5f;          // 扰动观测器->输出限幅
    user->transfer.DOB.OutMax_Wm = 10.5f;           // 扰动观测器->输出限幅
    user->transfer.DOB.OutMin_Wm = -10.5f;          // 扰动观测器->输出限幅
    #endif // CONFIG_DOB    

    // 9.谐波抑制
    #if CONFIG_Inhibit
    user->transfer.TPNF_D.K1 = 0.3f;                // 谐波抑制陷波滤波器->阻尼比
    user->transfer.TPNF_D.K2 = 0.1f;                // 谐波抑制陷波滤波器->阻尼比

    user->transfer.TPNF_Q.K1 = 0.3f;                // 谐波抑制陷波滤波器->阻尼比
    user->transfer.TPNF_Q.K2 = 0.1f;                // 谐波抑制陷波滤波器->阻尼比
    #endif // CONFIG_Inhibit

    // 10.弱磁控制参数
    #if CONFIG_FW
    user->transfer.FW.Wc = 100.0f;                  // 弱磁控制的PI控制器->截止频率
    user->transfer.FW.Kp = 7.0f;                    // 弱磁控制的PI控制器->Kp
    user->transfer.FW.Ki = 0.0f;                    // 弱磁控制的PI控制器->Ki
    user->transfer.FW.Kd = 0.0f;                    // 弱磁控制的PI控制器->Kd
    user->transfer.FW.OutMax = 230.0f;              // 弱磁控制的PI控制器->最大限幅
    user->transfer.FW.OutMin = -230.0f;             // 弱磁控制的PI控制器->最小限幅
    user->transfer.FW.IntMax = 150.0f;              // 弱磁控制的PI控制器->积分项上限
    user->transfer.FW.IntMin = -150.0f;             // 弱磁控制的PI控制器->积分项下限

    user->value.FW_BaseSpeed = 300.0f;              // 弱磁基速设计->MTPA转变区
    user->value.FW_Percentage = 0.92f;              // 弱磁调制占比->0.92工程经验
    #endif // CONFIG_FW

    // 11.速度前馈的参数
    #if CONFIG_VelFF
    user->value.VelFF_Beta = 62.8f;                 // (float)转速环角频率(前馈补偿参数)
    #endif // CONFIG_VelFF

    // 12.死区补偿参数
    #if CONFIG_DeadZone
    user->value.DeadZone_Time = 1e-10f;             // (float)死区时间填写(死区补偿参数)
    user->value.DeadZone_CurMin = 0.1f;             // (float)补偿最小相电流(死区补偿参数)
    #endif // CONFIG_DeadZone

    // 13.角度补偿参数
    #if CONFIG_AngleComp
    user->value.AngleComp_Td = 1e-7f;               // (float)随电角速度的相位延迟时间
    user->value.AngleComp_Offset = 0.03f;           // (float)随速度方向变化的固定相位偏置
    #endif // CONFIG_AngleComp

    // 14.最速控制结构体
    #if IS_LTD_MODE
    user->transfer.LTD.r = 2.0f;                    // LTD最速控制->跟踪因子2rad/s合适
    #endif // IS_LTD_MODE
    
    // 15.霍尔有感结构体
    #if IS_HALL_MODE
    user->transfer.Hall.Wc = 100.0f;                // 霍尔信号处理->滤波截止频率
    user->transfer.Hall.Hall_High = 0.6f;           // 霍尔信号处理->信号上边界
    user->transfer.Hall.Hall_Low = 0.4f;            // 霍尔信号处理->信号下边界
    #endif // IS_HALL_MODE

    // 16.高频正弦波注入
    #if IS_HFI_MODE
    user->transfer.HFI.Wo = 3455.8f;                // 高频正弦波->注入电压角频率
    user->transfer.HFI.h = 2.0f;                    // 高频正弦波->高频解调增益
    user->transfer.HFI.Uh = 2.4f;                   // 高频正弦波->注入电压幅值
    user->transfer.HFI.K1 = 0.3f;                   // 高频正弦波->陷波分母系数
    user->transfer.HFI.K2 = 0.1f;                   // 高频正弦波->陷波分子系数
    user->transfer.HFI.zeta = 0.2f;                 // 高频正弦波->带通滤波阻尼比
    #endif // IS_HFI_MODE

    // 17.滑模观测器
    #if IS_SMO_MODE
    user->transfer.SMO.Wc = 10000.0f;               // 滑模观测器->反电动势低通滤波
    user->transfer.SMO.h = 3.5f;                    // 滑模观测器->滑模解调增益
    user->transfer.SMO.IntMax = 2000.0f;            // 滑模观测器->观测器积分限幅
    user->transfer.SMO.IntMin = -2000.0f;           // 滑模观测器->观测器积分限幅
    #endif // IS_SMO_MODE

    // 18.非线性磁链观测器
    #if IS_NLFO_MODE
    user->transfer.NLFO.Gain = 15000000;            // 非线性磁链->磁链观测解调增益
    #endif // IS_NLFO_MODE

    // 19.无感参数数据
    #if CONFIG_MODE>=MODE_Sensorless_HFI
    user->value.Sensorless_Stop = 135.0f;           // 分界限->低速域观测器关闭
    user->value.Sensorless_Open = 125.0f;           // 分界限->低速域观测器开启
    user->value.Sensorless_AbsMax = 100.0f;         // 分界限->“过渡区”到“高速域”
    user->value.Sensorless_AbsMin = 80.0f;          // 分界限->“低速域”到“过渡区”
    #endif // CONFIG_MODE
}


#endif // USERDATA_PARAMETER_H

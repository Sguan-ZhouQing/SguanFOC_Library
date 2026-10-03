/*
 * @Author: 星必尘Sguan
 * @GitHub: https://github.com/Sguan-ZhouQing
 * @Date: 2026-01-26 22:38:09
 * @LastEditors: 星必尘Sguan|3464647102@qq.com
 * @LastEditTime: 2026-04-27 03:45:16
 * @FilePath: \SguanFOC_Debug\SguanFOC\Sguan_PID.c
 * @Description: SguanFOC库的“开环PID算法”实现
 * 
 * Copyright (c) 2026 by $星必尘Sguan, All Rights Reserved. 
 */
#include "Sguan_PID.h"

// 闭环系统PID核心参数初始化
void PID_Init(PID_STRUCT *pid){
    // 1.数学Q31数值转换
    if (pid->id == 0){
        // (电流to电压，等于电阻的基值)
        // (电流环......pid->Kp单位为Ω，pid->Ki单位为Ω/s)
        pid->go.Kp = iqmath_from_float(pid->Kp, BASE_Resistor);
        pid->go.Ki = iqmath_from_float(pid->Ki, (BASE_Resistor/BASE_Time));

        // (如果id=0，电流环，pid->go.Input单位A，pid->go.Io单位A*s，pid->go.Output单位V)
        // (那么pid->Kp单位为Ω即"V/A"，pid->Ki单位为Ω/s即"V/(A*s)")
        pid->go.OutMax = iqmath_from_float(pid->OutMax,BASE_Voltage);
        pid->go.OutMin = iqmath_from_float(pid->OutMin,BASE_Voltage);

        pid->go.IntMax = iqmath_from_float(pid->IntMax,(BASE_Current*BASE_Time));
        pid->go.IntMin = iqmath_from_float(pid->IntMin,(BASE_Current*BASE_Time));

        // .....................................................................
        // 范围计算Kp(D轴)  实际值：8.8599182   单位范围：BASE_Resistor
        // 范围计算Kp(Q轴)  实际值：12.100618   单位范围：BASE_Resistor
        // 范围计算Ki(DQ)   实际值：3516.8      单位范围：(BASE_Resistor/BASE_Time)
        // 范围计算OutMax   实际值：48.0        单位范围：BASE_Voltage
        // 范围计算OutMin   实际值：48.0        单位范围：BASE_Voltage
        // 范围计算IntMax   实际值：48.0/3516.8 单位范围：(BASE_Current*BASE_Time)
        // 范围计算IntMin   实际值：48.0/3516.8 单位范围：(BASE_Current*BASE_Time)
    }
    else{
        // (机械角速度to电流)
        // (速度环......pid->Kp单位为(A/Rad)*s，pid->Ki单位为A/Rad)
        pid->go.Kp = iqmath_from_float(pid->Kp, ((BASE_Current/BASE_Speed)*BASE_SUB));
        pid->go.Ki = iqmath_from_float(pid->Ki, (BASE_Current/BASE_Rad)); 

        // (如果id=1，速度环，pid->go.Input单位Rad/s，pid->go.Io单位Rad，pid->go.Output单位A)
        // (那么pid->Kp单位为(A/Rad)*s，pid->Ki单位为A/Rad)
        pid->go.OutMax = iqmath_from_float(pid->OutMax,BASE_Current);
        pid->go.OutMin = iqmath_from_float(pid->OutMin,BASE_Current);

        pid->go.IntMax = iqmath_from_float(pid->IntMax,BASE_Rad);
        pid->go.IntMin = iqmath_from_float(pid->IntMin,BASE_Rad);

        // .....................................................................
        // 范围计算Kp       实际值：0.42        单位范围：(BASE_Current/BASE_Speed)
        // 范围计算Ki       实际值：6.85        单位范围：(BASE_Current/BASE_Rad)
        // 范围计算OutMax   实际值：10.0        单位范围：BASE_Current
        // 范围计算OutMin   实际值：10.0        单位范围：BASE_Current
        // 范围计算IntMax   实际值：10.0/6.85   单位范围：BASE_Rad
        // 范围计算IntMin   实际值：10.0/6.85   单位范围：BASE_Rad
    }
    pid->go.T = PMSM_RUN_T_q31;

    // 2.初始化为零
    pid->go.Input = 0;

    pid->go.Ref = 0;
    pid->go.Fbk = 0;
    pid->go.Output = 0;
    pid->go.IntegralFrozen_flag = 0;
}

// 闭环控制运算的离散服务函数
void PID_Loop(PID_STRUCT *pid){
    // 1.计算比例、积分、微分项
    pid->go.Input = iqmath_sub(pid->go.Ref, pid->go.Fbk);
    if (pid->Ki){
        // 判断是否需要冻结积分
        if (pid->go.IntegralFrozen_flag){
            // 如果积分已冻结，保持上次的积分值
            
            // 检查是否可以解除冻结
            // 情况1：误差反向（误差符号与积分输出符号相反）
            // 情况2：积分值回到限幅范围内
            if (((pid->go.Input >= 0) && (pid->go.Io < 0)) || 
                ((pid->go.Input < 0) && (pid->go.Io >= 0)) || 
                ((pid->go.Io < pid->go.IntMax) && 
                (pid->go.Io > pid->go.IntMin))){
                pid->go.IntegralFrozen_flag = 0;
            }
        } else{
            // 正常计算积分
            pid->go.Io = iqmath_add(
                iqmath_mul(pid->go.Input, pid->go.T), 
                pid->go.Io);
            
            // 检查是否达到限幅，达到则冻结积分
            if (pid->go.Io > pid->go.IntMax){
                pid->go.Io = pid->go.IntMax;
                pid->go.IntegralFrozen_flag = 1;
            }
            else if (pid->go.Io < pid->go.IntMin){
                pid->go.Io = pid->go.IntMin;
                pid->go.IntegralFrozen_flag = 1;
            }
        }
    }

    // 2.运算控制器输出量并输出限幅
    // (如果id=0，电流环，pid->go.Input单位A，pid->go.Io单位A*s，pid->go.Output单位V)
    // (那么pid->Kp单位为Ω即"V/A"，pid->Ki单位为Ω/s即"V/(A*s)")
    // ......................................
    // (如果id=1，速度环，pid->go.Input单位Rad/s，pid->go.Io单位Rad，pid->go.Output单位A)
    // (那么pid->Kp单位为(A/Rad)*s，pid->Ki单位为A/Rad)
    if (pid->id == 0){
        pid->go.Output = iqmath_add(
            iqmath_mul(pid->go.Input, pid->Kp), 
            iqmath_mul(pid->go.Io, pid->Ki));
    }
    else{
        Q31_t add_temp = iqmath_mul(pid->go.Input, pid->Kp);

        // 缩放32倍处理，还原BASE_SUB
        Q31_t add_0 = iqmath_shift5_left(add_temp);
        Q31_t add_1 = iqmath_mul(pid->go.Io, pid->Ki);
        pid->go.Output = iqmath_add(add_0,add_1);
    }

    iqmath_limit(&pid->go.Output, pid->go.OutMax, pid->go.OutMin);
}



/*
 * @Author: 星必尘Sguan
 * @GitHub: https://github.com/Sguan-ZhouQing
 * @Date: 2026-01-26 22:38:09
 * @LastEditors: 星必尘Sguan|3464647102@qq.com
 * @LastEditTime: 2026-03-20 22:57:23
 * @FilePath: \SguanFOC_Debug\SguanFOC\Sguan_PID.c
 * @Description: SguanFOC库的“开环PID算法”实现
 * 
 * Copyright (c) 2026 by $星必尘Sguan, All Rights Reserved. 
 */
#include "Sguan_PID.h"

// 闭环系统PID核心参数初始化，主函数调用
void PID_Init(PID_STRUCT *pid){
    double temp0 = pid->T*pid->Ki;
    double temp1 = pid->T*pid->Wc;
    double temp2 = pid->Kd*pid->Wc;
    pid->run.I_num = (float)temp0;
    pid->run.D_num = (float)(2*temp2);
    pid->run.D_den[0] = (float)(2+temp1);
    pid->run.D_den[1] = (float)(-2+temp1);
    // 初始化为零
    for (int n = 0; n < 2; n++){
        pid->run.i[n] = 0;
        pid->run.Io[n] = 0;
        pid->run.Do[n] = 0;
    }
    pid->run.Ref = 0;
    pid->run.Fbk = 0;
    pid->run.Output = 0;
    pid->run.IntegralFrozen_flag = 0;
}

// 闭环控制运算的定时器中断服务函数
void PID_Loop(PID_STRUCT *pid){
    // 刷新历史输入和输出数值
    pid->run.i[1] = pid->run.i[0];
    pid->run.Io[1] = pid->run.Io[0];
    pid->run.Do[1] = pid->run.Do[0];
    // 更新当前输入
    pid->run.i[0] = pid->run.Ref - pid->run.Fbk;
    if (pid->Ki){
        // 判断是否需要冻结积分
        if (pid->run.IntegralFrozen_flag) {
            // 如果积分已冻结，保持上次的积分值
            pid->run.Io[0] = pid->run.Io[1];
            
            // 检查是否可以解除冻结
            // 情况1：误差反向（误差符号与积分输出符号相反）
            // 情况2：积分值回到限幅范围内
            if ((pid->run.i[0] * pid->run.Io[0] < 0) ||  // 误差反向
                (pid->run.Io[0] < pid->IntMax && pid->run.Io[0] > pid->IntMin)) {  // 回到范围内
                pid->run.IntegralFrozen_flag = 0;
            }
        } else {
            // 正常计算积分
            pid->run.Io[0] = (pid->run.I_num*pid->run.i[0] + pid->run.I_num*pid->run.i[1] 
                        + 2.0f*pid->run.Io[1]) / 2.0f;
            
            // 检查是否达到限幅，达到则冻结积分
            if (pid->run.Io[0] > pid->IntMax) {
                pid->run.Io[0] = pid->IntMax;
                pid->run.IntegralFrozen_flag = 1;
            }
            else if (pid->run.Io[0] < pid->IntMin) {
                pid->run.Io[0] = pid->IntMin;
                pid->run.IntegralFrozen_flag = 1;
            }
        }
    }
    if (pid->Kd){
        pid->run.Do[0] = (pid->run.D_num*pid->run.i[0] - pid->run.D_num*pid->run.i[1] 
                    - pid->run.D_den[1]*pid->run.Do[1]) / pid->run.D_den[0];
    }
    pid->run.Output = pid->run.i[0]*pid->Kp + pid->run.Io[0] + pid->run.Do[0];
    pid->run.Output = Value_Limit(pid->run.Output,pid->OutMax,pid->OutMin);
}


// ============================ Q31 版本代码 ============================

void PID_Init_q31(PID_STRUCT_q31 *pid){
    double temp0 = pid->T*pid->Ki;
    double temp1 = pid->T*pid->Wc;
    double temp2 = pid->Kd*pid->Wc;     // 电流->电压  电流为1  电压为64  Gs=64=电压/电流
    pid->run.P_num = IQmath_Q31_from_float(pid->Kp,pid->BASE_Out/pid->BASE_In);
    pid->run.I_num = IQmath_Q31_from_float((float)(temp0/2.0),pid->BASE_Out/pid->BASE_In);
    pid->run.D_num = IQmath_Q31_from_float((float)(2*temp2/(2+temp1)),pid->BASE_Out/pid->BASE_In);
    pid->run.D_den = IQmath_Q31_from_float((float)((-2+temp1)/(2+temp1)),1.0f);

    pid->run.OutMax = IQmath_Q31_from_float(pid->OutMax,pid->BASE_Out);
    pid->run.OutMin = IQmath_Q31_from_float(pid->OutMin,pid->BASE_Out);

    pid->run.IntMax = IQmath_Q31_from_float(pid->IntMax,pid->BASE_Out);
    pid->run.IntMin = IQmath_Q31_from_float(pid->IntMin,pid->BASE_Out);

    for (int n = 0; n < 2; n++){
        pid->run.i[n] = 0;
        pid->run.Io[n] = 0;
        pid->run.Do[n] = 0;
    }
    pid->run.Ref = 0;
    pid->run.Fbk = 0;
    pid->run.Output = 0;
    pid->run.IntegralFrozen_flag = 0;
}

void PID_Loop_q31(PID_STRUCT_q31 *pid){
    pid->run.i[1] = pid->run.i[0];
    pid->run.Io[1] = pid->run.Io[0];
    pid->run.Do[1] = pid->run.Do[0];

    pid->run.i[0] = pid->run.Ref - pid->run.Fbk;
    if (pid->Ki){
        if (pid->run.IntegralFrozen_flag){
            pid->run.Io[0] = pid->run.Io[1];

            if ((IQmath_Q31_mul(pid->run.i[0],pid->run.Io[0]) < 0) ||
                (pid->run.Io[0] < pid->run.IntMax && pid->run.Io[0] > pid->run.IntMin)){
                pid->run.IntegralFrozen_flag = 0;
            }
        } else{
            Q31_t Io = IQmath_Q31_mul(pid->run.I_num,pid->run.i[0]) + 
                    IQmath_Q31_mul(pid->run.I_num,pid->run.i[1]) + 
                    pid->run.Io[1];
            pid->run.Io[0] = IQmath_Q31_Limit(Io);
            
            if (pid->run.Io[0] > pid->run.IntMax){
                pid->run.Io[0] = pid->run.IntMax;
                pid->run.IntegralFrozen_flag = 1;
            }
            else if (pid->run.Io[0] < pid->run.IntMin){
                pid->run.Io[0] = pid->run.IntMin;
                pid->run.IntegralFrozen_flag = 1;
            }
        }
    }
    if (pid->Kd){
        Q31_t Do = IQmath_Q31_mul(pid->run.D_num,pid->run.i[0]) - 
                        IQmath_Q31_mul(pid->run.D_den,pid->run.Do[1]) + 
                        IQmath_Q31_mul(pid->run.D_num,pid->run.i[1]);
        pid->run.Do[0] = IQmath_Q31_Limit(Do);
    }
    pid->run.Do[0] = IQmath_Q31_mul(pid->run.i[0],pid->run.P_num);
    pid->run.Output = pid->run.Io[0] + pid->run.Do[0];
    pid->run.Output = Value_Limit_q31(pid->run.Output,pid->run.OutMax,pid->run.OutMin);
}


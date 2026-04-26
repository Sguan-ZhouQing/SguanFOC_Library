/*
 * @Author: 星必尘Sguan
 * @GitHub: https://github.com/Sguan-ZhouQing
 * @Date: 2026-01-26 22:38:09
 * @LastEditors: 星必尘Sguan|3464647102@qq.com
 * @LastEditTime: 2026-04-19 01:00:54
 * @FilePath: \SguanFOC_Debug\SguanFOC\Sguan_PID.c
 * @Description: SguanFOC库的“开环PID算法”实现
 * 
 * Copyright (c) 2026 by $星必尘Sguan, All Rights Reserved. 
 */
#include "Sguan_PID.h"

/**
 * @description: 闭环系统PID核心参数初始化
 * @param {PID_STRUCT} *pid
 * @return {*}
 */
void PID_Init(PID_STRUCT *pid){
    double temp0 = pid->T*pid->Ki/2.0;
    double temp1 = pid->T*pid->Wc;
    double temp2 = pid->Kd*pid->Wc;
    pid->run.I_num = (float)temp0;
    pid->run.D_num = (float)((2*temp2)/(-2+temp1));
    pid->run.D_den = (float)((2+temp1)/(-2+temp1));
    // 初始化为零
    pid->run.i[0] = 0.0f;
    pid->run.i[1] = 0.0f;

    pid->run.Ref = 0.0f;
    pid->run.Fbk = 0.0f;
    pid->run.Output = 0.0f;
    pid->run.IntegralFrozen_flag = 0.0f;
}

/**
 * @description: 闭环控制运算的离散服务函数
 * @param {PID_STRUCT} *pid
 * @return {*}
 */
void PID_Loop(PID_STRUCT *pid){
    // 1.计算比例、积分、微分项
    pid->run.i[0] = pid->run.Ref - pid->run.Fbk;
    if (pid->Ki){
        // 判断是否需要冻结积分
        if (pid->run.IntegralFrozen_flag){
            // 如果积分已冻结，保持上次的积分值
            
            // 检查是否可以解除冻结
            // 情况1：误差反向（误差符号与积分输出符号相反）
            // 情况2：积分值回到限幅范围内
            if ((pid->run.i[0]*pid->run.Io < 0) ||  // 误差反向
                ((pid->run.Io < pid->IntMax) && 
                (pid->run.Io > pid->IntMin))){
                pid->run.IntegralFrozen_flag = 0;
            }
        } else {
            // 正常计算积分
            pid->run.Io = pid->run.I_num*(pid->run.i[0] + pid->run.i[1]) 
                        + pid->run.Io;
            
            // 检查是否达到限幅，达到则冻结积分
            if (pid->run.Io > pid->IntMax){
                pid->run.Io = pid->IntMax;
                pid->run.IntegralFrozen_flag = 1;
            }
            else if (pid->run.Io < pid->IntMin){
                pid->run.Io = pid->IntMin;
                pid->run.IntegralFrozen_flag = 1;
            }
        }
    }
    if (pid->Kd){
        pid->run.Do = pid->run.D_num*(pid->run.i[0] + pid->run.i[1]) -  
                    pid->run.D_den*pid->run.Do;
    }

    // 2.运算控制器输出量
    pid->run.Output = pid->run.i[0]*pid->Kp + pid->run.Io + pid->run.Do;

    // 3.输出限幅
    pid->run.Output = Value_Limit(pid->run.Output, 
                                pid->OutMax, 
                                pid->OutMin);
    
    // 4.刷新历史输入和输出数值
    pid->run.i[1] = pid->run.i[0];
}


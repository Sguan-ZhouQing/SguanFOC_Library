/*
 * @Author: 星必尘Sguan
 * @GitHub: https://github.com/Sguan-ZhouQing
 * @Date: 2026-01-26 22:38:09
 * @LastEditors: 星必尘Sguan|3464647102@qq.com
 * @LastEditTime: 2026-02-06 05:29:40
 * @FilePath: \stm_SguanFOCtest\SguanFOC\Sguan_PID.c
 * @Description: SguanFOC库的“闭环PID算法”实现
 * 
 * Copyright (c) 2026 by $星必尘Sguan, All Rights Reserved. 
 */
#include "Sguan_PID.h"

// 闭环系统PID核心参数初始化，主函数调用
void PID_Init(PID_STRUCT *pid){
    double temp0 = pid->T*pid->Ki;
    double temp1 = pid->T*pid->Wc;
    double temp2 = pid->Kd*pid->Wc;
    pid->run.I_num[0] = (float)temp0;
    pid->run.I_num[1] = (float)temp0;
    pid->run.I_den[0] = (float)2;
    pid->run.I_den[1] = (float)(-2);
    pid->run.D_num[0] = (float)(2*temp2);
    pid->run.D_num[1] = (float)(-2*temp2);
    pid->run.D_den[0] = (float)(2+temp1);
    pid->run.D_den[1] = (float)(-2+temp1);
    // 初始化为零
    for (int n = 0; n < 2; n++){
        pid->run.i[n] = 0;
        pid->run.Io[n] = 0;
        pid->run.Do[n] = 0;
    }
}

// 闭环控制运算的定时器中断服务函数
void PID_Loop(PID_STRUCT *pid){
    // 更新历史输入和输出数值
    pid->run.i[1] = pid->run.i[0];
    pid->run.Io[1] = pid->run.Io[0];
    pid->run.Do[1] = pid->run.Do[0];
    // 更新当前输入
    pid->run.i[0] = pid->run.Ref - pid->run.Fbk;
    if (pid->Ki){
        pid->run.Io[0] = (pid->run.I_num[0]*pid->run.i[0] + pid->run.I_num[1]*pid->run.i[1] 
                    - pid->run.I_den[1]*pid->run.Io[1]) / pid->run.I_den[0];
    }
    if (pid->Kd){
        pid->run.Do[0] = (pid->run.D_num[0]*pid->run.i[0] + pid->run.D_num[1]*pid->run.i[1] 
                    - pid->run.D_den[1]*pid->run.Do[1]) / pid->run.D_den[0];
    }
    pid->run.Output = pid->run.i[0]*pid->Kp + pid->run.Io[0] + pid->run.Do[0];
    pid->run.Output = Value_Limit(pid->run.Output,pid->OutMax,pid->OutMin);
}


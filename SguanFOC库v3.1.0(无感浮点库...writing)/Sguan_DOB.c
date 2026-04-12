/*
 * @Author: 星必尘Sguan
 * @GitHub: https://github.com/Sguan-ZhouQing
 * @Date: 2026-04-09 16:16:24
 * @LastEditors: 星必尘Sguan|3464647102@qq.com
 * @LastEditTime: 2026-04-09 17:22:46
 * @FilePath: \SguanFOC_Debug\SguanFOC\Sguan_DOB.c
 * @Description: SguanFOC库的“超螺旋滑模扰动观测器”实现
 * 
 * Copyright (c) 2026 by $星必尘Sguan, All Rights Reserved. 
 */
#include "Sguan_DOB.h"

// 超螺旋滑模DOB的初始化函数
void DOB_Init(DOB_STRUCT *dob){
    dob->smdo.num = (float)(dob->T/2.0);
    dob->smdo.Gain0 = (float)(1.5*dob->Pn*dob->Flux/dob->J);
    dob->smdo.Gain1 = (float)(-dob->B/dob->J);
    // 初始化为零
    dob->smdo.Fd_i = 0.0f;
    dob->smdo.Wm_i = 0.0f;
    dob->smdo.Fd_o = 0.0f;
    dob->smdo.Wm_o = 0.0f;

    dob->smdo.Input_Iq = 0.0f;
    dob->smdo.Input_Wm = 0.0f;
    dob->smdo.Output_Fd = 0.0f;
    dob->smdo.Output_Wm = 0.0f;
}

// 超螺旋滑模DOB的离散运行函数
void DOB_Loop(DOB_STRUCT *dob){
    // 计算扰动转矩大小
    float dob_0,dob_1,dob_2;
    dob_0 = dob->smdo.Input_Iq*dob->smdo.Gain0;
    dob_1 = dob->smdo.Wm_o*dob->smdo.Gain1;
    float temp = dob->smdo.Wm_o - dob->smdo.Input_Wm;
    if (temp > 0){
        dob_2 = Value_sqrtf(
            Value_fabsf(temp));
    }
    else{
        dob_2 = -Value_sqrtf(
            Value_fabsf(temp));
    }

    // 预估扰动转矩
    float Fd_in = (-dob->K2)*dob_2;
    dob->smdo.Output_Fd = dob->smdo.num*(Fd_in + dob->smdo.Fd_i) + 
                        dob->smdo.Fd_o;
    dob->smdo.Output_Fd = Value_Limit(dob->smdo.Output_Fd,dob->OutMax,dob->OutMin);

    // 预估机械角速度
    float Wm_in = dob_0 + dob_1 - dob->K1*dob_2 + dob->smdo.Output_Fd;
    dob->smdo.Output_Wm = dob->smdo.num*(Wm_in + dob->smdo.Wm_i) + 
                        dob->smdo.Wm_o;
    dob->smdo.Output_Wm = Value_Limit(dob->smdo.Output_Wm,dob->OutMax,dob->OutMin);

    // 更新历史输入输出值
    dob->smdo.Fd_i = Fd_in;
    dob->smdo.Wm_i = Wm_in;
    dob->smdo.Fd_o = dob->smdo.Output_Fd;
    dob->smdo.Wm_o = dob->smdo.Output_Wm;
}



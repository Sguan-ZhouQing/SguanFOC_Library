/*
 * @Author: 星必尘Sguan
 * @GitHub: https://github.com/Sguan-ZhouQing
 * @Date: 2026-04-09 16:16:24
 * @LastEditors: 星必尘Sguan|3464647102@qq.com
 * @LastEditTime: 2026-04-19 00:59:45
 * @FilePath: \SguanFOC_Debug\SguanFOC\Sguan_DOB.c
 * @Description: SguanFOC库的“超螺旋滑模扰动观测器”实现
 * 
 * Copyright (c) 2026 by $星必尘Sguan, All Rights Reserved. 
 */
#include "Sguan_DOB.h"

/**
 * @description: 超螺旋滑模DOB的初始化函数
 * @param {DOB_STRUCT} *dob
 * @return {*}
 */
void DOB_Init(DOB_STRUCT *dob){
    dob->smdo.I_num = (float)(dob->T/2.0);
    dob->smdo.O_num = (float)((dob->J*dob->T)/(2.0*dob->J + dob->B*dob->T));
    dob->smdo.O_den = (float)((2.0*dob->J - dob->B*dob->T)/
                                (2.0*dob->J + dob->B*dob->T));
    dob->smdo.Gain0 = (float)((1.5*dob->Pn*dob->Flux)/dob->J);
    dob->smdo.Gain1 = (float)(1.0/dob->J);
    // 初始化为零
    dob->smdo.Output_Wm = 0.0f;

    dob->smdo.Input_Iq = 0.0f;
    dob->smdo.Input_Wm = 0.0f;
    dob->smdo.Output_Fd = 0.0f;
    dob->smdo.Output_Wm = 0.0f;
}

/**
 * @description: 超螺旋滑模DOB的离散运行函数
 * @reminder: https://github.com/Sguan-ZhouQing/SguanFOC_Library/blob/main/%E6%9C%80%E6%96%B0example%E5%8F%8A%E8%B5%84%E6%96%99%5BSTM32G4%2C%E4%B8%8B%E6%A1%A5%E8%87%82%E5%8F%8C%E7%94%B5%E9%98%BB%5D/%E3%80%90Simulink%E3%80%91Sguan%E5%AD%90%E6%A8%A1%E5%9D%97%E5%8E%9F%E7%90%86%E5%9B%BE/Sguan_DOB.png
 * @reminder: (上方链接是此Sguan_DOB模块Simulink原理仿真图)
 * @param {DOB_STRUCT} *dob
 * @return {*}
 */
void DOB_Loop(DOB_STRUCT *dob){
    // 1.创建局部变量，并运算(part0)
    float part_main,part0,part1,part2,error_wm,sign,temp_fd;
    part0 = dob->smdo.Input_Iq*dob->smdo.Gain0;

    // 2.计算扰动力矩(part1)
    error_wm = dob->smdo.Output_Wm - dob->smdo.Input_Wm;
    sign = Value_Sign(error_wm);
    temp_fd = sign*dob->K2;
    dob->smdo.Output_Fd += dob->smdo.I_num*(temp_fd + dob->smdo.Fd_i);
    dob->smdo.Output_Fd = Value_Limit(dob->smdo.Output_Fd,
                                    dob->OutMax_Fd,
                                    dob->OutMin_Fd);
    part1 = dob->smdo.Output_Fd*dob->smdo.Gain1;
    
    // 3.计算不连续量(part3)
    part2 = Value_sqrtf(Value_fabsf(error_wm))*sign*dob->K1;

    // 4.总积分项
    part_main = part0 - part1 - part2;
    dob->smdo.Output_Wm = dob->smdo.O_num*(part_main + dob->smdo.Wm_i) + 
                        dob->smdo.O_den*dob->smdo.Output_Wm;
    dob->smdo.Output_Wm = Value_Limit(dob->smdo.Output_Wm,
                                    dob->OutMax_Wm,
                                    dob->OutMin_Wm);

    // 5.更新历史输入输出值
    dob->smdo.Fd_i = temp_fd;
    dob->smdo.Wm_i = part_main;
}


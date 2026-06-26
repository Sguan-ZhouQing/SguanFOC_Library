/*
 * @Author: 星必尘Sguan
 * @GitHub: https://github.com/Sguan-ZhouQing
 * @Date: 2026-04-29 19:43:41
 * @LastEditors: 星必尘Sguan|3464647102@qq.com
 * @LastEditTime: 2026-06-05 03:46:38
 * @FilePath: \SguanFOC_Debug\SguanFOC\Sguan_HFI.c
 * @Description: SguanFOC库的“HFI无感高频注入算法(高频脉振正弦波)”实现
 * 
 * Copyright (c) 2026 by $星必尘Sguan, All Rights Reserved. 
 */
#include "Sguan_HFI.h"

/**
 * @description: 无感高频方波注入的初始化函数
 * @reminder: (初始化相关系数float->double->float)
 * @reminder: (单浮点转double运算，提高系数精度)
 * @param {HFI_STRUCT} *hfi
 * @return {*}
 */
void HFI_Init(HFI_STRUCT *hfi){
    // 1.陷波传递函数中间参数的初始化设置
    double temp0 = ((double)hfi->T)*((double)hfi->Wo)*((double)hfi->K1);
    double temp1 = ((double)hfi->T)*((double)hfi->Wo)*((double)hfi->K2);
    double temp2 = ((double)hfi->T)*((double)hfi->Wo)*
                ((double)hfi->T)*((double)hfi->Wo);
    double den1 = 4.0+4.0*temp0+temp2;
    double den2 = 4.0+8.0*temp0+4.0*temp2;

    hfi->go.s1_num[0] = (float)((4.0+4.0*temp1+temp2)/den1);
    hfi->go.s1_num[1] = (float)((-8.0+2.0*temp2)/den1);
    hfi->go.s1_num[2] = (float)((4.0-4.0*temp1+temp2)/den1);
    hfi->go.s1_den = (float)((4.0-4.0*temp0+temp2)/den1);

    hfi->go.s2_num[0] = (float)((4.0+8.0*temp1+4.0*temp2)/den2);
    hfi->go.s2_num[1] = (float)((-8.0+8.0*temp2)/den2);
    hfi->go.s2_num[2] = (float)((4.0-8.0*temp1+4.0*temp2)/den2);
    hfi->go.s2_den = (float)((4.0-8.0*temp0+4.0*temp2)/den2);

    // 2.初始化陷波历史数值
    hfi->data0.x_i[0] = 0.0f;
    hfi->data0.x_i[1] = 0.0f;
    hfi->data0.x_o[0] = 0.0f;
    hfi->data0.x_o[1] = 0.0f;

    hfi->data0.y_i[0] = 0.0f;
    hfi->data0.y_i[1] = 0.0f;
    hfi->data0.y_o[0] = 0.0f;
    hfi->data0.y_o[1] = 0.0f;

    hfi->data1.y_i[0] = 0.0f;
    hfi->data1.y_i[1] = 0.0f;
    hfi->data1.y_o[0] = 0.0f;
    hfi->data1.y_o[1] = 0.0f;

    // 3.初始化陷波输入输出
    hfi->go.Input_Id = 0.0f;
    hfi->go.Input_Iq = 0.0f;
    hfi->go.Output_Id = 0.0f;
    hfi->go.Output_Iq = 0.0f;

    // 4.带通传递函数中间参数的初始化设置
    double p_temp1 = ((double)hfi->T)*((double)hfi->Wo)*(((double)hfi->zeta)*4.0);
    double p_temp2 = ((double)hfi->T)*((double)hfi->Wo)*
                    ((double)hfi->T)*((double)hfi->Wo);

    double den = p_temp2+p_temp1+4.0;

    hfi->go.p_num = (float)(p_temp1/den);
    hfi->go.p_den[0] = (float)((-8.0+2.0*p_temp2)/den);
    hfi->go.p_den[1] = (float)((p_temp2-p_temp1+4.0)/den);

    // 5.初始化带通历史数值
    hfi->data0.x_i[0] = 0.0f;
    hfi->data0.x_i[1] = 0.0f;
    hfi->data0.x_o[0] = 0.0f;
    hfi->data0.x_o[1] = 0.0f;

    
    // 6.初始化带通输入输出
    hfi->go.Input_Q = 0.0f;
    hfi->go.Output_Q = 0.0f;

    // 7.初始化注入电压值及其余
    hfi->go.Output_Uin = 0.0f;
    hfi->go.Angle = 0.0f;
}

/**
 * @description: 无感高频方波注入的离散运行函数
 * @reminder: (处理Q轴坐标系下的高频信号)
 * @reminder: https://github.com/Sguan-ZhouQing/SguanFOC_Library/blob/main/%E9%85%8D%E5%A5%97Simulink%E6%A8%A1%E5%9E%8B%E5%BC%80%E6%BA%90%E2%91%A1%5B%E7%AE%97%E6%B3%95%E5%8E%9F%E7%90%86%E5%9B%BE%5D/Sguan_HFI.png
 * @reminder: (上方链接是此Sguan_HFI模块Simulink原理仿真图)
 * @param {HFI_STRUCT} *hfi
 * @return {*}
 */
void HFI_ReadRad_Loop(HFI_STRUCT *hfi){
    // 1.带通滤波，得到高频电流分量
    hfi->go.High = hfi->go.p_num*(hfi->go.Input_Q - hfi->data1.x_i[1]) - 
                        hfi->go.p_den[0]*hfi->data1.x_o[0] - 
                        hfi->go.p_den[1]*hfi->data1.x_o[1];

    // 2.更新历史输入和输出数值
    hfi->data1.x_i[1] = hfi->data1.x_i[0];
    hfi->data1.x_i[0] = hfi->go.Input_Q;
    hfi->data1.x_o[1] = hfi->data1.x_o[0];
    hfi->data1.x_o[0] = hfi->go.High;

    // 3.同频相乘，得到待解调的高频量
    float TPNF_in = hfi->go.High*hfi->h*hfi->go.Sine[1];

    // 4.陷波滤波，滤除二倍高频分量
    hfi->go.Output_Q = hfi->go.s2_num[0]*TPNF_in + 
                        hfi->go.s2_num[1]*(hfi->data1.y_i[0]-hfi->data1.y_o[0]) + 
                        hfi->go.s2_num[2]*hfi->data1.y_i[1] - 
                        hfi->go.s2_den*hfi->data1.y_o[1];

    // 5.更新历史输入和输出数值
    hfi->data1.y_i[1] = hfi->data1.y_i[0];
    hfi->data1.y_i[0] = TPNF_in;
    hfi->data1.y_o[1] = hfi->data1.y_o[0];
    hfi->data1.y_o[0] = hfi->go.Output_Q;
    hfi->go.Sine[1] = hfi->go.Sine[0];

    // 6.刷新注入信号
    hfi->go.Angle += hfi->Wo*hfi->T;
    hfi->go.Angle = Value_normalize(hfi->go.Angle);
    float Cosine;
    fast_sin_cos(hfi->go.Angle, &hfi->go.Sine[0], &Cosine);
    hfi->go.Output_Uin = Cosine*hfi->Uh;
}

/**
 * @description: 无感高频方波注入的离散运行函数
 * @reminder: (处理DQ轴电流环的基频信号量)
 * @reminder: https://github.com/Sguan-ZhouQing/SguanFOC_Library/blob/main/%E9%85%8D%E5%A5%97Simulink%E6%A8%A1%E5%9E%8B%E5%BC%80%E6%BA%90%E2%91%A1%5B%E7%AE%97%E6%B3%95%E5%8E%9F%E7%90%86%E5%9B%BE%5D/Sguan_HFI.png
 * @reminder: (上方链接是此Sguan_HFI模块Simulink原理仿真图)
 * @param {HFI_STRUCT} *hfi
 * @return {*}
 */
void HFI_Current_Loop(HFI_STRUCT *hfi){
    // 1.带入差分方程，计算输出
    hfi->go.Output_Id = hfi->go.s1_num[0]*hfi->go.Input_Id + 
                        hfi->go.s1_num[1]*(hfi->data0.x_i[0]-hfi->data0.x_o[0]) + 
                        hfi->go.s1_num[2]*hfi->data0.x_i[1] - 
                        hfi->go.s1_den*hfi->data0.x_o[1];

    hfi->go.Output_Iq = hfi->go.s1_num[0]*hfi->go.Input_Iq + 
                        hfi->go.s1_num[1]*(hfi->data0.y_i[0]-hfi->data0.y_o[0]) + 
                        hfi->go.s1_num[2]*hfi->data0.y_i[1] - 
                        hfi->go.s1_den*hfi->data0.y_o[1];

    // 2.更新历史输入和输出数值
    hfi->data0.x_i[1] = hfi->data0.x_i[0];
    hfi->data0.x_i[0] = hfi->go.Input_Id;
    hfi->data0.x_o[1] = hfi->data0.x_o[0];
    hfi->data0.x_o[0] = hfi->go.Output_Id;

    hfi->data0.y_i[1] = hfi->data0.y_i[0];
    hfi->data0.y_i[0] = hfi->go.Input_Iq;
    hfi->data0.y_o[1] = hfi->data0.y_o[0];
    hfi->data0.y_o[0] = hfi->go.Output_Iq;
}

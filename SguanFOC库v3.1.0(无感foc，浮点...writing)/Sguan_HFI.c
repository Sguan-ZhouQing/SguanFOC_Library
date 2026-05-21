/*
 * @Author: 星必尘Sguan
 * @GitHub: https://github.com/Sguan-ZhouQing
 * @Date: 2026-04-29 19:43:41
 * @LastEditors: 星必尘Sguan|3464647102@qq.com
 * @LastEditTime: 2026-04-30 01:57:22
 * @FilePath: \SguanFOC_Debug\SguanFOC\Sguan_HFI.c
 * @Description: SguanFOC库的“HFI无感高频注入算法”实现
 * 
 * Copyright (c) 2026 by $星必尘Sguan, All Rights Reserved. 
 */
#include "Sguan_HFI.h"

/**
 * @description: 无感高频方波注入的初始化函数
 * @param {SMO_STRUCT} *smo
 * @return {*}
 */
void HFI_Init(HFI_STRUCT *hfi){
    // 1.陷波传递函数中间参数的初始化设置
    double s_temp1 = ((double)hfi->T)*((double)hfi->Wo)*(((double)hfi->zeta0)*4.0);
    double s_temp2 = ((double)hfi->T)*((double)hfi->Wo)*
                    ((double)hfi->T)*((double)hfi->Wo);
    hfi->go.s_num[0] = (float)((s_temp2+4.0)/(s_temp2+s_temp1+4.0));
    hfi->go.s_num[1] = (float)((-8.0+2.0*s_temp2)/(s_temp2+s_temp1+4.0));
    hfi->go.s_den[0] = (float)(-8.0+2.0*s_temp2)/(s_temp2+s_temp1+4.0);
    hfi->go.s_den[1] = (float)(s_temp2-s_temp1+4.0)/(s_temp2+s_temp1+4.0);

    // 2.初始化陷波历史数值
    hfi->data0.s_i[0] = 0.0f;
    hfi->data0.s_i[1] = 0.0f;
    hfi->data0.s_o = 0.0f;
    hfi->data1.s_i[0] = 0.0f;
    hfi->data1.s_i[1] = 0.0f;
    hfi->data1.s_o = 0.0f;

    // 3.初始化陷波输入输出
    hfi->go.Input_Id = 0.0f;
    hfi->go.Input_Iq = 0.0f;
    hfi->go.Output_Id = 0.0f;
    hfi->go.Output_Iq = 0.0f;

    // 4.带通传递函数中间参数的初始化设置
    double p_temp1 = ((double)hfi->T)*((double)hfi->Wo)*(((double)hfi->zeta1)*4.0);
    double p_temp2 = ((double)hfi->T)*((double)hfi->Wo)*
                    ((double)hfi->T)*((double)hfi->Wo);
    hfi->go.p_num = (float)(p_temp1/(p_temp2+p_temp1+4.0));
    hfi->go.p_den[0] = (float)(-8.0+2.0*p_temp2)/(p_temp2+p_temp1+4.0);
    hfi->go.p_den[1] = (float)(p_temp2-p_temp1+4.0)/(p_temp2+p_temp1+4.0);

    // 5.初始化带通历史数值
    hfi->data0.p_i[0] = 0.0f;
    hfi->data0.p_i[1] = 0.0f;
    hfi->data0.p_o[0] = 0.0f;
    hfi->data0.p_o[1] = 0.0f;
    hfi->data1.p_i[0] = 0.0f;
    hfi->data1.p_i[1] = 0.0f;
    hfi->data1.p_o[0] = 0.0f;
    hfi->data1.p_o[1] = 0.0f;
    
    // 6.初始化带通输入输出
    hfi->go.Input_Ialpha = 0.0f;
    hfi->go.Input_Ibeta = 0.0f;
    hfi->go.Output_Ialpha = 0.0f;
    hfi->go.Output_Ibeta = 0.0f;

    // 7.初始化注入电压值及其余
    hfi->go.Output_Uin = 0.0f;
    hfi->go.Angle = 0.0f;
    hfi->go.Sine = 0.0f;
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
    hfi->go.Output_Id = hfi->go.s_num[0]*(hfi->go.Input_Id+hfi->data0.s_i[1]) + 
                        hfi->go.s_num[1]*hfi->data0.s_i[0] - 
                        hfi->go.s_den[0]*hfi->go.Output_Id - 
                        hfi->go.s_den[1]*hfi->data0.s_o;

    hfi->go.Output_Iq = hfi->go.s_num[0]*(hfi->go.Input_Iq+hfi->data1.s_i[1]) + 
                        hfi->go.s_num[1]*hfi->data1.s_i[0] - 
                        hfi->go.s_den[0]*hfi->go.Output_Iq - 
                        hfi->go.s_den[1]*hfi->data1.s_o;

    // 2.更新历史输入和输出数值
    hfi->data0.s_i[1] = hfi->data0.s_i[0];
    hfi->data0.s_i[0] = hfi->go.Input_Id;
    hfi->data0.s_o = hfi->go.Output_Id;

    hfi->data1.s_i[1] = hfi->data1.s_i[0];
    hfi->data1.s_i[0] = hfi->go.Input_Iq;
    hfi->data1.s_o = hfi->go.Output_Iq;
}

/**
 * @description: 无感高频方波注入的离散运行函数
 * @reminder: (处理静止坐标系下的高频信号)
 * @reminder: https://github.com/Sguan-ZhouQing/SguanFOC_Library/blob/main/%E9%85%8D%E5%A5%97Simulink%E6%A8%A1%E5%9E%8B%E5%BC%80%E6%BA%90%E2%91%A1%5B%E7%AE%97%E6%B3%95%E5%8E%9F%E7%90%86%E5%9B%BE%5D/Sguan_HFI.png
 * @reminder: (上方链接是此Sguan_HFI模块Simulink原理仿真图)
 * @param {HFI_STRUCT} *hfi
 * @return {*}
 */
void HFI_ReadRad_Loop(HFI_STRUCT *hfi){
    // 1.带入差分方程，计算输出
    float Ialpha = hfi->go.p_num*(hfi->go.Input_Ialpha - hfi->data0.p_i[1]) - 
                        hfi->go.p_den[0]*hfi->data0.p_o[0] - 
                        hfi->go.p_den[1]*hfi->data0.p_o[1];

    float Ibeta = hfi->go.p_num*(hfi->go.Input_Ibeta - hfi->data1.p_i[1]) - 
                        hfi->go.p_den[0]*hfi->data1.p_o[0] - 
                        hfi->go.p_den[1]*hfi->data1.p_o[1];

    // 2.更新历史输入和输出数值
    hfi->data0.p_i[1] = hfi->data0.p_i[0];
    hfi->data0.p_i[0] = hfi->go.Input_Ialpha;
    hfi->data0.p_o[1] = hfi->data0.p_o[0];
    hfi->data0.p_o[0] = hfi->go.Output_Ialpha;

    hfi->data1.p_i[1] = hfi->data1.p_i[0];
    hfi->data1.p_i[0] = hfi->go.Input_Ibeta;
    hfi->data1.p_o[1] = hfi->data1.p_o[0];
    hfi->data1.p_o[0] = hfi->go.Output_Ibeta;

    // 3.输出待解调的高频量
    hfi->go.Output_Ialpha = Ialpha*hfi->go.Sine;
    hfi->go.Output_Ibeta = Ibeta*hfi->go.Sine;

    // 4.更新注入信号
    Value_Rad_Loop(&hfi->go.Angle, hfi->Wo, hfi->T);
    hfi->go.Sine = fast_sin(hfi->go.Angle);
    hfi->go.Output_Uin = hfi->go.Sine*hfi->Uh;
}

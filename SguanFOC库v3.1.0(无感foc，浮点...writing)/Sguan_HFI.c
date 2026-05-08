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

// 局部静态函数的声明
static float HFI_ToggleVBUS_Loop(float offset);
static void HFI_ReadHigh_Loop(HFI_GO_STRUCT *hfi);
static void HFI_ReadBase_Loop(HFI_GO_STRUCT *hfi);

// 高频信号获取(翻转电平)
static float HFI_ToggleVBUS_Loop(float offset){
    static uint8_t Toggle_flag = 0;
    Toggle_flag = !Toggle_flag;
    if (Toggle_flag){
        return offset; 
    }
    else{
        return -offset; 
    }

}

// 读取高频电流分量
static void HFI_ReadHigh_Loop(HFI_GO_STRUCT *h){
    // 计算高频分量
    h->Output = (h->Input - 2.0f*h->i[0] + h->i[1])/4.0f;
    // 更新历史输入值
    h->i[1] = h->i[0];
    h->i[0] = h->Input;
}

// 读取基频电流分量
static void HFI_ReadBase_Loop(HFI_GO_STRUCT *f){
    // 计算高频分量
    f->Output = (f->Input + 2.0f*f->i[0] + f->i[1])/4.0f;
    // 更新历史输入值
    f->i[1] = f->i[0];
    f->i[0] = f->Input;
}

// 初始化结构体参数
static void HFI_SetZero(HFI_GO_STRUCT *x){
    x->i[0] = 0.0f;
    x->i[1] = 0.0f;

    x->Input = 0.0f;
    x->Output = 0.0f;
}

/**
 * @description: 无感高频方波注入的初始化函数
 * @param {SMO_STRUCT} *smo
 * @return {*}
 */
void HFI_Init(HFI_STRUCT *hfi){
    // 初始化为零，data数据
    hfi->data.Last_Ialpha = 0.0f;
    hfi->data.Last_Ibeta = 0.0f;
    hfi->data.Uin = 0.0f;
    hfi->data.Input_Ud = 0.0f;

    // 初始化为零，各个结构体
    HFI_SetZero(&hfi->Id_f);
    HFI_SetZero(&hfi->Iq_f);
    HFI_SetZero(&hfi->Ialpha_h);
    HFI_SetZero(&hfi->Ibeta_h);
}

/**
 * @description: 无感高频方波注入的离散运行函数
 * @reminder: (处理静止坐标系下的高频信号)
 * @reminder: https://github.com/Sguan-ZhouQing/SguanFOC_Library/blob/main/%E9%85%8D%E5%A5%97Simulink%E6%A8%A1%E5%9E%8B%E5%BC%80%E6%BA%90%E2%91%A1%5B%E7%AE%97%E6%B3%95%E5%8E%9F%E7%90%86%E5%9B%BE%5D/Sguan_SMO.png
 * @reminder: (上方链接是此Sguan_HFI模块Simulink原理仿真图)
 * @param {HFI_STRUCT} *hfi
 * @return {*}
 */
void HFI_ReadRad_Loop(HFI_STRUCT *hfi){
    // 传入alpha和beta轴的数据
    HFI_ReadHigh_Loop(&hfi->Ialpha_h);
    HFI_ReadHigh_Loop(&hfi->Ibeta_h);
    hfi->data.Output_Iah = Value_Sign(hfi->data.Uin)*
                    (hfi->Ialpha_h.Output - hfi->data.Last_Ialpha);
    hfi->data.Output_Ibh = Value_Sign(hfi->data.Uin)*
                    (hfi->Ibeta_h.Output - hfi->data.Last_Ibeta);

    // 更新历史输出值
    hfi->data.Last_Ialpha = hfi->Ialpha_h.Output;
    hfi->data.Last_Ibeta = hfi->Ibeta_h.Output;

    // 处理高频信号
    hfi->data.Uin = HFI_ToggleVBUS_Loop(hfi->Percentage_hfi*hfi->data.Input_VBUS);
    hfi->data.Output_Ud = hfi->data.Input_Ud + hfi->data.Uin;
    // 输出hfi->data.Output_Iah
    // 输出hfi->data.Output_Ibh
}

/**
 * @description: 无感高频方波注入的离散运行函数
 * @reminder: (处理DQ轴电流环的基频信号量)
 * @reminder: https://github.com/Sguan-ZhouQing/SguanFOC_Library/blob/main/%E9%85%8D%E5%A5%97Simulink%E6%A8%A1%E5%9E%8B%E5%BC%80%E6%BA%90%E2%91%A1%5B%E7%AE%97%E6%B3%95%E5%8E%9F%E7%90%86%E5%9B%BE%5D/Sguan_SMO.png
 * @reminder: (上方链接是此Sguan_HFI模块Simulink原理仿真图)
 * @param {HFI_STRUCT} *hfi
 * @return {*}
 */
void HFI_Current_Loop(HFI_STRUCT *hfi){
    HFI_ReadBase_Loop(&hfi->Id_f);
    HFI_ReadBase_Loop(&hfi->Iq_f);
    // 输出hfi->Id_f.Output
    // 输出hfi->Iq_f.Output
}


// =========================== 电机极性辨识(NSD) ===================
void NSD_Loop(void){

}


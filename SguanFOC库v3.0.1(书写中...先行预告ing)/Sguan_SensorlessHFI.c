/*
 * @Author: 星必尘Sguan
 * @GitHub: https://github.com/Sguan-ZhouQing
 * @Date: 2026-02-23 03:51:26
 * @LastEditors: 星必尘Sguan|3464647102@qq.com
 * @LastEditTime: 2026-02-23 03:51:26
 * @FilePath: \stm_SguanFOCtest\SguanFOC\Sguan_SensorlessHFI.c
 * @Description: SguanFOC库的“无感控制算法HFI”实现
 * 
 * Copyright (c) 2026 by $星必尘Sguan, All Rights Reserved. 
 */
#include "Sguan_SensorlessHFI.h"

// 原理上实现高频方波注入算法
float HFI_ToggleVBUS(float Ud_Bias, float Ud_Limit){
    static uint8_t count = 0;
    if (count){
        return Ud_Bias + Ud_Limit;
    }
    else{
        return Ud_Bias - Ud_Limit;
    }
}

// 读取高频电流分量
void HFI_ReadHighCurrent(HFI_STRUCT *hfi){
    // 计算高频分量
    hfi->Output = (hfi->Input - 2.0f*hfi->i[0] + hfi->i[1])/4.0f;
    // 更新历史输入值
    hfi->i[1] = hfi->i[0];
    hfi->i[0] = hfi->Input;
}

// 读取基频电流分量
void HFI_ReadFundamentalCurrent(HFI_STRUCT *hfi){
    // 计算高频分量
    hfi->Output = (hfi->Input + 2.0f*hfi->i[0] + hfi->i[1])/4.0f;
    // 更新历史输入值
    hfi->i[1] = hfi->i[0];
    hfi->i[0] = hfi->Input;
}


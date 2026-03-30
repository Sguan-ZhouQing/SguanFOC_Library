/*
 * @Author: 星必尘Sguan
 * @GitHub: https://github.com/Sguan-ZhouQing
 * @Date: 2026-01-26 22:50:37
 * @LastEditors: 星必尘Sguan|3464647102@qq.com
 * @LastEditTime: 2026-03-20 22:57:34
 * @FilePath: \SguanFOC_Debug\SguanFOC\Sguan_PLL.c
 * @Description: SguanFOC库的“开环PLL锁相环”实现
 * 
 * Copyright (c) 2026 by $星必尘Sguan, All Rights Reserved. 
 */
#include "Sguan_PLL.h"

// 锁相环PLL核心参数初始化，主函数调用
void PLL_Init(PLL_STRUCT *pll){
    double temp0 = pll->T*pll->Ki;
    pll->go.X_num[0] = (float)((2*pll->Kp+temp0)/2.0);
    pll->go.X_num[1] = (float)((-2*pll->Kp+temp0)/2.0);
    pll->go.Y_num = (float)(pll->T/2.0);
    // 初始化为零
    pll->is_position_mode = 0; // 默认非位置环mode
    pll->go.i = 0;
    pll->go.Xo = 0;
    pll->go.Yo = 0;
    pll->go.OutWe = 0;
    pll->go.OutRe = 0;
    pll->go.Error = 0;
}

// 闭环控制运算的定时器中断服务函数
void PLL_Loop(PLL_STRUCT *pll){
    // 计算PI控制器(并输出We)
    pll->go.OutWe = pll->go.X_num[0]*pll->go.Error + pll->go.X_num[1]*pll->go.i 
                + pll->go.Xo;
    // 计算积分器(并输出Re)
    pll->go.OutRe = pll->go.Y_num*pll->go.OutWe + pll->go.Y_num*pll->go.Xo 
                + pll->go.Yo;
    if (!pll->is_position_mode){
        // 非位置环模式：使用normalize_angle函数归一化到[0, 2π)
        pll->go.OutRe = Value_normalize(pll->go.OutRe);
    }
    // 更新历史输入和输出数值
    pll->go.i = pll->go.Error;
    pll->go.Xo = pll->go.OutWe;
    pll->go.Yo = pll->go.OutRe;
}


// ============================ Q31 版本代码 ============================

void PLL_Init_q31(PLL_STRUCT_q31 *pll){
    double temp0 = pll->T*pll->Ki;
    pll->go.X_num[0] = IQmath_Q31_from_float((float)((2*pll->Kp+temp0)/2.0),BASE_Hz*BASE_PLL_Num);
    pll->go.X_num[1] = IQmath_Q31_from_float((float)((-2*pll->Kp+temp0)/2.0),BASE_Hz*BASE_PLL_Num);
    pll->go.Y_num = IQmath_Q31_from_float((float)(pll->T/2.0),BASE_Time);

    pll->is_position_mode = 0;
    pll->go.i = 0;
    pll->go.Xo = 0;
    pll->go.Yo = 0;
    pll->go.OutWe = 0;
    pll->go.OutRe = 0;
    pll->go.Error = 0;
}

void PLL_Loop_q31(PLL_STRUCT_q31 *pll){
    Q31_t OutWe = IQmath_Q31_mul(pll->go.X_num[0],pll->go.Error) + 
                IQmath_Q31_mul(pll->go.X_num[1],pll->go.i) + 
                IQmath_Q31_mul(pll->go.Xo,INT32_PLL_Num_16);
    pll->go.OutWe = IQmath_Q31_Shift_Left(OutWe,SHIFT_PLL_Num);

    Q31_t OutRe = IQmath_Q31_mul(pll->go.Y_num,pll->go.OutWe) + 
                IQmath_Q31_mul(pll->go.Y_num,pll->go.Xo) + pll->go.Yo;
    pll->go.OutRe = IQmath_Q31_Limit(OutRe);

    if (!pll->is_position_mode){
        pll->go.OutRe = Value_normalize_q31(pll->go.OutRe);
    }

    pll->go.i = pll->go.Error;
    pll->go.Xo = pll->go.OutWe;
    pll->go.Yo = pll->go.OutRe;
}


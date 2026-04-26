/*
 * @Author: 星必尘Sguan
 * @GitHub: https://github.com/Sguan-ZhouQing
 * @Date: 2026-01-26 22:50:37
 * @LastEditors: 星必尘Sguan|3464647102@qq.com
 * @LastEditTime: 2026-04-22 15:26:12
 * @FilePath: \SguanFOC_Debug\SguanFOC\Sguan_PLL.c
 * @Description: SguanFOC库的“开环PLL锁相环”实现
 * 
 * Copyright (c) 2026 by $星必尘Sguan, All Rights Reserved. 
 */
#include "Sguan_PLL.h"

/**
 * @description: 锁相环PLL核心参数初始化
 * @param {PLL_STRUCT} *pll
 * @return {*}
 */
void PLL_Init(PLL_STRUCT *pll){
    double temp0 = pll->T*pll->Ki;
    pll->go.X_num[0] = (float)((2*pll->Kp+temp0)/2.0);
    pll->go.X_num[1] = (float)((-2*pll->Kp+temp0)/2.0);
    pll->go.Y_num = (float)(pll->T/2.0);
    // 初始化为零
    pll->is_position_mode = 0; // 默认非位置环mode
    pll->go.We_i = 0.0f;
    pll->go.Re_i = 0.0f;
    pll->go.OutWe = 0.0f;
    pll->go.OutRe = 0.0f;
    pll->go.Error = 0.0f;
}

/**
 * @description: 锁相环运算的离散函数
 * @param {PLL_STRUCT} *pll
 * @return {*}
 */
void PLL_Loop(PLL_STRUCT *pll){
    // 1.计算PI控制器(并输出We)
    pll->go.OutWe += pll->go.X_num[0]*pll->go.Error + 
                    pll->go.X_num[1]*pll->go.We_i;

    // 2.计算积分器(并输出Re)
    pll->go.OutRe += pll->go.Y_num*(pll->go.OutWe + pll->go.Re_i);
    if (!pll->is_position_mode){
        // 非位置环模式：使用normalize_angle函数归一化到[0, 2π)
        pll->go.OutRe = Value_normalize(pll->go.OutRe);
    }

    // 3.更新历史输入和输出数值
    pll->go.We_i = pll->go.Error;
    pll->go.Re_i = pll->go.OutWe;
}


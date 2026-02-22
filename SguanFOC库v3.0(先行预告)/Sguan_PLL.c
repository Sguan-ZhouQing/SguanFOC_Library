/*
 * @Author: 星必尘Sguan
 * @GitHub: https://github.com/Sguan-ZhouQing
 * @Date: 2026-01-26 22:50:37
 * @LastEditors: 星必尘Sguan|3464647102@qq.com
 * @LastEditTime: 2026-02-14 00:40:33
 * @FilePath: \stm_SguanFOCtest\SguanFOC\Sguan_PLL.c
 * @Description: SguanFOC库的“开环PLL锁相环”实现
 * 
 * Copyright (c) 2026 by $星必尘Sguan, All Rights Reserved. 
 */
#include "Sguan_PLL.h"

// 锁相环PLL核心参数初始化，主函数调用
void PLL_Init(PLL_STRUCT *pll){
    double temp0 = pll->T*pll->Ki;
    pll->go.X_num[0] = (float)(2*pll->Kp+temp0);
    pll->go.X_num[1] = (float)(-2*pll->Kp+temp0);
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
    pll->go.OutWe = (pll->go.X_num[0]*pll->go.Error + pll->go.X_num[1]*pll->go.i 
                + 2.0f*pll->go.Xo) / 2.0f;
    // 计算积分器(并输出Re)
    pll->go.OutRe = (pll->T*pll->go.OutWe + pll->T*pll->go.Xo 
                + 2.0f*pll->go.Yo) / 2.0f;
    if (!pll->is_position_mode){
        // 非位置环模式：使用normalize_angle函数归一化到[0, 2π)
        pll->go.OutRe = normalize_angle(pll->go.OutRe);
    }
    // 更新历史输入和输出数值
    pll->go.i = pll->go.Error;
    pll->go.Xo = pll->go.OutWe;
    pll->go.Yo = pll->go.OutRe;
}



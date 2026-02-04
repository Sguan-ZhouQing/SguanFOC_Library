/*
 * @Author: 星必尘Sguan
 * @GitHub: https://github.com/Sguan-ZhouQing
 * @Date: 2026-01-26 22:50:37
 * @LastEditors: 星必尘Sguan|3464647102@qq.com
 * @LastEditTime: 2026-02-04 01:26:02
 * @FilePath: \demo_SguanFOCCode\SguanFOC库\Sguan_PLL.c
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
    pll->go.X_den[0] = (float)2;
    pll->go.X_den[1] = (float)(-2);
    pll->go.Y_num[0] = (float)pll->T;
    pll->go.Y_num[1] = (float)pll->T;
    pll->go.Y_den[0] = (float)2;
    pll->go.Y_den[1] = (float)(-2);
    // 初始化为零
    for (int n = 0; n < 2; n++){
        pll->go.i[n] = 0;
        pll->go.Xo[n] = 0;
        pll->go.Yo[n] = 0;
    }
}

// 闭环控制运算的定时器中断服务函数
void PLL_Loop(PLL_STRUCT *pll){
    // 更新历史输入和输出数值
    pll->go.i[1] = pll->go.i[0];
    pll->go.Xo[1] = pll->go.Xo[0];
    pll->go.Yo[1] = pll->go.Yo[0];
    // 更新当前输入
    pll->go.i[0] = pll->go.Error;
    // 计算PI控制器(并输出We)
    pll->go.Xo[0] = (pll->go.X_num[0]*pll->go.i[0] + pll->go.X_num[1]*pll->go.i[1] 
                - pll->go.X_den[1]*pll->go.Xo[1]) / pll->go.X_den[0];
    pll->go.OutWe = pll->go.Xo[0];
    // 计算积分器(并输出Re)
    pll->go.Yo[0] = (pll->go.Y_num[0]*pll->go.Xo[0] + pll->go.Y_num[1]*pll->go.Xo[1] 
                - pll->go.Y_den[1]*pll->go.Yo[1]) / pll->go.Y_den[0];
    pll->go.OutRe = pll->go.Yo[0];
}



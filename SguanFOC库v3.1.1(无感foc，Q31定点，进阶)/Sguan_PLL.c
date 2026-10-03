/*
 * @Author: 星必尘Sguan
 * @GitHub: https://github.com/Sguan-ZhouQing
 * @Date: 2026-01-26 22:50:37
 * @LastEditors: 星必尘Sguan|3464647102@qq.com
 * @LastEditTime: 2026-04-01 03:45:08
 * @FilePath: \SguanFOC_Debug\SguanFOC\Sguan_PLL.c
 * @Description: SguanFOC库的“开环PLL锁相环”实现
 * 
 * Copyright (c) 2026 by $星必尘Sguan, All Rights Reserved. 
 */
#include "Sguan_PLL.h"

// 锁相环初始化函数
void PLL_Init(PLL_STRUCT *pll){
    // 1.数学Q31数值转换
    // (数值1.0f->机械角速度)
    // 数值转换条件：
    // (pll->go.Error单位是1, pll->go.Io单位是s)
    // (pll->go.OutWe单位是Rad/s, pll->go.OutRe单位是Rad)
    // (那么pll->go.Kp单位是Rad/s, pll->go.Ki单位是Rad/(s^2))
    pll->go.Kp = iqmath_from_float(pll->Kp, BASE_Speed);
    pll->go.Ki = iqmath_from_float(pll->Kp, (BASE_Speed/BASE_Time));
    
    pll->go.T = PMSM_RUN_T_q31;

    // .....................................................................
    // 范围计算Kp       实际值：650.0       单位范围：BASE_Speed
    // 范围计算Ki       实际值：210000.0    单位范围：(BASE_Speed/BASE_Time)

    // 2.初始化为零
    pll->go.OutWe = 0;
    pll->go.OutRe = 0;
    pll->go.Error = 0;
}

// 锁相环运算的离散函数
void PLL_Loop(PLL_STRUCT *pll){
    // 1.计算PI控制器(并输出We)
    pll->go.Io = iqmath_add(
        iqmath_mul(pll->go.Error, pll->go.T), 
        pll->go.Io);

    pll->go.OutWe = iqmath_add(
        iqmath_mul(pll->go.Kp, pll->go.Error), 
        iqmath_mul(pll->go.Ki, pll->go.Io));

    // 2.计算积分器(并输出Re)
    pll->go.OutRe = iqmath_add(
        iqmath_mul(pll->go.OutWe, pll->go.T), 
        pll->go.OutRe);

    // 3.非位置环模式：使用normalize_angle函数归一化到[0, 2π)
    pll->go.OutRe = iqmath_normalize(pll->go.OutRe);
}



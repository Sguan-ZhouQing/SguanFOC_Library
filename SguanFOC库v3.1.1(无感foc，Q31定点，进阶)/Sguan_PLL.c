/* ===================================================================
 * 文件:     Sguan_PLL.c
 * 项目:     SguanFOC_v3.1.1 —— 开源 FOC 电机控制框架（无感FOC · Q31定点 · 进阶）
 * 链接:     https://github.com/Sguan-ZhouQing/SguanFOC_Library
 * 版权:     Copyright 2026 星必尘Sguan <3464647102@qq.com>
 * 许可证:   MIT License
 * SPDX:     SPDX-License-Identifier: MIT
 *
 * 许可说明:
 *   [EN] Permission is hereby granted, free of charge, to use, copy,
 *        modify, merge, publish, distribute, sublicense, and/or sell
 *        copies of this software, provided this notice is retained.
 *   [中文] 本软件依据 MIT 许可证授权：可免费使用、复制、修改、合并、
 *        出版、分发、再授权及销售，仅须保留本版权与许可声明。
 *   [DE] Hiermit wird jeder Person kostenlos erlaubt, diese Software zu
 *        nutzen, zu kopieren, zu veraendern, zu veroeffentlichen, zu
 *        vertreiben, zu unterlizenzieren und/oder zu verkaufen, sofern
 *        dieser Hinweis erhalten bleibt.
 *   [JA] 本ソフトウェアは MIT ライセンスに基づき、無償で使用・複製・
 *        改変・公開・配布・再許諾・販売が許可されます。ただし本表示を
 *        保持する必要があります。
 * 完整许可文本: 参见仓库根目录下的 LICENSE-MIT 文件。
 * =================================================================== */
#include "Sguan_PLL.h"

// 锁相环初始化函数
void PLL_Init(PLL_STRUCT *pll){
    // 1.数学Q31数值转换
    // (数值1.0f->机械角速度)
    // 数值转换条件：
    // (pll->go.Error单位是1, pll->go.Io单位是s)
    // (pll->go.OutWe单位是Rad/s, pll->go.OutRe单位是Rad)
    // (那么pll->go.Kp单位是Rad/s, pll->go.Ki单位是Rad/(s^2))f
    pll->go.Kp = iqmath_from_float(pll->Kp, BASE_Speed);
    pll->go.Ki = iqmath_from_float(pll->Ki, (BASE_Speed/BASE_Time));
    
    pll->go.T = iqmath_from_float(pll->T, BASE_Time);;

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



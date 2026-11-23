/* ===================================================================
 * 文件:     Sguan_PLL.c
 * 项目:     SguanFOC_v3.0.1 —— 开源 FOC 电机控制框架（有感FOC · Q31定点 · 初阶）
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

uint8_t PLL_Init_q31(PLL_STRUCT_q31 *pll){
    double temp0 = pll->T*pll->Ki;
    pll->go.X_num[0] = IQmath_Q31_from_float((float)((2*pll->Kp+temp0)/2.0),(BASE_Hz*BASE_PLL_Num));
    pll->go.X_num[1] = IQmath_Q31_from_float((float)((-2*pll->Kp+temp0)/2.0),(BASE_Hz*BASE_PLL_Num));
    pll->go.Y_num = IQmath_Q31_from_float((float)(pll->T/2.0),BASE_Time);

    pll->is_position_mode = 0;
    pll->go.i = 0;
    pll->go.Xo = 0;
    pll->go.Yo = 0;
    pll->go.OutWe = 0;
    pll->go.OutRe = 0;
    pll->go.Error = 0;

    if ((((2*pll->Kp+temp0)/2.0) <= (BASE_Hz*BASE_PLL_Num)) && 
        (((-2*pll->Kp+temp0)/2.0) <= (BASE_Hz*BASE_PLL_Num)) && 
        ((pll->T/2.0) <= BASE_Time)){
        return 0x00;
    }
    else{
        return 0x01;
    }
}

void PLL_Loop_q31(PLL_STRUCT_q31 *pll){
    Q31_t OutWe = IQmath_Q31_mul(pll->go.X_num[0],pll->go.Error) + 
                IQmath_Q31_mul(pll->go.X_num[1],pll->go.i) + 
                IQmath_Q31_mul(pll->go.Xo,INT32_PLL_Num_16);
    pll->go.OutWe = IQmath_Q31_Shift_Left(OutWe,SHIFT_PLL_Num);

    pll->go.OutRe = IQmath_Q31_mul(pll->go.Y_num,pll->go.OutWe) + 
                IQmath_Q31_mul(pll->go.Y_num,pll->go.Xo) + pll->go.Yo;

    if (!pll->is_position_mode){
        pll->go.OutRe = Value_normalize_q31(pll->go.OutRe);
    }

    pll->go.i = pll->go.Error;
    pll->go.Xo = pll->go.OutWe;
    pll->go.Yo = pll->go.OutRe;
}


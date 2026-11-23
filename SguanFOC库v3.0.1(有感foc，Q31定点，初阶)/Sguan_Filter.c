/* ===================================================================
 * 文件:     Sguan_Filter.c
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
#include "Sguan_Filter.h"

// 二阶典型环节参数初始化，主函数调用
void LPF_Init(LPF_STRUCT *lpf){
    double temp1 = lpf->T*lpf->Wc*2.828427124746f;
    double temp2 = lpf->T*lpf->T*lpf->Wc*lpf->Wc;
    lpf->filter.num[0] = (float)temp2;
    lpf->filter.num[1] = (float)(2*temp2);
    lpf->filter.num[2] = (float)temp2;
    lpf->filter.den[0] = (float)(temp2+temp1+4);
    lpf->filter.den[1] = (float)(-8+2*temp2);
    lpf->filter.den[2] = (float)(temp2-temp1+4);
    // 初始化为零
    for (int n = 0; n < 3; n++){
        lpf->filter.i[n] = 0;
        lpf->filter.o[n] = 0;
    }
    lpf->filter.Input = 0;
    lpf->filter.Output = 0;
}

// 定时器1ms中断服务函数
void LPF_Loop(LPF_STRUCT *lpf){
    // 更新历史输入和输出数值
    for (int n = 2; n > 0; n--){
        lpf->filter.i[n] = lpf->filter.i[n-1];
        lpf->filter.o[n] = lpf->filter.o[n-1];
    }

    // 更新当前输入,计算输出
    lpf->filter.i[0] = lpf->filter.Input;
    float num = lpf->filter.num[0] * lpf->filter.i[0] + 
                lpf->filter.num[1] * lpf->filter.i[1] + 
                lpf->filter.num[2] * lpf->filter.i[2];
    float den = lpf->filter.den[1] * lpf->filter.o[1] + 
                lpf->filter.den[2] * lpf->filter.o[2];

    // 安全检查并输出结果，避免除以零或产生NaN/Inf
    if (lpf->filter.den[0] != 0.0f && !Value_isnan(den) && !Value_isinf(den)) {
        lpf->filter.o[0] = (num - den) / lpf->filter.den[0];
    }
    if (Value_isnan(lpf->filter.o[0]) || Value_isinf(lpf->filter.o[0])) {
        lpf->filter.o[0] = 0.0f;
    }
    lpf->filter.Output = lpf->filter.o[0];
}


// ============================ Q31 版本代码 ============================

uint8_t LPF_Init_q31(LPF_STRUCT_q31 *lpf){
    double temp1 = lpf->T*lpf->Wc*2.828427124746;
    double temp2 = lpf->T*lpf->T*lpf->Wc*lpf->Wc;
    lpf->filter.num[0] = IQmath_Q31_from_float((float)(temp2/(temp2+temp1+4.0)),BASE_Filter_Num);
    lpf->filter.num[1] = IQmath_Q31_from_float((float)((2.0*temp2)/(temp2+temp1+4.0)),BASE_Filter_Num);
    lpf->filter.num[2] = IQmath_Q31_from_float((float)(temp2/(temp2+temp1+4.0)),BASE_Filter_Num);
    lpf->filter.den[0] = IQmath_Q31_from_float((float)((-8.0+2.0*temp2)/(temp2+temp1+4.0)),BASE_Filter_Num);
    lpf->filter.den[1] = IQmath_Q31_from_float((float)((temp2-temp1+4.0)/(temp2+temp1+4.0)),BASE_Filter_Num);

    for (int n = 0; n < 3; n++){
        lpf->filter.i[n] = 0;
        lpf->filter.o[n] = 0;
    }
    lpf->filter.Input = 0;
    lpf->filter.Output = 0;

    if (((temp2/(temp2+temp1+4.0)) <= BASE_Filter_Num) && 
        (((2.0*temp2)/(temp2+temp1+4.0)) <= BASE_Filter_Num) && 
        ((temp2/(temp2+temp1+4.0)) <= BASE_Filter_Num) && 
        (((-8.0+2.0*temp2)/(temp2+temp1+4.0)) <= BASE_Filter_Num) && 
        (((temp2-temp1+4.0)/(temp2+temp1+4.0)) <= BASE_Filter_Num)){
        return 0x00;
    }
    else{
        return 0x01;
    }
}

void LPF_Loop_q31(LPF_STRUCT_q31 *lpf){
    for (int n = 2; n > 0; n--){
        lpf->filter.i[n] = lpf->filter.i[n-1];
        lpf->filter.o[n] = lpf->filter.o[n-1];
    }

    lpf->filter.i[0] = lpf->filter.Input;
    Q31_t x = IQmath_Q31_mul(lpf->filter.num[0],lpf->filter.i[0]) -
            IQmath_Q31_mul(lpf->filter.den[0],lpf->filter.o[1]) +  
            IQmath_Q31_mul(lpf->filter.num[1],lpf->filter.i[1]) + 
            IQmath_Q31_mul(lpf->filter.num[2],lpf->filter.i[2]) - 
            IQmath_Q31_mul(lpf->filter.den[1],lpf->filter.o[2]);

    lpf->filter.o[0] = IQmath_Q31_Shift_Left(x,SHIFT_Filter_Num);
    lpf->filter.Output = lpf->filter.o[0];
}


/* ===================================================================
 * 文件:     Sguan_Optimize.c
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
#include "Sguan_Optimize.h"

// 经典最大转矩电流id计算公式
void MTPA_Loop(float *Target_id,float flux,float Ld,float Lq,float iq){
    *Target_id = - (flux + Value_sqrtf(flux*flux + 
                8*(Ld - Lq)*(Ld - Lq)*iq*iq))/
                ((Ld - Lq)*4);
}

void MTPA_Loop_q31(Q31_t *Target_id, Q31_t flux, Q31_t Ld, Q31_t Lq, Q31_t iq){
    Q31_t Ld_minus_Lq;
    Q31_t temp;
    Q31_t sqrt_input;
    
    Ld_minus_Lq = IQmath_Q31_sub(Ld, Lq);
    
    temp = IQmath_Q31_mul(Ld_minus_Lq, Ld_minus_Lq);
    temp = IQmath_Q31_mul(temp, iq);
    temp = IQmath_Q31_mul(temp, iq);
    temp = IQmath_Q31_mul(temp, 8);
    
    sqrt_input = IQmath_Q31_add(IQmath_Q31_mul(flux, flux), temp);
    temp = Value_sqrt_q31(sqrt_input);
    
    temp = IQmath_Q31_add(flux, temp);
    temp = -temp;
    Ld_minus_Lq = IQmath_Q31_mul(Ld_minus_Lq, 4);

    *Target_id = IQmath_Q31_div(temp, Ld_minus_Lq);
}

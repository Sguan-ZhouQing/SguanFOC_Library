/* ===================================================================
 * 文件:     Sguan_LPF.c
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
#include "Sguan_LPF.h"


// LPF初始化函数
void LPF_Init(LPF_STRUCT *lpf){
    // 1.计算其余传递函数系数
    float num = (float)(((double)lpf->T)*((double)lpf->Wc)/(1.0 + ((double)lpf->T)*((double)lpf->Wc)));
    float den = (float)(1.0/(1.0 + ((double)lpf->T)*((double)lpf->Wc)));

    lpf->go.num = iqmath_from_float(num, 1.0f);
    lpf->go.den = iqmath_from_float(den, 1.0f);

    // 3.初始化为零
    lpf->go.Input = 0;
    lpf->go.Output = 0;
}


// LPF运行函数
void LPF_Loop(LPF_STRUCT *lpf){
    // 1.带入差分方程，计算输出
    // (Output和Input是同一单位，所以num和den是单位为1的比例系数)
    lpf->go.Output = iqmath_add(
        iqmath_mul(lpf->go.Input, lpf->go.num),
        iqmath_mul(lpf->go.Output, lpf->go.den));
}


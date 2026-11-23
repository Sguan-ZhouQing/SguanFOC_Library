/* ===================================================================
 * 文件:     Sguan_Filter.c
 * 项目:     SguanFOC_v3.0.0 —— 开源 FOC 电机控制框架（有感FOC · 浮点运算 · 初阶）
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
void BPF_Init(BPF_STRUCT *bpf){
    double temp1 = bpf->T*bpf->Wc*2.828427124746f;
    double temp2 = bpf->T*bpf->T*bpf->Wc*bpf->Wc;
    bpf->filter.num[0] = (float)temp2;
    bpf->filter.num[1] = (float)(2*temp2);
    bpf->filter.num[2] = (float)temp2;
    bpf->filter.den[0] = (float)(temp2+temp1+4);
    bpf->filter.den[1] = (float)(-8+2*temp2);
    bpf->filter.den[2] = (float)(temp2-temp1+4);
    // 初始化为零
    for (int n = 0; n < 3; n++){
        bpf->filter.i[n] = 0;
        bpf->filter.o[n] = 0;
    }
    bpf->filter.Input = 0;
    bpf->filter.Output = 0;
}

// 定时器中断服务函数
void BPF_Loop(BPF_STRUCT *bpf){
    // 更新历史输入和输出数值
    for (int n = 2; n > 0; n--){
        bpf->filter.i[n] = bpf->filter.i[n-1];
        bpf->filter.o[n] = bpf->filter.o[n-1];
    }

    // 更新当前输入,计算输出
    bpf->filter.i[0] = bpf->filter.Input;
    float num = bpf->filter.num[0] * bpf->filter.i[0] + 
                bpf->filter.num[1] * bpf->filter.i[1] + 
                bpf->filter.num[2] * bpf->filter.i[2];
    float den = bpf->filter.den[1] * bpf->filter.o[1] + 
                bpf->filter.den[2] * bpf->filter.o[2];

    // 安全检查并输出结果，避免除以零或产生NaN/Inf
    if (bpf->filter.den[0] != 0.0f && !Value_isnan(den) && !Value_isinf(den)) {
        bpf->filter.o[0] = (num - den) / bpf->filter.den[0];
    }
    if (Value_isnan(bpf->filter.o[0]) || Value_isinf(bpf->filter.o[0])) {
        bpf->filter.o[0] = 0.0f;
    }
    bpf->filter.Output = bpf->filter.o[0];
}

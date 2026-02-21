/*
 * @Author: 星必尘Sguan
 * @GitHub: https://github.com/Sguan-ZhouQing
 * @Date: 2026-01-26 22:37:25
 * @LastEditors: 星必尘Sguan|3464647102@qq.com
 * @LastEditTime: 2026-02-16 00:47:53
 * @FilePath: \stm_SguanFOCtest\SguanFOC\Sguan_Filter.c
 * @Description: SguanFOC库的“二阶巴特沃斯滤低通滤波器”实现
 * 
 * Copyright (c) 2026 by $星必尘Sguan, All Rights Reserved. 
 */
#include "Sguan_Filter.h"

// 二阶典型环节参数初始化，主函数调用
void BPF_Init(BPF_STRUCT *bpf){
    double temp0 = bpf->T*bpf->Wc*2.828427124746f;
    double temp1 = bpf->T*bpf->Wc;
    double temp2 = bpf->T*bpf->T*bpf->Wc*bpf->Wc;
    bpf->filter.num[0] = (float)temp2;
    bpf->filter.num[1] = (float)(2*temp2);
    bpf->filter.num[2] = (float)temp2;
    bpf->filter.den[0] = (float)(temp2+temp0+4);
    bpf->filter.den[1] = (float)(-8+2*temp2);
    bpf->filter.den[2] = (float)(temp2-temp0+4);
    // 初始化为零
    for (int n = 0; n < 3; n++){
        bpf->filter.i[n] = 0;
        bpf->filter.o[n] = 0;
    }
    bpf->filter.Input = 0;
    bpf->filter.Output = 0;
}

// 定时器1ms中断服务函数
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

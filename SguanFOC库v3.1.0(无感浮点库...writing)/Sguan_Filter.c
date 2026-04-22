/*
 * @Author: 星必尘Sguan
 * @GitHub: https://github.com/Sguan-ZhouQing
 * @Date: 2026-01-26 22:37:25
 * @LastEditors: 星必尘Sguan|3464647102@qq.com
 * @LastEditTime: 2026-04-22 15:23:31
 * @FilePath: \SguanFOC_Debug\SguanFOC\Sguan_Filter.c
 * @Description: SguanFOC库的“二阶巴特沃斯滤低通滤波器”实现
 * 
 * Copyright (c) 2026 by $星必尘Sguan, All Rights Reserved. 
 */
#include "Sguan_Filter.h"

// 二阶典型环节参数初始化，主函数调用

/**
 * @description: 
 * @param {LPF_STRUCT} *lpf
 * @return {*}
 */
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

/**
 * @description: 
 * @param {LPF_STRUCT} *lpf
 * @return {*}
 */
void LPF_Loop(LPF_STRUCT *lpf){
    // 1.更新历史输入和输出数值
    for (int n = 2; n > 0; n--){
        lpf->filter.i[n] = lpf->filter.i[n-1];
        lpf->filter.o[n] = lpf->filter.o[n-1];
    }

    // 2.更新当前输入,计算输出
    lpf->filter.i[0] = lpf->filter.Input;
    float num = lpf->filter.num[0] * lpf->filter.i[0] + 
                lpf->filter.num[1] * lpf->filter.i[1] + 
                lpf->filter.num[2] * lpf->filter.i[2];
    float den = lpf->filter.den[1] * lpf->filter.o[1] + 
                lpf->filter.den[2] * lpf->filter.o[2];

    // 3.安全检查并输出结果，避免除以零或产生NaN/Inf
    if (lpf->filter.den[0] != 0.0f && !Value_isnan(den) && !Value_isinf(den)) {
        lpf->filter.o[0] = (num - den) / lpf->filter.den[0];
    }
    if (Value_isnan(lpf->filter.o[0]) || Value_isinf(lpf->filter.o[0])) {
        lpf->filter.o[0] = 0.0f;
    }
    lpf->filter.Output = lpf->filter.o[0];
}


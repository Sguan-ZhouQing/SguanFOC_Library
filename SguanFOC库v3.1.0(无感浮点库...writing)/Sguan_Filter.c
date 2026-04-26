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

/**
 * @description: 二阶低通滤波器参数初始化
 * @param {LPF_STRUCT} *lpf
 * @return {*}
 */
void LPF_Init(LPF_STRUCT *lpf){
    double temp1 = lpf->T*lpf->Wc*((double)Value_2_SQRT2);
    double temp2 = lpf->T*lpf->T*lpf->Wc*lpf->Wc;
    lpf->filter.num[0] = (float)(temp2/(temp2+temp1+4));
    lpf->filter.num[1] = (float)((2*temp2)/(temp2+temp1+4));
    lpf->filter.num[2] = (float)(temp2/(temp2+temp1+4));
    lpf->filter.den[0] = (float)(-8+2*temp2)/(temp2+temp1+4);
    lpf->filter.den[1] = (float)(temp2-temp1+4)/(temp2+temp1+4);
    // 初始化为零
    lpf->filter.i[0] = 0.0f;
    lpf->filter.i[1] = 0.0f;
    lpf->filter.o[0] = 0.0f;
    lpf->filter.o[1] = 0.0f;
    lpf->filter.Input = 0.0f;
    lpf->filter.Output = 0.0f;
}

/**
 * @description: 滤波器离散服务函数
 * @reminder: https://github.com/Sguan-ZhouQing/SguanFOC_Library/blob/main/%E6%9C%80%E6%96%B0example%E5%8F%8A%E8%B5%84%E6%96%99%5BSTM32G4%2C%E4%B8%8B%E6%A1%A5%E8%87%82%E5%8F%8C%E7%94%B5%E9%98%BB%5D/%E3%80%90Simulink%E3%80%91Sguan%E5%AD%90%E6%A8%A1%E5%9D%97%E5%8E%9F%E7%90%86%E5%9B%BE/Sguan_Filter.png
 * @reminder: (上方链接是此Sguan_Filter模块Simulink原理仿真图)
 * @param {LPF_STRUCT} *lpf
 * @return {*}
 */
void LPF_Loop(LPF_STRUCT *lpf){
    // 1.带入差分方程，计算输出
    lpf->filter.Output = lpf->filter.num[0] * lpf->filter.Input + 
                        lpf->filter.num[1] * lpf->filter.i[0] + 
                        lpf->filter.num[2] * lpf->filter.i[1] - 
                        lpf->filter.den[0] * lpf->filter.o[0] - 
                        lpf->filter.den[1] * lpf->filter.o[1];

    // 1.更新历史输入和输出数值
    lpf->filter.i[1] = lpf->filter.i[0];
    lpf->filter.i[0] = lpf->filter.Input;
    lpf->filter.o[1] = lpf->filter.o[0];
    lpf->filter.o[0] = lpf->filter.Output;
}


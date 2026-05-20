/*
 * @Author: 星必尘Sguan
 * @GitHub: https://github.com/Sguan-ZhouQing
 * @Date: 2026-01-26 22:37:25
 * @LastEditors: 星必尘Sguan|3464647102@qq.com
 * @LastEditTime: 2026-04-30 00:22:21
 * @FilePath: \SguanFOC_Debug\SguanFOC\Sguan_Filter.c
 * @Description: SguanFOC库的“不同滤波器算法”实现
 * 
 * Copyright (c) 2026 by $星必尘Sguan, All Rights Reserved. 
 */
#include "Sguan_Filter.h"

// =============================== LPF低通滤波器 ============================
/**
 * @description: 低通滤波器的参数初始化
 * @reminder: (初始化相关系数float->double->float)
 * @reminder: (单浮点转double运算，提高系数精度)
 * @param {LPF_STRUCT} *lpf
 * @return {*}
 */
void LPF_Init(LPF_STRUCT *lpf){
    #if CONFIG_Filter==0x01
    double A = 0.9361693222916462; // 品质Q=1.186,db=0.1
    double B = 1.4775803817752846; // 切比雪夫滤波器
    lpf->Wc *= A;
    double temp1 = ((double)lpf->T)*((double)lpf->Wc)*B;
    #elif CONFIG_Filter==0x02
    lpf->Wc *= Value_SQRT3;         // 贝塞尔滤波器
    double temp1 = ((double)lpf->T)*((double)lpf->Wc)*((double)Value_INV_SQRT3);
    #else // CONFIG_Filter
    double temp1 = ((double)lpf->T)*((double)lpf->Wc)*((double)Value_2_SQRT2);
    #endif // CONFIG_Filter
    double temp2 = ((double)lpf->T)*((double)lpf->T)*
                    ((double)lpf->Wc)*((double)lpf->Wc);
    lpf->filter.num[0] = (float)(temp2/(temp2+temp1+4.0));
    lpf->filter.num[1] = (float)((2.0*temp2)/(temp2+temp1+4.0));
    lpf->filter.den[0] = (float)(-8.0+2.0*temp2)/(temp2+temp1+4.0);
    lpf->filter.den[1] = (float)(temp2-temp1+4.0)/(temp2+temp1+4.0);

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
 * @reminder: https://github.com/Sguan-ZhouQing/SguanFOC_Library/blob/main/%E9%85%8D%E5%A5%97Simulink%E6%A8%A1%E5%9E%8B%E5%BC%80%E6%BA%90%E2%91%A1%5B%E7%AE%97%E6%B3%95%E5%8E%9F%E7%90%86%E5%9B%BE%5D/Sguan_Filter.png
 * @reminder: (上方链接是此Sguan_Filter模块Simulink原理仿真图)
 * @param {LPF_STRUCT} *lpf
 * @return {*}
 */
void LPF_Loop(LPF_STRUCT *lpf){
    // 1.带入差分方程，计算输出
    lpf->filter.Output = lpf->filter.num[0]*(lpf->filter.Input+lpf->filter.i[1]) + 
                        lpf->filter.num[1]*lpf->filter.i[0] - 
                        lpf->filter.den[0]*lpf->filter.o[0] - 
                        lpf->filter.den[1]*lpf->filter.o[1];

    // 2.更新历史输入和输出数值
    lpf->filter.i[1] = lpf->filter.i[0];
    lpf->filter.i[0] = lpf->filter.Input;
    lpf->filter.o[1] = lpf->filter.o[0];
    lpf->filter.o[0] = lpf->filter.Output;
}


// =============================== BSF陷波滤波器 ============================
/**
 * @description: 陷波滤波器的参数初始化
 * @reminder: (初始化相关系数float->double->float)
 * @reminder: (单浮点转double运算，提高系数精度)
 * @param {LPF_STRUCT} *lpf
 * @return {*}
 */
void BSF_Init(BSF_STRUCT *bsf){
    bsf->filter.Gain0 = (float)(((double)bsf->T)*((double)bsf->zeta)*4.0);
    bsf->filter.Gain1 = (float)(((double)bsf->T)*((double)bsf->T));

    // 初始化为零
    bsf->filter.i[0] = 0.0f;
    bsf->filter.i[1] = 0.0f;
    bsf->filter.o[0] = 0.0f;
    bsf->filter.o[1] = 0.0f;
    bsf->filter.Input = 0.0f;
    bsf->filter.Output = 0.0f;
}

/**
 * @description: 滤波器离散服务函数
 * @reminder: https://github.com/Sguan-ZhouQing/SguanFOC_Library/blob/main/%E9%85%8D%E5%A5%97Simulink%E6%A8%A1%E5%9E%8B%E5%BC%80%E6%BA%90%E2%91%A1%5B%E7%AE%97%E6%B3%95%E5%8E%9F%E7%90%86%E5%9B%BE%5D/Sguan_Filter.png
 * @reminder: (上方链接是此Sguan_Filter模块Simulink原理仿真图)
 * @param {LPF_STRUCT} *lpf
 * @return {*}
 */
void BSF_Loop(BSF_STRUCT *bsf){
    // 1.计算离散传递函数的系数
    float temp1 = bsf->filter.Gain0*bsf->Wo;
    float temp2 = bsf->filter.Gain1*bsf->Wo*bsf->Wo;

    float num_0 = (temp2+4.0f)/(temp2+temp1+4.0f);
    float num_1 = (-8.0f+2.0f*temp2)/(temp2+temp1+4.0f);
    float den_0 = (-8.0f+2.0f*temp2)/(temp2+temp1+4.0f);
    float den_1 = (temp2-temp1+4.0f)/(temp2+temp1+4.0f);

    // 2.带入差分方程，计算输出
    bsf->filter.Output = num_0*(bsf->filter.Input+bsf->filter.i[1]) + 
                        num_1*bsf->filter.i[0] - 
                        den_0*bsf->filter.o[0] - 
                        den_1*bsf->filter.o[1];

    // 3.更新历史输入和输出数值
    bsf->filter.i[1] = bsf->filter.i[0];
    bsf->filter.i[0] = bsf->filter.Input;
    bsf->filter.o[1] = bsf->filter.o[0];
    bsf->filter.o[0] = bsf->filter.Output;
}


// =============================== BPF带通滤波器 ============================
/**
 * @description: 带通滤波器的参数初始化
 * @reminder: (初始化相关系数float->double->float)
 * @reminder: (单浮点转double运算，提高系数精度)
 * @param {LPF_STRUCT} *lpf
 * @return {*}
 */
void BPF_Init(BPF_STRUCT *bpf){
    double temp1 = ((double)bpf->T)*((double)bpf->Wo)*(((double)bpf->zeta)*4.0);
    double temp2 = ((double)bpf->T)*((double)bpf->Wo)*
                    ((double)bpf->T)*((double)bpf->Wo);
    bpf->filter.num = (float)(temp1/(temp2+temp1+4.0));
    bpf->filter.den[0] = (float)(-8.0+2.0*temp2)/(temp2+temp1+4.0);
    bpf->filter.den[1] = (float)(temp2-temp1+4.0)/(temp2+temp1+4.0);

    // 初始化为零
    bpf->filter.i[0] = 0.0f;
    bpf->filter.i[1] = 0.0f;
    bpf->filter.o[0] = 0.0f;
    bpf->filter.o[1] = 0.0f;
    bpf->filter.Input = 0.0f;
    bpf->filter.Output = 0.0f;
}

/**
 * @description: 滤波器离散服务函数
 * @reminder: https://github.com/Sguan-ZhouQing/SguanFOC_Library/blob/main/%E9%85%8D%E5%A5%97Simulink%E6%A8%A1%E5%9E%8B%E5%BC%80%E6%BA%90%E2%91%A1%5B%E7%AE%97%E6%B3%95%E5%8E%9F%E7%90%86%E5%9B%BE%5D/Sguan_Filter.png
 * @reminder: (上方链接是此Sguan_Filter模块Simulink原理仿真图)
 * @param {LPF_STRUCT} *lpf
 * @return {*}
 */
void BPF_Loop(BPF_STRUCT *bpf){
    // 1.带入差分方程，计算输出
    bpf->filter.Output = bpf->filter.num*(bpf->filter.Input - bpf->filter.i[1]) - 
                        bpf->filter.den[0]*bpf->filter.o[0] - 
                        bpf->filter.den[1]*bpf->filter.o[1];

    // 2.更新历史输入和输出数值
    bpf->filter.i[1] = bpf->filter.i[0];
    bpf->filter.i[0] = bpf->filter.Input;
    bpf->filter.o[1] = bpf->filter.o[0];
    bpf->filter.o[0] = bpf->filter.Output;
}


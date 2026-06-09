/*
 * @Author: 星必尘Sguan
 * @GitHub: https://github.com/Sguan-ZhouQing
 * @Date: 2026-01-26 22:37:25
 * @LastEditors: 星必尘Sguan|3464647102@qq.com
 * @LastEditTime: 2026-06-05 03:46:18
 * @FilePath: \SguanFOC_Debug\SguanFOC\Sguan_Filter.c
 * @Description: SguanFOC库的“LPF和TPNF滤波器算法”实现
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
    // 1.宏定义选择需要使用的滤波器(本质依旧是二阶传递函数)
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

    // 2.计算其余传递函数系数
    double temp2 = ((double)lpf->T)*((double)lpf->T)*
                    ((double)lpf->Wc)*((double)lpf->Wc);
    double den = temp2+temp1+4.0;

    lpf->filter.num[0] = (float)(temp2/den);
    lpf->filter.num[1] = (float)((2.0*temp2)/den);
    lpf->filter.den[0] = (float)((-8.0+2.0*temp2)/den);
    lpf->filter.den[1] = (float)((temp2-temp1+4.0)/den);

    // 3.初始化为零
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


// =========================== TPNF三参数陷波滤波器 ==========================
/**
 * @description: 陷波滤波器的参数初始化
 * @reminder: (初始化相关系数float->double->float)
 * @reminder: (单浮点转double运算，提高系数精度)
 * @param {TPNF_STRUCT} *tpnf
 * @return {*}
 */
void TPNF_Init(TPNF_STRUCT *tpnf){
    // 1.计算传递函数增益
    tpnf->filter.Gain0 = (float)(((double)tpnf->T)*((double)tpnf->K1));
    tpnf->filter.Gain1 = (float)(((double)tpnf->T)*((double)tpnf->K2));
    tpnf->filter.Gain2 = (float)(((double)tpnf->T)*((double)tpnf->T));

    // 2.初始化为零
    tpnf->filter.i[0] = 0.0f;
    tpnf->filter.i[1] = 0.0f;
    tpnf->filter.o[0] = 0.0f;
    tpnf->filter.o[1] = 0.0f;
    tpnf->filter.Input = 0.0f;
    tpnf->filter.Output = 0.0f;
}

/**
 * @description: 滤波器离散服务函数
 * @reminder: https://github.com/Sguan-ZhouQing/SguanFOC_Library/blob/main/%E9%85%8D%E5%A5%97Simulink%E6%A8%A1%E5%9E%8B%E5%BC%80%E6%BA%90%E2%91%A1%5B%E7%AE%97%E6%B3%95%E5%8E%9F%E7%90%86%E5%9B%BE%5D/Sguan_Filter.png
 * @reminder: (上方链接是此Sguan_Filter模块Simulink原理仿真图)
 * @param {TPNF_STRUCT} *tpnf
 * @return {*}
 */
void TPNF_Loop(TPNF_STRUCT *tpnf){
    // 1.计算传递函数分子分母系数
    float temp0 = tpnf->filter.Gain0*tpnf->Wo;
    float temp1 = tpnf->filter.Gain1*tpnf->Wo;
    float temp2 = tpnf->filter.Gain2*tpnf->Wo*tpnf->Wo;
    float num = 4.0f+temp2;
    float den = 4.0f+4.0f*temp0+temp2;

    float num_0 = (num+4.0f*temp1)/den;
    float num_1 = (num-8.0f+temp2)/den;
    float num_2 = (num-4.0f*temp1)/den;
    float den_0 = (num-4.0f*temp0)/den;

    // 2.带入差分方程，计算输出
    tpnf->filter.Output = num_0*tpnf->filter.Input + 
                        num_1*(tpnf->filter.i[0]-tpnf->filter.o[0]) + 
                        num_2*tpnf->filter.i[1] - 
                        den_0*tpnf->filter.o[1];

    // 3.更新历史输入和输出数值
    tpnf->filter.i[1] = tpnf->filter.i[0];
    tpnf->filter.i[0] = tpnf->filter.Input;
    tpnf->filter.o[1] = tpnf->filter.o[0];
    tpnf->filter.o[0] = tpnf->filter.Output;
}


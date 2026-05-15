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
    double temp1 = ((double)lpf->T)*((double)lpf->Wc)*((double)Value_2_SQRT2);
    double temp2 = ((double)lpf->T)*((double)lpf->T)*
                    ((double)lpf->Wc)*((double)lpf->Wc);
    lpf->filter.num[0] = (float)(temp2/(temp2+temp1+4.0));
    lpf->filter.num[1] = (float)((2.0*temp2)/(temp2+temp1+4.0));
    lpf->filter.num[2] = (float)(temp2/(temp2+temp1+4.0));
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
    lpf->filter.Output = lpf->filter.num[0] * lpf->filter.Input + 
                        lpf->filter.num[1] * lpf->filter.i[0] + 
                        lpf->filter.num[2] * lpf->filter.i[1] - 
                        lpf->filter.den[0] * lpf->filter.o[0] - 
                        lpf->filter.den[1] * lpf->filter.o[1];

    // 2.更新历史输入和输出数值
    lpf->filter.i[1] = lpf->filter.i[0];
    lpf->filter.i[0] = lpf->filter.Input;
    lpf->filter.o[1] = lpf->filter.o[0];
    lpf->filter.o[0] = lpf->filter.Output;
}



// =============================== LPF低通滤波器 ============================
/**
 * @description: 陷波滤波器的参数初始化
 * @reminder: (初始化相关系数float->double->float)
 * @reminder: (单浮点转double运算，提高系数精度)
 * @param {LPF_STRUCT} *lpf
 * @return {*}
 */
void BSF_Init(BSF_STRUCT *bsf){

}

/**
 * @description: 滤波器离散服务函数
 * @reminder: https://github.com/Sguan-ZhouQing/SguanFOC_Library/blob/main/%E9%85%8D%E5%A5%97Simulink%E6%A8%A1%E5%9E%8B%E5%BC%80%E6%BA%90%E2%91%A1%5B%E7%AE%97%E6%B3%95%E5%8E%9F%E7%90%86%E5%9B%BE%5D/Sguan_Filter.png
 * @reminder: (上方链接是此Sguan_Filter模块Simulink原理仿真图)
 * @param {LPF_STRUCT} *lpf
 * @return {*}
 */
void BSF_Loop(BSF_STRUCT *bsf){

}

/**
 * @description: 带通滤波器的参数初始化
 * @reminder: (初始化相关系数float->double->float)
 * @reminder: (单浮点转double运算，提高系数精度)
 * @param {LPF_STRUCT} *lpf
 * @return {*}
 */
void BPF_Init(BPF_STRUCT *bpf){

}

/**
 * @description: 滤波器离散服务函数
 * @reminder: https://github.com/Sguan-ZhouQing/SguanFOC_Library/blob/main/%E9%85%8D%E5%A5%97Simulink%E6%A8%A1%E5%9E%8B%E5%BC%80%E6%BA%90%E2%91%A1%5B%E7%AE%97%E6%B3%95%E5%8E%9F%E7%90%86%E5%9B%BE%5D/Sguan_Filter.png
 * @reminder: (上方链接是此Sguan_Filter模块Simulink原理仿真图)
 * @param {LPF_STRUCT} *lpf
 * @return {*}
 */
void BPF_Loop(BPF_STRUCT *bpf){

}


/**
 * @description: 全通滤波器的参数初始化
 * @reminder: (初始化相关系数float->double->float)
 * @reminder: (单浮点转double运算，提高系数精度)
 * @param {LPF_STRUCT} *lpf
 * @return {*}
 */
void APF_Init(APF_STRUCT *apf){

}

/**
 * @description: 滤波器离散服务函数
 * @reminder: https://github.com/Sguan-ZhouQing/SguanFOC_Library/blob/main/%E9%85%8D%E5%A5%97Simulink%E6%A8%A1%E5%9E%8B%E5%BC%80%E6%BA%90%E2%91%A1%5B%E7%AE%97%E6%B3%95%E5%8E%9F%E7%90%86%E5%9B%BE%5D/Sguan_Filter.png
 * @reminder: (上方链接是此Sguan_Filter模块Simulink原理仿真图)
 * @param {LPF_STRUCT} *lpf
 * @return {*}
 */
void APF_Loop(APF_STRUCT *apf){

}

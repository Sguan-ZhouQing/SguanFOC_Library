/*
 * @Author: 星必尘Sguan
 * @GitHub: https://github.com/Sguan-ZhouQing
 * @Date: 2026-05-14 22:17:01
 * @LastEditors: 星必尘Sguan|3464647102@qq.com
 * @LastEditTime: 2026-05-14 22:18:29
 * @FilePath: \SguanFOC_Debug\SguanFOC\Sguan_LTD.c
 * @Description: SguanFOC库的“LTD最速控制函数(仿效果)”实现
 * 
 * Copyright (c) 2026 by $星必尘Sguan, All Rights Reserved. 
 */
#include "Sguan_LTD.h"

/**
 * @description: 最速控制LTD函数初始化
 * @reminder: (初始化相关系数float->double->float)
 * @reminder: (单浮点转double运算，提高系数精度)
 * @param {LTD_STRUCT} *ltd
 * @return {*}
 */
void LTD_Init(LTD_STRUCT *ltd){
    double temp1 = ((double)ltd->T)*((double)ltd->r);
    double den = 2.0+temp1;

    ltd->go.num = (float)(temp1/den);
    ltd->go.den = (float)((-2.0+temp1)/den);

    // 初始化为零
    ltd->go.Input = 0.0f;
    ltd->go.Output = 0.0f;
    ltd->go.Last_i = 0.0f;
}

/**
 * @description: 最速控制LTD的运行函数
 * @reminder: https://github.com/Sguan-ZhouQing/SguanFOC_Library/blob/main/%E9%85%8D%E5%A5%97Simulink%E6%A8%A1%E5%9E%8B%E5%BC%80%E6%BA%90%E2%91%A1%5B%E7%AE%97%E6%B3%95%E5%8E%9F%E7%90%86%E5%9B%BE%5D/Sguan_Hall.png
 * @reminder: (上方链接是此Sguan_LTD模块Simulink原理仿真图)
 * @param {LTD_STRUCT} *ltd
 * @return {*}
 */
void LTD_Loop(LTD_STRUCT *ltd){
    // 1.带入差分方程，计算输出
    ltd->go.Output = ltd->go.num*(ltd->go.Input + ltd->go.Last_i) - 
                    ltd->go.den*ltd->go.Output;

    // 2.更新历史输入数值
    ltd->go.Last_i = ltd->go.Input;
}


/*
 * @Author: 星必尘Sguan
 * @GitHub: https://github.com/Sguan-ZhouQing
 * @Date: 2026-04-22 13:27:40
 * @LastEditors: 星必尘Sguan|3464647102@qq.com
 * @LastEditTime: 2026-06-05 03:46:27
 * @FilePath: \SguanFOC_Debug\SguanFOC\Sguan_Hall.c
 * @Description: SguanFOC库的“三霍尔信号处理的函数”实现
 * 
 * Copyright (c) 2026 by $星必尘Sguan, All Rights Reserved. 
 */
#include "Sguan_Hall.h"

/**
 * @description: 霍尔信号数据初始化
 * @reminder: (初始化相关系数float->double->float)
 * @reminder: (单浮点转double运算，提高系数精度)
 * @param {HALL_STRUCT} *hall
 * @return {*}
 */
void Hall_Init(HALL_STRUCT *hall){
    // 在SguanFOC读取偏置后，再激活
    // hall->go.Gain = (float)(((double)hall->T)*((double)hall->Wc));
    // hall->go.Normalized_Gain = (float)(1.0 - 
    //                         (((double)hall->T)*((double)hall->Wc)));
    hall->go.Gain = 1.0f;
    hall->go.Normalized_Gain = 0.0f;

    // 初始化为零
    hall->go.Input_Ga = 0;
    hall->go.Input_Gb = 0;
    hall->go.Input_Gc = 0;
    hall->go.Output_Rad = 0.0f;

    hall->go.Hall_A = 0.0f;
    hall->go.Hall_B = 0.0f;
    hall->go.Hall_C = 0.0f;
}

/**
 * @description: 霍尔信号处理的运行函数
 * @reminder: https://github.com/Sguan-ZhouQing/SguanFOC_Library/blob/main/%E9%85%8D%E5%A5%97Simulink%E6%A8%A1%E5%9E%8B%E5%BC%80%E6%BA%90%E2%91%A1%5B%E7%AE%97%E6%B3%95%E5%8E%9F%E7%90%86%E5%9B%BE%5D/Sguan_Hall.png
 * @reminder: (上方链接是此Sguan_Hall模块Simulink原理仿真图)
 * @param {HALL_STRUCT} *hall
 * @return {*}
 */
void Hall_Loop(HALL_STRUCT *hall){
    // 1.创建函数局部变量
    static uint8_t v[6] = {5, 3, 4, 1, 0, 2};
    uint8_t Signal_a,Signal_b,Signal_c,Sector;

    // 2.三相输入信号的低通滤波
    hall->go.Hall_A = hall->go.Gain*((float)hall->go.Input_Ga) + 
                    hall->go.Normalized_Gain*hall->go.Hall_A;
    hall->go.Hall_B = hall->go.Gain*((float)hall->go.Input_Gb) + 
                    hall->go.Normalized_Gain*hall->go.Hall_B;
    hall->go.Hall_C = hall->go.Gain*((float)hall->go.Input_Gc) + 
                    hall->go.Normalized_Gain*hall->go.Hall_C;
    
    // 3.信号边界判断(防抖动)
    if (hall->go.Hall_A >= hall->Hall_High) Signal_a = 1;
    else if (hall->go.Hall_A <= hall->Hall_Low) Signal_a = 0;
    
    if (hall->go.Hall_B >= hall->Hall_High) Signal_b = 1;
    else if (hall->go.Hall_B <= hall->Hall_Low) Signal_b = 0;

    if (hall->go.Hall_C >= hall->Hall_High) Signal_c = 1;
    else if (hall->go.Hall_C <= hall->Hall_Low) Signal_c = 0;

    // 4.霍尔数据处理并输出角度值
    Sector = (Signal_a << 2) | (Signal_b << 1) | (Signal_c);
    hall->go.Output_Rad = (v[Sector - 1]*(float)Value_2PI)/6.0f;
}


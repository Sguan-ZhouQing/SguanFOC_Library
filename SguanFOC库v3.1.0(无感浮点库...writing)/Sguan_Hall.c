/*
 * @Author: 星必尘Sguan
 * @GitHub: https://github.com/Sguan-ZhouQing
 * @Date: 2026-04-22 13:27:40
 * @LastEditors: 星必尘Sguan|3464647102@qq.com
 * @LastEditTime: 2026-04-22 15:23:44
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
    hall->rad.Gain = (float)(((double)hall->T)*((double)hall->Wc));
    hall->rad.Normalized_Gain = (float)(1.0 - 
                            (((double)hall->T)*((double)hall->Wc)));

    // 初始化为零
    hall->rad.Input_Ga = 0;
    hall->rad.Input_Gb = 0;
    hall->rad.Input_Gc = 0;
    hall->rad.Output_Rad = 0.0f;
    
    hall->rad.Hall_A = 0.0f;
    hall->rad.Hall_B = 0.0f;
    hall->rad.Hall_C = 0.0f;
}

/**
 * @description: 霍尔信号处理的运行函数
 * @reminder: https://github.com/Sguan-ZhouQing/SguanFOC_Library/blob/main/%E6%9C%80%E6%96%B0example%E5%8F%8A%E8%B5%84%E6%96%99%5BSTM32G4%2C%E4%B8%8B%E6%A1%A5%E8%87%82%E5%8F%8C%E7%94%B5%E9%98%BB%5D/%E3%80%90Simulink%E3%80%91Sguan%E5%AD%90%E6%A8%A1%E5%9D%97%E5%8E%9F%E7%90%86%E5%9B%BE/Sguan_Hall.png
 * @reminder: (上方链接是此Sguan_Hall模块Simulink原理仿真图)
 * @param {HALL_STRUCT} *hall
 * @return {*}
 */
void Hall_Loop(HALL_STRUCT *hall){
    // 1.创建函数局部变量
    static uint8_t v[6] = {5, 3, 4, 1, 0, 2};
    uint8_t Signal_a,Signal_b,Signal_c,Sector;

    // 2.三相输入信号的低通滤波
    hall->rad.Hall_A = hall->rad.Gain*((float)hall->rad.Input_Ga) + 
                    hall->rad.Normalized_Gain*hall->rad.Hall_A;
    hall->rad.Hall_B = hall->rad.Gain*((float)hall->rad.Input_Gb) + 
                    hall->rad.Normalized_Gain*hall->rad.Hall_B;
    hall->rad.Hall_C = hall->rad.Gain*((float)hall->rad.Input_Gc) + 
                    hall->rad.Normalized_Gain*hall->rad.Hall_C;
    
    // 3.信号边界判断(防抖动)
    if (hall->rad.Hall_A >= hall->Hall_High) Signal_a = 1;
    else if (hall->rad.Hall_A <= hall->Hall_Low) Signal_a = 0;
    
    if (hall->rad.Hall_B >= hall->Hall_High) Signal_b = 1;
    else if (hall->rad.Hall_B <= hall->Hall_Low) Signal_b = 0;

    if (hall->rad.Hall_C >= hall->Hall_High) Signal_c = 1;
    else if (hall->rad.Hall_C <= hall->Hall_Low) Signal_c = 0;

    // 4.霍尔数据处理并输出角度值
    Sector = (Signal_a << 2) | (Signal_b << 1) | (Signal_c);
    hall->rad.Output_Rad = (v[Sector]*Value_2PI)/6.0f;
}



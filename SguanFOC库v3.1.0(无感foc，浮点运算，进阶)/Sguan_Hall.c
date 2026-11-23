/* ===================================================================
 * 文件:     Sguan_Hall.c
 * 项目:     SguanFOC_v3.1.0 —— 开源 FOC 电机控制框架（无感FOC · 浮点运算 · 进阶）
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


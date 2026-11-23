/* ===================================================================
 * 文件:     Sguan_Cogging.c
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
#include "Sguan_Cogging.h"

/* UserData配置文件声明 */
#include "UserData_Function.h"

// 离线标定的1800点位抗齿槽电流表
static int16_t iq_tab[Cogging_N] = {0};
// static const int16_t iq_tab[Cogging_N] = {0};

/**
 * @description: 抗齿槽离线标定的初始化函数
 * @param {COGGING_STRUCT} *cogging
 * @return {*}
 */
void Cogging_SetIq_Init(COGGING_STRUCT *cogging){
    for (int j = 0; j < (cogging->Count); j++){
        for (int i = 0 ; i < Cogging_N; i++){
            iq_tab[i] += IQmath_Q15_from_float(
                cogging->go.Input_Init_Iq/CONFIG_BASE);
            cogging->go.Output_Init_Rad += 
                (((float)Value_2PI)*i)/((float)Cogging_N);
            User_Delay((uint32_t)(cogging->Cycle/((float)Cogging_N)));
        }
    }
    for (int k = 0; k < Cogging_N; k++){
        iq_tab[k] /= cogging->Count;
    }
}

/**
 * @description: 抗齿槽离线标定的运行函数
 * @reminder: (离线标定法是通过预先测量和记录电机在不同位置的齿槽效应数据)
 * @reminder: (在运行时，根据当前电机位置从查找表中读取对应的补偿电流值)
 * @param {COGGING_STRUCT} *cogging
 * @return {*}
 */
void Cogging_ReadIq_Loop(COGGING_STRUCT *cogging){
    uint16_t K = cogging->go.Input_Loop_Rad*
                ((float)Cogging_N)/((float)Value_2PI);

    cogging->go.Output_Loop_Iq = 
                IQmath_Q15_to_float(iq_tab[K])*CONFIG_BASE;
}

/**
 * @description: 抗齿槽离线标定的运行函数
 * @reminder: (读取表格数值，并返回定点数值)
 * @param {uint16_t} K
 * @return {int16_t}
 */
int16_t Cogging_ReadIq_Tick(uint16_t K){
    return iq_tab[K];
}


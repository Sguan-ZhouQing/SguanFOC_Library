/* ===================================================================
 * 文件:     Sguan_PLL.c
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
#include "Sguan_PLL.h"

/**
 * @description: 锁相环PLL核心参数初始化
 * @reminder: (初始化相关系数float->double->float)
 * @reminder: (单浮点转double运算，提高系数精度)
 * @param {PLL_STRUCT} *pll
 * @return {*}
 */
void PLL_Init(PLL_STRUCT *pll){
    double temp0 = ((double)pll->T)*((double)pll->Ki);
    pll->go.X_num[0] = (float)((2.0*((double)pll->Kp)+temp0)/2.0);
    pll->go.X_num[1] = (float)((-2.0*((double)pll->Kp)+temp0)/2.0);
    pll->go.Y_num = (float)(((double)pll->T)/2.0);

    // 初始化为零
    pll->is_position_mode = 0; // 默认非位置环mode
    pll->go.We_i = 0.0f;
    pll->go.Re_i = 0.0f;
    pll->go.OutWe = 0.0f;
    pll->go.OutRe = 0.0f;
    pll->go.Error = 0.0f;
}

/**
 * @description: 锁相环运算的离散函数
 * @reminder: https://github.com/Sguan-ZhouQing/SguanFOC_Library/blob/main/%E9%85%8D%E5%A5%97Simulink%E6%A8%A1%E5%9E%8B%E5%BC%80%E6%BA%90%E2%91%A1%5B%E7%AE%97%E6%B3%95%E5%8E%9F%E7%90%86%E5%9B%BE%5D/Sguan_PLL.png
 * @reminder: (上方链接是此Sguan_PLL模块Simulink原理仿真图)
 * @param {PLL_STRUCT} *pll
 * @return {*}
 */
void PLL_Loop(PLL_STRUCT *pll){
    // 1.计算PI控制器(并输出We)
    pll->go.OutWe += pll->go.X_num[0]*pll->go.Error + 
                    pll->go.X_num[1]*pll->go.We_i;

    // 2.计算积分器(并输出Re)
    pll->go.OutRe += pll->go.Y_num*(pll->go.OutWe + pll->go.Re_i);
    if (!pll->is_position_mode){
        // 非位置环模式：使用normalize_angle函数归一化到[0, 2π)
        pll->go.OutRe = Value_normalize(pll->go.OutRe);
    }

    // 3.更新历史输入和输出数值
    pll->go.We_i = pll->go.Error;
    pll->go.Re_i = pll->go.OutWe;
}


/* ===================================================================
 * 文件:     Sguan_SMC.c
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
#include "Sguan_SMC.h"

/**
 * @description: 传统滑模控制SMC的初始化函数
 * @reminder: (初始化相关系数float->double->float)
 * @reminder: (单浮点转double运算，提高系数精度)
 * @param {SMC_STRUCT} *smc
 * @return {*}
 */
void SMC_Init(SMC_STRUCT *smc){
    smc->run.I_num = (float)(((double)smc->T)/2.0);

    // 初始化为零
    smc->run.I_i = 0.0f;
    smc->run.D_i = 0.0f;

    smc->run.Ref = 0.0f;
    smc->run.Fbk = 0.0f;
    smc->run.Output = 0.0f;
    
    smc->run.IntegralFrozen_flag = 0;
}

/**
 * @description: 传统滑模控制SMC的离散运行函数
 * @reminder: https://github.com/Sguan-ZhouQing/SguanFOC_Library/blob/main/%E9%85%8D%E5%A5%97Simulink%E6%A8%A1%E5%9E%8B%E5%BC%80%E6%BA%90%E2%91%A1%5B%E7%AE%97%E6%B3%95%E5%8E%9F%E7%90%86%E5%9B%BE%5D/Sguan_SMC.png
 * @reminder: (上方链接是此Sguan_SMC模块Simulink原理仿真图)
 * @param {SMC_STRUCT} *smc
 * @return {*}
 */
void SMC_Loop(SMC_STRUCT *smc){
    // 1.计算误差和滑模微分
    float Error_value,D_value,Value,I_in;
    Error_value = smc->run.Ref - smc->run.Fbk;
    D_value = (Error_value - smc->run.D_i)/smc->T;
    Value = smc->C*Error_value + D_value;

    // 2.计算滑模积分输入量
    I_in = (Value_Sign(Value)*smc->miu + 
                smc->q*Value + 
                smc->C*D_value)*
                smc->Gain;

    // 3.积分限幅
    if (smc->run.IntegralFrozen_flag){
        // 如果积分已冻结，保持上次的积分值
        
        // 检查是否可以解除冻结
        // 情况1：误差反向（误差符号与积分输出符号相反）
        // 情况2：积分值回到限幅范围内
        if ((I_in * smc->run.Output < 0) ||  // 误差反向
            ((smc->run.Output < smc->IntMax) && 
            (smc->run.Output > smc->IntMin))){
            smc->run.IntegralFrozen_flag = 0;
        }
    } else{
        // 正常计算积分
        smc->run.Output += smc->run.I_num*(I_in + smc->run.I_i);
        
        // 检查是否达到限幅，达到则冻结积分
        if (smc->run.Output > smc->IntMax){
            smc->run.Output = smc->IntMax;
            smc->run.IntegralFrozen_flag = 1;
        }
        else if (smc->run.Output < smc->IntMin){
            smc->run.Output = smc->IntMin;
            smc->run.IntegralFrozen_flag = 1;
        }
    }

    // 4.更新历史输入输出值
    smc->run.D_i = Error_value;
    smc->run.I_i = I_in;
}


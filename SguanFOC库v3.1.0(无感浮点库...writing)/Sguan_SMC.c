/*
 * @Author: 星必尘Sguan
 * @GitHub: https://github.com/Sguan-ZhouQing
 * @Date: 2026-04-09 17:16:27
 * @LastEditors: 星必尘Sguan|3464647102@qq.com
 * @LastEditTime: 2026-04-19 01:01:24
 * @FilePath: \SguanFOC_Debug\SguanFOC\Sguan_SMC.c
 * @Description: SguanFOC库的“传统指数型趋近率SMC(转速环)”实现
 * 
 * Copyright (c) 2026 by $星必尘Sguan, All Rights Reserved. 
 */
#include "Sguan_SMC.h"

/**
 * @description: 传统滑模控制SMC的初始化函数
 * @param {SMC_STRUCT} *smc
 * @return {*}
 */
void SMC_Init(SMC_STRUCT *smc){
    smc->run.I_num = smc->T/2.0f;
    // 初始化为零
    smc->run.I_i = 0.0f;
    smc->run.I_o = 0.0f;
    smc->run.D_i = 0.0f;

    smc->run.Ref = 0.0f;
    smc->run.Fbk = 0.0f;
    smc->run.Output = 0.0f;
    
    smc->run.IntegralFrozen_flag = 0;
}

/**
 * @description: 传统滑模控制SMC的离散运行函数
 * @param {SMC_STRUCT} *smc
 * @return {*}
 */
void SMC_Loop(SMC_STRUCT *smc){
    // 1.计算误差和滑模微分
    float Error_value = smc->run.Ref - smc->run.Fbk;
    float D_value = (Error_value - smc->run.D_i)/smc->T;
    float Value = smc->C*Error_value + D_value;

    // 2.计算滑模积分输入量
    float I_in;
    if (Value > 0){
        I_in = (smc->q*Value + smc->miu + smc->C*D_value)*smc->Gain;
    }
    else{
        I_in = (smc->q*Value - smc->miu + smc->C*D_value)*smc->Gain;
    }

    // 3.积分限幅
    if (smc->run.IntegralFrozen_flag){
        // 如果积分已冻结，保持上次的积分值
        smc->run.Output = smc->run.I_o;
        
        // 检查是否可以解除冻结
        // 情况1：误差反向（误差符号与积分输出符号相反）
        // 情况2：积分值回到限幅范围内
        if ((I_in * smc->run.Output < 0) ||  // 误差反向
            (smc->run.Output < smc->IntMax && smc->run.Output > smc->IntMin)){
            smc->run.IntegralFrozen_flag = 0;
        }
    } else{
        // 正常计算积分
        smc->run.Output = smc->run.I_num*(I_in + smc->run.I_i) + 
                            smc->run.I_o;
        
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
    smc->run.I_o = smc->run.Output;
}


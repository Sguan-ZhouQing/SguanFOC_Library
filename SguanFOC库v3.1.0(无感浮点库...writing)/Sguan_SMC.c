/*
 * @Author: 星必尘Sguan
 * @GitHub: https://github.com/Sguan-ZhouQing
 * @Date: 2026-04-09 17:16:27
 * @LastEditors: 星必尘Sguan|3464647102@qq.com
 * @LastEditTime: 2026-04-09 17:22:33
 * @FilePath: \SguanFOC_Debug\SguanFOC\Sguan_SMC.c
 * @Description: SguanFOC库的“传统指数型趋近率SMC(转速环)”实现
 * 
 * Copyright (c) 2026 by $星必尘Sguan, All Rights Reserved. 
 */
#include "Sguan_SMC.h"

// 传统滑模控制SMC的初始化函数
void SMC_Init(SMC_STRUCT *smc){
    double temp0 = smc->T*smc->Wc;
    smc->tradition.I_num = (float)(smc->T/2.0);
    smc->tradition.D_num = (float)((2*smc->Wc)/(-2+temp0));
    smc->tradition.D_den = (float)((2+temp0)/(-2+temp0));
    smc->tradition.Gain = (float)(smc->J/(1.5*smc->Pn*smc->Flux));
    // 初始化为零
    smc->tradition.I_i = 0.0f;
    smc->tradition.I_o = 0.0f;
    smc->tradition.D_i = 0.0f;
    smc->tradition.D_o = 0.0f;

    smc->tradition.Ref = 0.0f;
    smc->tradition.Fbk = 0.0f;
    smc->tradition.Output = 0.0f;
    
    smc->tradition.IntegralFrozen_flag = 0;
}

// 传统滑模控制SMC的离散运行函数
void SMC_Loop(SMC_STRUCT *smc){
    // 计算误差和滑模微分
    float Error_value = smc->tradition.Ref - smc->tradition.Fbk;
    float D_value = smc->tradition.D_num*(Error_value + smc->tradition.D_i) - 
                smc->tradition.D_den*smc->tradition.D_o;
    float Value = smc->C*Error_value + D_value;

    // 计算滑模积分
    float I_in;
    if (Value > 0){
        I_in = (smc->q + smc->miu)*Value + smc->C*D_value;
    }
    else{
        I_in = (smc->q - smc->miu)*Value + smc->C*D_value;
    }
    smc->tradition.Output = smc->tradition.I_num*(I_in + smc->tradition.I_i) + 
                        smc->tradition.I_o;

    // 积分限幅
    if (smc->tradition.IntegralFrozen_flag) {
        // 如果积分已冻结，保持上次的积分值
        smc->tradition.Output = smc->tradition.I_o;
        
        // 检查是否可以解除冻结
        // 情况1：误差反向（误差符号与积分输出符号相反）
        // 情况2：积分值回到限幅范围内
        if ((I_in * smc->tradition.Output < 0) ||  // 误差反向
            (smc->tradition.Output < smc->IntMax && smc->tradition.Output > smc->IntMin)) {  // 回到范围内
            smc->tradition.IntegralFrozen_flag = 0;
        }
    } else {
        // 正常计算积分
        smc->tradition.Output = smc->tradition.I_num*(I_in + smc->tradition.I_i) + 
                            smc->tradition.I_o;
        
        // 检查是否达到限幅，达到则冻结积分
        if (smc->tradition.Output > smc->IntMax) {
            smc->tradition.Output = smc->IntMax;
            smc->tradition.IntegralFrozen_flag = 1;
        }
        else if (smc->tradition.Output < smc->IntMin) {
            smc->tradition.Output = smc->IntMin;
            smc->tradition.IntegralFrozen_flag = 1;
        }
    }

    // 更新历史输入输出值
    smc->tradition.D_i = Error_value;
    smc->tradition.D_o = D_value;
    smc->tradition.I_i = I_in;
    smc->tradition.I_o = smc->tradition.Output;
}



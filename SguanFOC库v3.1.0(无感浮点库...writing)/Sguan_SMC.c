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
 * @reminder: https://github.com/Sguan-ZhouQing/SguanFOC_Library/blob/main/%E6%9C%80%E6%96%B0example%E5%8F%8A%E8%B5%84%E6%96%99%5BSTM32G4%2C%E4%B8%8B%E6%A1%A5%E8%87%82%E5%8F%8C%E7%94%B5%E9%98%BB%5D/%E3%80%90Simulink%E3%80%91Sguan%E5%AD%90%E6%A8%A1%E5%9D%97%E5%8E%9F%E7%90%86%E5%9B%BE/Sguan_SMC.png
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


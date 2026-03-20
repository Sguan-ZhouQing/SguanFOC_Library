/*
 * @Author: 星必尘Sguan
 * @GitHub: https://github.com/Sguan-ZhouQing
 * @Date: 2026-02-26 22:37:25
 * @LastEditors: 星必尘Sguan|3464647102@qq.com
 * @LastEditTime: 2026-03-20 22:58:14
 * @FilePath: \SguanFOC_Debug\SguanFOC\Sguan_STA.c
 * @Description: SguanFOC库的“二阶滑膜控制算法(基于误差和误差微分)”实现
 * 
 * Copyright (c) 2026 by $星必尘Sguan, All Rights Reserved. 
 */
#include "Sguan_STA.h"

// 静态函数声明 - STA辅助函数
static float STA_SignFunction(STA_STRUCT *sta);

/**
 * @description: 二阶滑模控制器(STA)核心参数初始化
 * @param {STA_STRUCT} *sta STA结构体指针
 * @return {void}
 */
void STA_Init(STA_STRUCT *sta){
    // 初始化所有运行变量为零
    sta->run.s = 0.0f;
    sta->run.integral = 0.0f;
    sta->run.output = 0.0f;
    sta->run.ref = 0.0f;
    sta->run.fbk = 0.0f;
    
    // 默认边界层厚度
    sta->boundary = 0.01f;
    
    // 默认积分限幅
    sta->IntMax = 1000.0f;
    sta->IntMin = -1000.0f;
    
    // 默认输出限幅
    sta->OutMax = 2000.0f;
    sta->OutMin = -2000.0f;
}

/**
 * @description: 饱和函数 - 替代符号函数以减少抖振
 * @param {STA_STRUCT} *sta STA结构体指针
 * @return {float} 饱和函数输出值
 */
static float STA_SignFunction(STA_STRUCT *sta){
    float s_abs = fabsf(sta->run.s);
    
    // 边界层内的线性区
    if(s_abs < sta->boundary){
        return sta->run.s / sta->boundary;
    }
    // 边界层外的符号区
    else{
        return (sta->run.s > 0) ? 1.0f : -1.0f;
    }
}

/**
 * @description: STA主循环函数(定时器中断中调用)
 * @param {STA_STRUCT} *sta STA结构体指针
 * @return {void}
 */
void STA_Loop(STA_STRUCT *sta){
    static uint8_t IntegralFrozen_flag = 0;
    
    // 1. 计算滑模面(速度误差)
    sta->run.s = sta->run.ref - sta->run.fbk;
    
    // 2. 获取饱和函数值(替代符号函数)
    float sign_s = STA_SignFunction(sta);
    
    // 3. 计算非线性项 u1 = k1 * |s|^(1/2) * sign(s)
    float nonlinear = sta->k1 * sqrtf(fabsf(sta->run.s)) * sign_s;
    
    // 4. 积分项计算 u2 = ∫ k2 * sign(s) dt
    if(sta->k2 != 0.0f){
        // 判断是否需要冻结积分
        if(IntegralFrozen_flag){
            // 积分已冻结，保持上次值
            // 检查是否可以解除冻结
            // 情况1：滑模面符号与积分项符号相反(系统正在退出饱和)
            // 情况2：积分值回到限幅范围内
            if((sta->run.s * sta->run.integral < 0) ||
               (sta->run.integral < sta->IntMax && sta->run.integral > sta->IntMin)){
                IntegralFrozen_flag = 0;
            }
        }
        
        if(!IntegralFrozen_flag){
            // 正常积分
            sta->run.integral += sta->k2 * sign_s * sta->T;
            
            // 积分限幅检查
            if(sta->run.integral > sta->IntMax){
                sta->run.integral = sta->IntMax;
                IntegralFrozen_flag = 1;
            }
            else if(sta->run.integral < sta->IntMin){
                sta->run.integral = sta->IntMin;
                IntegralFrozen_flag = 1;
            }
        }
    }
    
    // 5. 计算总输出 u = u1 + u2
    sta->run.output = nonlinear + sta->run.integral;
    
    // 6. 输出限幅
    sta->run.output = Value_Limit(sta->run.output, sta->OutMax, sta->OutMin);
}



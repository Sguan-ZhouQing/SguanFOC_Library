/*
 * @Author: 星必尘Sguan
 * @GitHub: https://github.com/Sguan-ZhouQing
 * @Date: 2026-02-26 22:37:25
 * @LastEditors: 星必尘Sguan|3464647102@qq.com
 * @LastEditTime: 2026-04-09 17:24:48
 * @FilePath: \SguanFOC_Debug\SguanFOC\Sguan_STA.c
 * @Description: SguanFOC库的“STA超螺旋，简化版二阶滑模控制算法”实现
 * 
 * Copyright (c) 2026 by $星必尘Sguan, All Rights Reserved. 
 */
#include "Sguan_STA.h"

// 静态函数声明 - STA辅助函数
static float STA_SignFunction(STA_STRUCT *sta,float s_abs);

/**
 * @description: 二阶滑模控制器(STA)核心参数初始化
 * @param {STA_STRUCT} *sta STA结构体指针
 * @return {void}
 */
void STA_Init(STA_STRUCT *sta){
    // 初始化所有运行变量为零
    sta->sta.s = 0.0f;
    sta->sta.integral = 0.0f;
    sta->sta.output = 0.0f;
    sta->sta.Ref = 0.0f;
    sta->sta.Fbk = 0.0f;
    // 积分抗饱和标志位
    sta->sta.IntegralFrozen_flag = 0;
}

/**
 * @description: 饱和函数 - 替代符号函数以减少抖振
 * @param {STA_STRUCT} *sta STA结构体指针
 * @return {float} 饱和函数输出值
 */
static float STA_SignFunction(STA_STRUCT *sta,float s_abs){
    // 边界层内的线性区
    if(s_abs < sta->boundary){
        return sta->sta.s / sta->boundary;
    }
    // 边界层外的符号区
    else{
        return (sta->sta.s > 0) ? 1.0f : -1.0f;
    }
}

/**
 * @description: STA主循环函数(定时器中断中调用)
 * @param {STA_STRUCT} *sta STA结构体指针
 * @return {void}
 */
void STA_Loop(STA_STRUCT *sta){
    // 1. 计算滑模面(速度误差)
    sta->sta.s = sta->sta.Ref - sta->sta.Fbk;
    
    // 2. 获取饱和函数值(替代符号函数)
    float abs = Value_fabsf(sta->sta.s);
    float sign_s = STA_SignFunction(sta,abs);
    
    // 3. 计算非线性项 u1 = k1 * |s|^(1/2) * sign(s)
    float nonlinear = sta->k1 * Value_sqrtf(abs) * sign_s;
    
    // 4. 积分项计算 u2 = ∫ k2 * sign(s) dt
    if(sta->k2 != 0.0f){
        // 判断是否需要冻结积分
        if(sta->sta.IntegralFrozen_flag){
            // 积分已冻结，保持上次值
            // 检查是否可以解除冻结
            // 情况1：滑模面符号与积分项符号相反(系统正在退出饱和)
            // 情况2：积分值回到限幅范围内
            if((sta->sta.s * sta->sta.integral < 0) ||
               (sta->sta.integral < sta->IntMax && sta->sta.integral > sta->IntMin)){
                sta->sta.IntegralFrozen_flag = 0;
            }
        }
        else{
            // 正常积分
            sta->sta.integral += sta->k2 * sign_s * sta->T;
            
            // 积分限幅检查
            if(sta->sta.integral > sta->IntMax){
                sta->sta.integral = sta->IntMax;
                sta->sta.IntegralFrozen_flag = 1;
            }
            else if(sta->sta.integral < sta->IntMin){
                sta->sta.integral = sta->IntMin;
                sta->sta.IntegralFrozen_flag = 1;
            }
        }
    }
    
    // 5. 计算总输出 u = u1 + u2
    sta->sta.output = nonlinear + sta->sta.integral;
    
    // 6. 输出限幅
    sta->sta.output = Value_Limit(sta->sta.output, sta->OutMax, sta->OutMin);
}


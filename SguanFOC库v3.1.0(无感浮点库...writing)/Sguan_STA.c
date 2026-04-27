/*
 * @Author: 星必尘Sguan
 * @GitHub: https://github.com/Sguan-ZhouQing
 * @Date: 2026-02-26 22:37:25
 * @LastEditors: 星必尘Sguan|3464647102@qq.com
 * @LastEditTime: 2026-04-19 01:03:01
 * @FilePath: \SguanFOC_Debug\SguanFOC\Sguan_STA.c
 * @Description: SguanFOC库的“STA超螺旋，简化版二阶滑模控制算法”实现
 * 
 * Copyright (c) 2026 by $星必尘Sguan, All Rights Reserved. 
 */
#include "Sguan_STA.h"

// 静态函数声明 - STA辅助函数
static float STA_SignFunction(STA_STRUCT *sta,float s_abs);

// 饱和函数_替代符号函数以减少抖振
static float STA_SignFunction(STA_STRUCT *sta,float s_abs){
    // 边界层内的线性区
    if(s_abs < sta->boundary){
        return sta->run.s[0] / sta->boundary;
    }
    // 边界层外的符号区
    else{
        return (sta->run.s > 0) ? 1.0f : -1.0f;
    }
}

/**
 * @description: 二阶滑模控制器(STA)核心参数初始化
 * @reminder: (初始化相关系数float->double->float)
 * @reminder: (单浮点转double运算，提高系数精度)
 * @param {STA_STRUCT} *sta
 * @return {void}
 */
void STA_Init(STA_STRUCT *sta){
    // 初始化所有运行变量为零
    sta->run.I_num = (float)((((double)sta->k2)*((double)sta->T))/2.0);

    // 初始化为零
    sta->run.s[0] = 0.0f;
    sta->run.s[1] = 0.0f;
    sta->run.Io = 0.0f;
    sta->run.Output = 0.0f;
    sta->run.Ref = 0.0f;
    sta->run.Fbk = 0.0f;

    // 积分抗饱和标志位
    sta->run.IntegralFrozen_flag = 0;
}

/**
 * @description: STA循环运行函数(定时器中断中调用)
 * @reminder: https://github.com/Sguan-ZhouQing/SguanFOC_Library/blob/main/%E6%9C%80%E6%96%B0example%E5%8F%8A%E8%B5%84%E6%96%99%5BSTM32G4%2C%E4%B8%8B%E6%A1%A5%E8%87%82%E5%8F%8C%E7%94%B5%E9%98%BB%5D/%E3%80%90Simulink%E3%80%91Sguan%E5%AD%90%E6%A8%A1%E5%9D%97%E5%8E%9F%E7%90%86%E5%9B%BE/Sguan_STA.png
 * @reminder: (上方链接是此Sguan_STA模块Simulink原理仿真图)
 * @param {STA_STRUCT} *sta STA结构体指针
 * @return {void}
 */
void STA_Loop(STA_STRUCT *sta){
    // 1.计算滑模面(速度误差)
    float s = sta->run.Ref - sta->run.Fbk;
    
    // 2.获取饱和函数值(替代符号函数)
    float abs = Value_fabsf(s);
    sta->run.s[0] = STA_SignFunction(sta,abs);
    
    // 3.计算非线性项 u1 = k1 * |s|^(1/2) * sign(s)
    float nonlinear = sta->k1 * Value_sqrtf(abs) * sta->run.s[0];
    
    // 4.积分项计算 u2 = ∫ k2 * sign(s) dt
    if(sta->k2){
        // 判断是否需要冻结积分
        if(sta->run.IntegralFrozen_flag){
            // 如果积分已冻结，保持上次的积分值

            // 积分已冻结，保持上次值
            // 检查是否可以解除冻结
            // 情况1：滑模面符号与积分项符号相反(系统正在退出饱和)
            // 情况2：积分值回到限幅范围内
            if((sta->run.s[0] * sta->run.Io < 0) ||
               ((sta->run.Io < sta->IntMax) && 
               (sta->run.Io > sta->IntMin))){
                sta->run.IntegralFrozen_flag = 0;
            }
        }
        else{
            // 正常计算积分
            sta->run.Io = sta->run.I_num*(sta->run.s[0] + sta->run.s[1]) + sta->run.Io;
            
            // 积分限幅检查
            if(sta->run.Io > sta->IntMax){
                sta->run.Io = sta->IntMax;
                sta->run.IntegralFrozen_flag = 1;
            }
            else if(sta->run.Io < sta->IntMin){
                sta->run.Io = sta->IntMin;
                sta->run.IntegralFrozen_flag = 1;
            }
        }
    }
    
    // 5.计算总输出并限幅
    sta->run.Output = nonlinear + sta->run.Io;
    sta->run.Output = Value_Limit(sta->run.Output, 
                                sta->OutMax, 
                                sta->OutMin);

    // 6.刷新历史输入和输出数值
    sta->run.s[1] = sta->run.s[0];
}


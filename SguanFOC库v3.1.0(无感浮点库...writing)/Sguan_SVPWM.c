/*
 * @Author: 星必尘Sguan
 * @GitHub: https://github.com/Sguan-ZhouQing
 * @Date: 2026-03-20 10:21:01
 * @LastEditors: 星必尘Sguan|3464647102@qq.com
 * @LastEditTime: 2026-04-19 01:03:14
 * @FilePath: \SguanFOC_Debug\SguanFOC\Sguan_SVPWM.c
 * @Description: SguanFOC库的“七段式SVPWM空间矢量合成”实现
 * 
 * Copyright (c) 2026 by $星必尘Sguan, All Rights Reserved. 
 */
#include "Sguan_SVPWM.h"

/**
 * SVPWM 函数 - 七段式空间矢量合成SVPWM
 * @param d: D轴电压，归一化范围 0-1
 * @param q: Q轴电压，归一化范围 0-1
 * @param sin_phi: 电角度的正弦值
 * @param cos_phi: 电角度的余弦值
 * @param d_u: 输出U相占空比，归一化到0-1
 * @param d_v: 输出V相占空比，归一化到0-1
 * @param d_w: 输出W相占空比，归一化到0-1
 */
void SVPWM(float u_alpha, float u_beta, 
        float *d_u, float *d_v, float *d_w){
    // 1.SVPWM实现开始
    const float ts = 1.0f;  // 周期归一化
    
    float u1 = u_beta;
    float u2 = -Value_SQRT3_2 * u_alpha - 0.5f * u_beta;
    float u3 = Value_SQRT3_2 * u_alpha - 0.5f * u_beta;
    
    // 2.扇区数值计算
    uint8_t sector = (u1 > 0.0f) + ((u2 > 0.0f) << 1) + ((u3 > 0.0f) << 2);
    
    float t_a, t_b, t_c;
    float k_svpwm;
    
    // 3.PWM占用时间幅值
    if (sector == 5) {
        float t4 = u3;
        float t6 = u1;
        float sum = t4 + t6;
        if (sum > ts) {
            k_svpwm = ts / sum;
            t4 = k_svpwm * t4;
            t6 = k_svpwm * t6;
        }
        float t7 = (ts - t4 - t6) / 2.0f;
        t_a = t4 + t6 + t7;
        t_b = t6 + t7;
        t_c = t7;
    } else if (sector == 1) {
        float t2 = -u3;
        float t6 = -u2;
        float sum = t2 + t6;
        if (sum > ts) {
            k_svpwm = ts / sum;
            t2 = k_svpwm * t2;
            t6 = k_svpwm * t6;
        }
        float t7 = (ts - t2 - t6) / 2.0f;
        t_a = t6 + t7;
        t_b = t2 + t6 + t7;
        t_c = t7;
    } else if (sector == 3) {
        float t2 = u1;
        float t3 = u2;
        float sum = t2 + t3;
        if (sum > ts) {
            k_svpwm = ts / sum;
            t2 = k_svpwm * t2;
            t3 = k_svpwm * t3;
        }
        float t7 = (ts - t2 - t3) / 2.0f;
        t_a = t7;
        t_b = t2 + t3 + t7;
        t_c = t3 + t7;
    } else if (sector == 2) {
        float t1 = -u1;
        float t3 = -u3;
        float sum = t1 + t3;
        if (sum > ts) {
            k_svpwm = ts / sum;
            t1 = k_svpwm * t1;
            t3 = k_svpwm * t3;
        }
        float t7 = (ts - t1 - t3) / 2.0f;
        t_a = t7;
        t_b = t3 + t7;
        t_c = t1 + t3 + t7;
    } else if (sector == 6) {
        float t1 = u2;
        float t5 = u3;
        float sum = t1 + t5;
        if (sum > ts) {
            k_svpwm = ts / sum;
            t1 = k_svpwm * t1;
            t5 = k_svpwm * t5;
        }
        float t7 = (ts - t1 - t5) / 2.0f;
        t_a = t5 + t7;
        t_b = t7;
        t_c = t1 + t5 + t7;
    } else if (sector == 4) {
        float t4 = -u2;
        float t5 = -u1;
        float sum = t4 + t5;
        if (sum > ts) {
            k_svpwm = ts / sum;
            t4 = k_svpwm * t4;
            t5 = k_svpwm * t5;
        }
        float t7 = (ts - t4 - t5) / 2.0f;
        t_a = t4 + t5 + t7;
        t_b = t7;
        t_c = t5 + t7;
    } else {
        t_a = 0.5f;
        t_b = 0.5f;
        t_c = 0.5f;
    }
    
    // 4.这里输出三相归一化后的电压信号
    // 暂不输出限幅,SguanFOC中会统一限幅
    *d_u = t_a;
    *d_v = t_b;
    *d_w = t_c;
}



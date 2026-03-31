/*
 * @Author: 星必尘Sguan
 * @GitHub: https://github.com/Sguan-ZhouQing
 * @Date: 2026-03-20 10:21:01
 * @LastEditors: 星必尘Sguan|3464647102@qq.com
 * @LastEditTime: 2026-03-20 22:58:19
 * @FilePath: \SguanFOC_Debug\SguanFOC\Sguan_SVPWM.c
 * @Description: SguanFOC库的“七段式SVPWM空间矢量合成”实现
 * 
 * Copyright (c) 2026 by $星必尘Sguan, All Rights Reserved. 
 */
#include "Sguan_SVPWM.h"

// 常量宏定义声明
#define Value_INV_SQRT3 0.5773502691896257f

/**
 * SVPWM 函数 - C语言版本
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
    // 1. SVPWM实现
    const float ts = 1.0f;  // 周期归一化
    
    float u1 = u_beta;
    float u2 = -0.8660254037844386f * u_alpha - 0.5f * u_beta;
    float u3 = 0.8660254037844386f * u_alpha - 0.5f * u_beta;
    
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
        // 零矢量（sector == 0 或无效扇区）
        t_a = 0.5f;
        t_b = 0.5f;
        t_c = 0.5f;
    }
    
    // 4. 将输出归一化到0-1范围
    // 原始t_a, t_b, t_c范围是0-1，但需要确保在0-1范围内
    *d_u = Value_Limit(t_a, 1.0f, 0.0f);
    *d_v = Value_Limit(t_b, 1.0f, 0.0f);
    *d_w = Value_Limit(t_c, 1.0f, 0.0f);
}

// 克拉克变换
void clarke(float *i_alpha,float *i_beta,float i_a,float i_b) {
  *i_alpha = i_a;
  *i_beta = (i_a + 2 * i_b) * Value_INV_SQRT3;
}

// 帕克变换
void park(float *i_d,float *i_q,float i_alpha,float i_beta,float sine,float cosine) {
  *i_d = i_alpha * cosine + i_beta * sine;
  *i_q = i_beta * cosine - i_alpha * sine;
}

// 帕克逆变换
void ipark(float *u_alpha,float *u_beta,float u_d,float u_q,float sine,float cosine) {
  *u_alpha = u_d * cosine - u_q * sine;
  *u_beta = u_q * cosine + u_d * sine;
}


// ============================ Q31 版本代码 ============================
#define Value_INV_SQRT3_q31 1239850240

void SVPWM_q31(Q31_t u_alpha, Q31_t u_beta, 
        Q31_t *d_u, Q31_t *d_v, Q31_t *d_w){
    const Q31_t ts = Q31_MAX;
    
    Q31_t u1 = u_beta;
    Q31_t u2 = IQmath_Q31_mul(-1859775360,u_alpha) - IQmath_Q31_mul(Q31_HALF,u_beta);
    Q31_t u3 = IQmath_Q31_mul(1859775360,u_alpha) - IQmath_Q31_mul(Q31_HALF,u_beta);
    
    uint8_t sector = (u1 > 0) + ((u2 > 0) << 1) + ((u3 > 0) << 2);
    
    Q31_t t_a, t_b, t_c;
    Q31_t k_svpwm;
    
    if (sector == 5) {
        Q31_t t4 = u3;
        Q31_t t6 = u1;
        Q31_t sum = t4 + t6;
        if (sum > ts) {
            k_svpwm = IQmath_Q31_div(ts,sum);
            t4 = IQmath_Q31_mul(k_svpwm,t4);
            t6 = IQmath_Q31_mul(k_svpwm,t6);
        }
        Q31_t t7 = (ts - t4 - t6) / 2;
        t_a = t4 + t6 + t7;
        t_b = t6 + t7;
        t_c = t7;
    } else if (sector == 1) {
        Q31_t t2 = -u3;
        Q31_t t6 = -u2;
        Q31_t sum = t2 + t6;
        if (sum > ts) {
            k_svpwm = IQmath_Q31_div(ts,sum);
            t2 = IQmath_Q31_mul(k_svpwm,t2);
            t6 = IQmath_Q31_mul(k_svpwm,t6);
        }
        Q31_t t7 = (ts - t2 - t6) / 2;
        t_a = t6 + t7;
        t_b = t2 + t6 + t7;
        t_c = t7;
    } else if (sector == 3) {
        Q31_t t2 = u1;
        Q31_t t3 = u2;
        Q31_t sum = t2 + t3;
        if (sum > ts) {
            k_svpwm = IQmath_Q31_div(ts,sum);
            t2 = IQmath_Q31_mul(k_svpwm,t2);
            t3 = IQmath_Q31_mul(k_svpwm,t3);
        }
        Q31_t t7 = (ts - t2 - t3) / 2;
        t_a = t7;
        t_b = t2 + t3 + t7;
        t_c = t3 + t7;
    } else if (sector == 2) {
        Q31_t t1 = -u1;
        Q31_t t3 = -u3;
        Q31_t sum = t1 + t3;
        if (sum > ts) {
            k_svpwm = IQmath_Q31_div(ts,sum);
            t1 = IQmath_Q31_mul(k_svpwm,t1);
            t3 = IQmath_Q31_mul(k_svpwm,t3);
        }
        Q31_t t7 = (ts - t1 - t3) / 2;
        t_a = t7;
        t_b = t3 + t7;
        t_c = t1 + t3 + t7;
    } else if (sector == 6) {
        Q31_t t1 = u2;
        Q31_t t5 = u3;
        Q31_t sum = t1 + t5;
        if (sum > ts) {
            k_svpwm = IQmath_Q31_div(ts,sum);
            t1 = IQmath_Q31_mul(k_svpwm,t1);
            t5 = IQmath_Q31_mul(k_svpwm,t5);
        }
        Q31_t t7 = (ts - t1 - t5) / 2;
        t_a = t5 + t7;
        t_b = t7;
        t_c = t1 + t5 + t7;
    } else if (sector == 4) {
        Q31_t t4 = -u2;
        Q31_t t5 = -u1;
        Q31_t sum = t4 + t5;
        if (sum > ts) {
            k_svpwm = IQmath_Q31_div(ts,sum);
            t4 = IQmath_Q31_mul(k_svpwm,t4);
            t5 = IQmath_Q31_mul(k_svpwm,t5);
        }
        Q31_t t7 = (ts - t4 - t5) / 2;
        t_a = t4 + t5 + t7;
        t_b = t7;
        t_c = t5 + t7;
    } else {
        t_a = Q31_HALF;
        t_b = Q31_HALF;
        t_c = Q31_HALF;
    }
    *d_u = t_a;
    *d_v = t_b;
    *d_w = t_c;
}

void clarke_q31(Q31_t *i_alpha,Q31_t *i_beta,Q31_t i_a,Q31_t i_b) {
  *i_alpha = i_a;
  *i_beta = IQmath_Q31_mul((i_a + 2 * i_b), Value_INV_SQRT3_q31);
}

void park_q31(Q31_t *i_d,Q31_t *i_q,Q31_t i_alpha,Q31_t i_beta,Q31_t sine,Q31_t cosine) {
  *i_d = IQmath_Q31_mul(i_alpha,cosine) + IQmath_Q31_mul(i_beta,sine);
  *i_q = IQmath_Q31_mul(i_beta,cosine) - IQmath_Q31_mul(i_alpha,sine);
}

void ipark_q31(Q31_t *u_alpha,Q31_t *u_beta,Q31_t u_d,Q31_t u_q,Q31_t sine,Q31_t cosine) {
  *u_alpha = IQmath_Q31_mul(u_d,cosine) - IQmath_Q31_mul(u_q,sine);
  *u_beta = IQmath_Q31_mul(u_q,cosine) + IQmath_Q31_mul(u_d,sine);
}


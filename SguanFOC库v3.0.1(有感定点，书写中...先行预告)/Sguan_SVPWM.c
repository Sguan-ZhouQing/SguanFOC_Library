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

#define Value_INV_SQRT3 0.5773502691896257f
static void Overmod(float *d, float *q);

// 对DQ轴电压进行幅值限制，防止过调制
static void Overmod(float *d, float *q){
  // 计算合成“矢量幅值的平方”
  float Vref = (*d)*(*d) + (*q)*(*q);
  
  if (Vref > 1.0f) {
    float scale = 1.0f / Value_sqrtf(Vref);
    *d *= scale;
    *q *= scale;
    // 幅值限制处理,如果“幅值平方”超过 1,进行等比例缩放
  }
}

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
void SVPWM(float d, float q, float sin_phi, float cos_phi, 
          float *d_u, float *d_v, float *d_w){
    // 1. 幅值限制，防止过调制
    float d_limited = d;
    float q_limited = q;
    Overmod(&d_limited, &q_limited);
    
    // 2. 帕克逆变换 (IPARK)
    float u_alpha = d_limited * cos_phi - q_limited * sin_phi;
    float u_beta = q_limited * cos_phi + d_limited * sin_phi;
    
    // 3. SVPWM实现
    const float ts = 1.0f;  // 周期归一化
    
    float u1 = u_beta;
    float u2 = -0.8660254037844386f * u_alpha - 0.5f * u_beta;
    float u3 = 0.8660254037844386f * u_alpha - 0.5f * u_beta;
    
    uint8_t sector = (u1 > 0.0f) + ((u2 > 0.0f) << 1) + ((u3 > 0.0f) << 2);
    
    float t_a, t_b, t_c;
    float k_svpwm;
    
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

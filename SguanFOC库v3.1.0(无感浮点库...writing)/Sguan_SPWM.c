/*
 * @Author: 星必尘Sguan
 * @GitHub: https://github.com/Sguan-ZhouQing
 * @Date: 2026-04-19 00:55:58
 * @LastEditors: 星必尘Sguan|3464647102@qq.com
 * @LastEditTime: 2026-04-21 10:00:00
 * @FilePath: \SguanFOC_Debug\SguanFOC\Sguan_SPWM.c
 * @Description: SguanFOC库的“SPWM三次谐波(零序分量注入，等效SVPWM)”实现
 * 
 * Copyright (c) 2026 by $星必尘Sguan, All Rights Reserved. 
 */
#include "Sguan_SPWM.h"

/**
 * SPWM 函数 - 带三次“零序分量注入”优化的SPWM（等效SVPWM）
 * @param u_alpha: Alpha轴电压，归一化范围 -0.577~0.577（最大线性调制区）
 * @param u_beta: Beta轴电压，归一化范围 -0.577~0.577
 * @param d_u: 输出U相占空比，归一化到0-1
 * @param d_v: 输出V相占空比，归一化到0-1
 * @param d_w: 输出W相占空比，归一化到0-1
 * 
 * 说明：SPWM + 三次谐波注入等效于SVPWM，需要将u_alpha/u_beta乘以系数1/sqrt(3)
 *       才能达到和SVPWM相同的电压利用率
 */
void SPWM(float u_alpha, float u_beta, 
          float *d_u, float *d_v, float *d_w) {
    
    // 重要：SPWM+三次谐波注入需要幅值缩放才能等效SVPWM
    // SVPWM的最大线性调制比为 2/sqrt(3) ≈ 1.1547 倍于传统SPWM
    // 因此需要将输入电压乘以 1/sqrt(3) ≈ 0.57735 进行补偿
    const float INV_SQRT3 = 0.5773502691896257f;
    float u_alpha_scaled = u_alpha * INV_SQRT3;
    float u_beta_scaled = u_beta * INV_SQRT3;
    
    // 1. 等幅值逆变换 → 三相电压(范围 -0.577 到 0.577)
    float u_a = u_alpha_scaled;
    float u_b = -0.5f * u_alpha_scaled + Value_SQRT3_2 * u_beta_scaled;
    float u_c = -0.5f * u_alpha_scaled - Value_SQRT3_2 * u_beta_scaled;
    
    // 2. 注入零序分量(三次谐波)，等效 SVPWM
    float u_max = Value_maxf(u_a, Value_maxf(u_b, u_c));
    float u_min = Value_minf(u_a, Value_minf(u_b, u_c));
    float u_zero = -(u_max + u_min) * 0.5f;  // 零序分量
    
    // 3. 加零序，马鞍波范围再偏移到 [0, 1] 区间
    //    经过零序注入后，电压范围从 [-0.577,0.577] 扩展到 [-0.5,0.5]
    float v_a = u_a + u_zero + 0.5f;
    float v_b = u_b + u_zero + 0.5f;
    float v_c = u_c + u_zero + 0.5f;
    
    // 4. 限幅输出
    *d_u = Value_Limit(v_a, 1.0f, 0.0f);
    *d_v = Value_Limit(v_b, 1.0f, 0.0f);
    *d_w = Value_Limit(v_c, 1.0f, 0.0f);
}


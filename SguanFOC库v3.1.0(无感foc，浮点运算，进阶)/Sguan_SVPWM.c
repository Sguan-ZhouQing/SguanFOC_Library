/* ===================================================================
 * 文件:     Sguan_SVPWM.c
 * 项目:     SguanFOC_v3.1.0 —— 开源 FOC 电机控制框架（无感FOC · 浮点运算 · 进阶）
 * 链接:     https://github.com/Sguan-ZhouQing/SguanFOC_Library
 * 版权:     Copyright 2026 星必尘Sguan <3464647102@qq.com>
 * 许可证:   MIT License
 * SPDX:     SPDX-License-Identifier: MIT
 *
 * 许可说明:
 *   [EN] Permission is hereby granted, free of charge, to use, copy,
 *        modify, merge, publish, distribute, sublicense, and/or sell
 *        copies of this software, provided this notice is retained.
 *   [中文] 本软件依据 MIT 许可证授权：可免费使用、复制、修改、合并、
 *        出版、分发、再授权及销售，仅须保留本版权与许可声明。
 *   [DE] Hiermit wird jeder Person kostenlos erlaubt, diese Software zu
 *        nutzen, zu kopieren, zu veraendern, zu veroeffentlichen, zu
 *        vertreiben, zu unterlizenzieren und/oder zu verkaufen, sofern
 *        dieser Hinweis erhalten bleibt.
 *   [JA] 本ソフトウェアは MIT ライセンスに基づき、無償で使用・複製・
 *        改変・公開・配布・再許諾・販売が許可されます。ただし本表示を
 *        保持する必要があります。
 * 完整许可文本: 参见仓库根目录下的 LICENSE-MIT 文件。
 * =================================================================== */
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



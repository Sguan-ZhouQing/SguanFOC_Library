/* ===================================================================
 * 文件:     Sguan_SPWM.c
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
#include "Sguan_SPWM.h"

/**
 * SPWM 函数 - 带三次“零序分量注入”优化的SPWM（等效SVPWM）
 * @param u_alpha: Alpha轴电压，归一化范围 -0.577~0.577(最大线性调制区)
 * @param u_beta: Beta轴电压，归一化范围 -0.577~0.577(最大线性调制区)
 * @param d_u: 输出U相占空比，归一化到0-1
 * @param d_v: 输出V相占空比，归一化到0-1
 * @param d_w: 输出W相占空比，归一化到0-1
 */
void SPWM(float u_alpha, float u_beta, 
          float *d_u, float *d_v, float *d_w) {
    
    // SPWM+三次谐波注入需要幅值缩放才能等效SVPWM
    // 1.等幅值逆变换 → 三相电压(范围 -0.577 到 0.577)
    float u_a = u_alpha * Value_INV_SQRT3;
    float u_b = (-0.5f*u_alpha + Value_SQRT3_2*u_beta)*Value_INV_SQRT3;
    float u_c = (-0.5f*u_alpha - Value_SQRT3_2*u_beta)*Value_INV_SQRT3;
    
    // 2.注入零序分量(三次谐波)，等效 SVPWM
    float u_max = Value_maxf(u_a, Value_maxf(u_b, u_c));
    float u_min = Value_minf(u_a, Value_minf(u_b, u_c));
    float u_zero = -(u_max + u_min) * 0.5f;  // 零序分量
    
    // 3.加零序，马鞍波范围再偏移到 [0, 1] 区间
    //   经过零序注入后，电压范围从 [-0.577,0.577] 扩展到 [-0.5,0.5]
    float v_a = u_a + u_zero + 0.5f;
    float v_b = u_b + u_zero + 0.5f;
    float v_c = u_c + u_zero + 0.5f;
    
    // 4.这里输出三相归一化后的电压信号
    // 暂不输出限幅,SguanFOC中会统一限幅
    *d_u = v_a;
    *d_v = v_b;
    *d_w = v_c;
}


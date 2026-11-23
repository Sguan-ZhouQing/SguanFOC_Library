/* ===================================================================
 * 文件:     Sguan_SVPWM.h
 * 项目:     SguanFOC_v3.1.1 —— 开源 FOC 电机控制框架（无感FOC · Q31定点 · 进阶）
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
#ifndef __SGUAN_SVPWM_H
#define __SGUAN_SVPWM_H

/* SguanFOC配置文件声明 */
#include "Sguan_Config.h"

// 全局变量声明
extern uint8_t sector_SVPWM;

// 电机相关
void SVPWM(Q31_t u_alpha, Q31_t u_beta, 
        Q31_t *d_u, Q31_t *d_v, Q31_t *d_w);
void clarke(Q31_t *i_alpha,Q31_t *i_beta,Q31_t i_a,Q31_t i_b);
void park(Q31_t *i_d,Q31_t *i_q,Q31_t i_alpha,Q31_t i_beta,Q31_t sine,Q31_t cosine);
void ipark(Q31_t *u_alpha,Q31_t *u_beta,Q31_t u_d,Q31_t u_q,Q31_t sine,Q31_t cosine);


#endif // SGUAN_SVPWM_H

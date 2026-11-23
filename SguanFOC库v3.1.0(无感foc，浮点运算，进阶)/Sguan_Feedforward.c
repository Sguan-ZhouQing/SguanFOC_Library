/* ===================================================================
 * 文件:     Sguan_Feedforward.c
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
#include "Sguan_Feedforward.h"

/**
 * @description: D轴电流前馈量
 * @param {float} We 电角速度
 * @param {float} Lq Q轴电感
 * @param {float} Iq Q轴电流
 * @return {float}
 */
float Feedforward_CurrentD(float We,float Lq,float Iq){
    return -(We*Lq*Iq);
}

/**
 * @description: Q轴电流前馈量
 * @param {float} We 电角速度
 * @param {float} Ld D轴电感
 * @param {float} Id D轴电流
 * @param {float} Flux 磁链
 * @return {float}
 */
float Feedforward_CurrentQ(float We,float Ld,float Id,float Flux){
    return We*(Ld*Id + Flux);
}

/**
 * @description: 转速环速度前馈量
 * @param {float} Speed 机械角速度
 * @param {float} Ba 电机转速环带宽
 * @return {float}
 */
float Feedforward_Velocity(float Speed,float Ba){
    return -Speed*Ba;
}

/**
 * @description: 扰动观测器前馈量
 * @param {float} Fd 观测的扰动力矩
 * @param {float} Pn 电机极对数
 * @param {float} Flux 磁链
 * @return {float}
 */
float Feedforward_DOB(float Fd,float Pn,float Flux){
    return Fd/(1.5f*Pn*Flux);
}


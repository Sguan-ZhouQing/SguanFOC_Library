/* ===================================================================
 * 文件:     Sguan_printf.h
 * 项目:     SguanFOC_v3.0.1 —— 开源 FOC 电机控制框架（有感FOC · Q31定点 · 初阶）
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
#ifndef __SGUAN_PRINTF_H
#define __SGUAN_PRINTF_H

/* SguanFOC配置文件声明 */
#include "Sguan_Config.h"

/**
 * @description: 自定义数据发送的个数(默认为8)
 * @return {*}
 */
#define CH_COUNT 12 // UART或者CAN发送的数据个数

typedef struct{
    float fdata[CH_COUNT];
    uint8_t tail[4];
}PRINTF_STRUCT;

extern uint8_t Sguan_PrintfBuff[200];

// 函数定义声明
void Printf_Init(PRINTF_STRUCT *str);
void Printf_Loop(PRINTF_STRUCT *str);
void Printf_Adjust(void);


#endif // SGUAN_PRINTF_H

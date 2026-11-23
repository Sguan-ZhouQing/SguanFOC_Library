/* ===================================================================
 * 文件:     Sguan_Printf.h
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
#ifndef __SGUAN_PRINTF_H
#define __SGUAN_PRINTF_H

#include "Sguan_Config.h"
#include <stdint.h>

#define CH_COUNT      8    // JustFloat 通道数
#define CMD_MAX_LEN   20    // 最大支持的指令长度
#define HASH_SIZE     32    // 哈希表大小
typedef void (*CmdCallback)(float value);

typedef struct {
    const char* name;       // 指令名称
    uint32_t hash_val;      // 哈希值 (运行时计算并缓存)
    CmdCallback callback;   // 对应的处理函数
}CMD_MAP_STRUCT;

typedef struct {
    uint8_t tail[4];
    float fdata[CH_COUNT];
}PRINTF_STRUCT;

extern uint8_t Sguan_PrintfBuff[64];

void Printf_TX_Init(PRINTF_STRUCT *str);
void Printf_TX_Loop(PRINTF_STRUCT *str);
void Printf_RX_Init(void);
void Printf_RX_Loop(uint8_t *data, uint16_t length);


#endif // SGUAN_PRINTF_H

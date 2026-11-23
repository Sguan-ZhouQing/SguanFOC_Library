/* ===================================================================
 * 文件:     Sguan_Printf.h
 * 项目:     SguanFOC_v4 —— 开源 FOC 电机控制框架
 * 说明:     1.Sguan固件库_串口调试
 * 链接:     https://github.com/Sguan-ZhouQing/SguanFOC_Library
 * 版权:     Copyright 2026 星必尘Sguan <3464647102@qq.com>
 * 许可证:   Apache License, Version 2.0
 * SPDX:     SPDX-License-Identifier: Apache-2.0
 *
 * 许可说明:
 *   [EN] Licensed under the Apache License, Version 2.0. You may use,
 *        copy, modify, and distribute this software, including for
 *        commercial purposes, provided you retain this copyright
 *        notice and the terms of the License.
 *   [中文] 本软件依据 Apache License 2.0 授权：可自由使用、复制、修改、
 *        分发（含商业用途），但必须保留本版权声明与许可证条款。
 *   [DE] Lizenziert unter der Apache License, Version 2.0. Nutzung,
 *        Vervielfaeltigung, Aenderung und Weitergabe (auch kommerziell)
 *        sind erlaubt, sofern dieser Urheberrechtshinweis und die
 *        Lizenzbedingungen erhalten bleiben.
 *   [JA] Apache License, Version 2.0 に基づき許諾されます。使用・複製・
 *        改変・再配布（商用を含む）が可能ですが、本著作権表示と
 *        ライセンス条項を保持する必要があります。
 * 完整许可文本: 参见仓库根目录下的 LICENSE-APACHE 文件。
 * =================================================================== */
#ifndef __SGUAN_PRINTF_H
#define __SGUAN_PRINTF_H

/* SguanFOC配置文件声明 */
#include "Sguan_Config.h"

#define LOCAL_CH_COUNT      10  // JustFloat 通道数
typedef void (*Cmd_Callback)(float value);

typedef struct{
    const char* name;           // 指令名称
    uint32_t hash_val;          // 哈希值 (运行时计算并缓存)
    Cmd_Callback callback;      // 对应的处理函数
}__LocalMap;

typedef struct{
    uint8_t tail[4];
    float fdata[LOCAL_CH_COUNT];
}Printf;

CPLUSPLUS_C_BEGIN // C和C++配置文件声明_BEGIN
extern uint8_t sguan_printfbuff[64];

void printf_tx_init(Printf *str);
void printf_tx_loop(Printf *str);
void printf_rx_init(void);
void printf_rx_loop(uint8_t *data, uint16_t length);
CPLUSPLUS_C_END // C和C++配置文件声明_END


#endif // SGUAN_PRINTF_H

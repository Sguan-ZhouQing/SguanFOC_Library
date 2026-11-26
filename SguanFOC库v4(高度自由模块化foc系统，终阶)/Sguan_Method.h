/* ===================================================================
 * 文件:     Sguan_Method.h
 * 项目:     SguanFOC_v4 —— 开源 FOC 电机控制框架
 * 说明:     1.Sguan固件库_方式方法
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
#ifndef __SGUAN_METHOD
#define __SGUAN_METHOD

/* SguanFOC配置文件声明 */
#include "Sguan_Config.h"

typedef struct{
    #if CONFIG_SGUANQ // 扩展SguanQ浮点数
    SguanQ             sguanq_ch[CONFIG_SGUANQ];
    #endif // CONFIG_SGUANQ

    #if CONFIG_INT8 // 扩展8位有符号数
    int8_t              int8_ch[CONFIG_INT8];
    #endif // CONFIG_INT8

    #if CONFIG_UINT8 // 扩展8位无符号数
    uint8_t             uint8_ch[CONFIG_UINT8];
    #endif // CONFIG_UINT8

    #if CONFIG_INT16 // 扩展16位有符号数
    int16_t             int16_ch[CONFIG_INT16];
    #endif // CONFIG_INT16

    #if CONFIG_UINT16 // 扩展16位无符号数
    uint16_t            uint16_ch[CONFIG_UINT16];
    #endif // CONFIG_UINT16

    #if CONFIG_INT32 // 扩展32位有符号数
    int32_t             int32_ch[CONFIG_INT32];
    #endif // CONFIG_INT32

    #if CONFIG_UINT32 // 扩展32位无符号数
    uint32_t            uint32_ch[CONFIG_UINT32];
    #endif // CONFIG_UINT32

    // (分割线)-------------------------------------|-------------------------------
    uint8_t response;                               // (环路倍率)内外环控制倍率
}Method;




#endif // SGUAN_METHOD

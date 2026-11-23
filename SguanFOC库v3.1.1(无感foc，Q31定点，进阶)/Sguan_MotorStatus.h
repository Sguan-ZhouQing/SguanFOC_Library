/* ===================================================================
 * 文件:     Sguan_MotorStatus.h
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
#ifndef __SGUAN_MOTORSTATUS_H
#define __SGUAN_MOTORSTATUS_H

/* SguanFOC配置文件声明 */
#include "Sguan_Config.h"

// 如果触发错误状态，电机直接变成Standby状态
#define STATUS_Standby              0x00    // (Standby)待机
#define STATUS_Ready                0x01    // (Ready)准备
#define STATUS_Initializing0        0x02    // (Initializing0)初始化
#define STATUS_Initializing1        0x03    // (Initializing1)初始化
#define STATUS_COM                  0x04    // (COM)正常运行

// 错误状态记录
#define ERROR_Zreo                  0x00    // (Zreo)正常运行，无错误
#define ERROR_OverVoltage           0x01    // (OverVoltage)过压
#define ERROR_UnderVoltage          0x02    // (UnderVoltage)欠压
#define ERROR_OverTemp              0x03    // (OverTemp)过温
#define ERROR_UnderTemp             0x04    // (UnderTemp)低温
#define ERROR_OverCurrent           0x05    // (OverCurrent)过流


#endif // SGUAN_MOTORSTATUS_H

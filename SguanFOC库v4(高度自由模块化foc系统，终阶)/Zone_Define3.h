/* ===================================================================
 * 文件:     Zone_Define3.h
 * 项目:     SguanFOC_v4 —— 开源 FOC 电机控制框架
 * 说明:     4.Zone用户驱动层_宏定义3
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
#ifndef __ZONE_DEFINE3_H
#define __ZONE_DEFINE3_H

#define DEFINE3_MC_VBUS         24.0                // (double)
#define DEFINE3_MC_ETA          1.0                 // (double)

#define DEFINE3_DUTY_MAX        4249                // (uint32_t)

#define DEFINE3_DIR_ENCODER     1                   // (int8_t)
#define DEFINE3_DIR_PWM         1                   // (int8_t)
#define DEFINE3_DIR_CURRENT0    1                   // (int8_t)
#define DEFINE3_DIR_CURRENT1    1                   // (int8_t)
#define DEFINE3_DIR_CURRENT2    1                   // (int8_t)

#define DEFINE3_ADC_PRECISION   4096                // (uint32_t)
#define DEFINE3_ADC_AMPLIFIER   10.0                // (double)
#define DEFINE3_ADC_VOLTAGE     3.3                 // (double)
#define DEFINE3_ADC_SAMPLING_RS 0.015               // (double)

// 定时器中断参数设计
#define DEFINE3_TIM_T 5e-5                           // 最大频率的控制周期


#endif // ZONE_DEFINE3_H

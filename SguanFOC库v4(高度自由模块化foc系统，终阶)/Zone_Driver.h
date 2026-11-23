/* ===================================================================
 * 文件:     Zone_Driver.h
 * 项目:     SguanFOC_v4 —— 开源 FOC 电机控制框架
 * 说明:     4.Zone用户驱动层_用户驱动
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
#ifndef __ZONE_DRIVER_H
#define __ZONE_DRIVER_H

/* SguanFOC配置文件声明 */
#include "SguanFOC.h"

void driver_initial_init(void);
void driver_start_init(void);
void driver_stop_init(void);

uint32_t driver_read_tick(void);
SguanQ driver_read_vbus(uint8_t CH);
SguanQ driver_read_ibus(uint8_t CH);
SguanQ driver_read_temp_motor(uint8_t CH);
SguanQ driver_read_temp_driver(uint8_t CH);
SguanQ driver_read_temp_pcb(uint8_t CH);
void driver_read_adc(uint8_t motor, uint16_t x);
void driver_set_pwm(uint8_t motor, 
    uint16_t du_0, uint16_t du_1, 
    uint16_t dv_0, uint16_t dv_1, 
    uint16_t dw_0, uint16_t dw_1,
    uint16_t dx_0, uint16_t dx_1);
void driver_send_uart(uint8_t *ch, uint16_t size);


#endif // ZONE_DRIVER_H

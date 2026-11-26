/* ===================================================================
 * 文件:     Zone_Driver.h
 * 项目:     SguanFOC_v4 —— 开源 FOC 电机控制框架
 * 说明:     4.Zone用户驱动层_用户驱动（精炼版1：表驱动直接调用）
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

// =================================================|==============================
// 1.电机驱动能力表(局部)
typedef struct{
    void   (*set_pwm)(const uint32_t duty[8]);      // (输出)PWM占空比下发
    SguanQ (*read_adc0)(void);                      // (输入)ADC0采样值
    SguanQ (*read_adc1)(void);                      // (输入)ADC1采样值
    SguanQ (*read_adc2)(void);                      // (输入)ADC2采样值
    SguanQ (*read_adc3)(void);                      // (输入)ADC3采样值
    SguanQ (*read_vbus)(void);                      // (输入)母线电压采样值
    SguanQ (*read_temp_motor)(void);                // (输入)电机温度采样值
    SguanQ (*read_temp_driver)(void);               // (输入)驱动器件温度采样值
    SguanQ (*read_temp_pcb)(void);                  // (输入)电路板温度采样值
    void (*start)(void);                            // (动作)启动
    void (*stop)(void);                             // (动作)停止
    void (*standby_loop)(void);                     // (状态)待机
    void (*ready_loop)(void);                       // (状态)就绪
    void (*initial_loop)(void);                     // (状态)初始化
    void (*holding_loop)(void);                     // (状态)数值保持
    void (*above_loop)(void);                       // (状态)数值高
    void (*below_loop)(void);                       // (状态)数值低
    void (*overvoltage_loop)(void);                 // (故障)过压
    void (*undervoltage_loop)(void);                // (故障)欠压
    void (*overcurrent_moment_loop)(void);          // (故障)瞬态过流
    void (*overcurrent_stable_loop)(void);          // (故障)稳态过流
    void (*overtemp_motor_loop)(void);              // (故障)电机过温
    void (*undertemp_motor_loop)(void);             // (故障)电机欠温
    void (*overtemp_driver_loop)(void);             // (故障)驱动器件过温
    void (*undertemp_driver_loop)(void);            // (故障)驱动器件欠温
    void (*overtemp_pcb_loop)(void);                // (故障)电路板过温
    void (*undertemp_pcb_loop)(void);               // (故障)电路板欠温
    void (*fault0_loop)(void);                      // (故障)故障0
    void (*fault1_loop)(void);                      // (故障)故障1
    void (*fault2_loop)(void);                      // (故障)故障2
}__LocalOps;

// =================================================|==============================
// 2.全局驱动函数和结构体变量声明
CPLUSPLUS_C_BEGIN // C和C++配置文件声明_BEGIN
extern const __LocalOps driver_ops[CONFIG_MOTOR];

uint32_t driver_read_tick(void);
void driver_delay_ms(uint32_t ms);
void driver_send_uart(uint8_t *ch, uint16_t size);
void driver_send_modbus(uint8_t data[8]);
void driver_initial(void);
CPLUSPLUS_C_END // C和C++配置文件声明_END


#endif // __ZONE_DRIVER_H

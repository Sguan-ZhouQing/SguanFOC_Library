/* ===================================================================
 * 文件:     SguanFOC.h
 * 项目:     SguanFOC_v4 —— 开源 FOC 电机控制框架
 * 说明:     1.Sguan固件库_(固件)算法核心
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
#ifndef __SGUANFOC_H
#define __SGUANFOC_H

/* SguanFOC配置文件声明 */
#include "Sguan_Method.h"
#include "Sguan_Modbus.h"
#include "Sguan_MotorStatus.h"
#include "Sguan_Printf.h"
#include "Sguan_Transfer.h"

// =================================================|==============================
// 1.全局宏定义设计
#define SGUANFOC_MOTOR_ONE              0X00        // 
#define SGUANFOC_MOTOR_TWO              0X01        // 
#define SGUANFOC_MOTOR_THREE            0X02        // 
#define SGUANFOC_MOTOR_FOUR             0X03        // 
#define SGUANFOC_MOTOR_FIVE             0X04        // 
#define SGUANFOC_MOTOR_SIX              0X05        // 

#define SGUANFOC_CONTROL_TORUE          0x00        // 
#define SGUANFOC_CONTROL_VELOCITY       0x01        // 
#define SGUANFOC_CONTROL_POSITION       0x02        // 

#define SGUANFOC_CURRENT_U              0x00        // 
#define SGUANFOC_CURRENT_V              0x01        // 
#define SGUANFOC_CURRENT_W              0x02        // 
#define SGUANFOC_CURRENT_OTHER          0x03        // 

#define SGUANFOC_HALL_A                 0x00        // 
#define SGUANFOC_HALL_B                 0x01        // 
#define SGUANFOC_HALL_C                 0x02        // 

// =================================================|==============================
// 2.局部结构体(声明)
typedef struct{
    SguanQ target_speed;                            // (期望速度)Target期望机械角速度
    SguanQ target_pos;                              // (期望角度)Target期望机械角度
    SguanQ target_id;                               // (期望电流)期望D轴电流
    SguanQ target_iq;                               // (期望电流)期望Q轴电流
    SguanQ target_ud;                               // (期望电压)期望D轴电压
    SguanQ target_uq;                               // (期望电压)期望Q轴电压

    SguanQ speed_in;                                // (输入量end)速度环输入值
    SguanQ ud_in;                                   // (输入量end)D轴电压输入
    SguanQ uq_in;                                   // (输入值end)Q轴电压输入

    // (分割线)-------------------------------------|-------------------------------
    SguanQ ualpha;                                  // (中间量)alpha轴电压
    SguanQ ubeta;                                   // (中间量)beta轴电压

    SguanQ du;                                      // (数据)U相占空比输入0~1
    SguanQ dv;                                      // (数据)V相占空比输入0~1
    SguanQ dw;                                      // (数据)W相占空比输入0~1

    SguanQ sine;                                    // (数据)sine临时保存的正弦值
    SguanQ cosine;                                  // (数据)cosine临时保存的余弦值

    SguanQ real_vbus;                               // (数据)Real实际的电机母线电压
    SguanQ real_is;                                 // (数据)Current实际的电流数值
    SguanQ real_temp_motor;                         // (数据)Temp实际的驱动器物理温度
    SguanQ real_temp_driver;                        // (数据)Temp实际的驱动器物理温度
    SguanQ real_temp_pcb;                           // (数据)Temp实际的驱动器物理温度
}Foc;

typedef struct{
    SguanQ real_speed;                              // ()
    SguanQ real_position;                           // ()
    SguanQ real_We;                                 // ()
    SguanQ real_Re;                                 // ()

    SguanQ real_offset;                             // ()

    // (分割线)-------------------------------------|-------------------------------
    SguanQ other_speed;                             // ()
    SguanQ other_position;                          // ()
    SguanQ other_We;                                // ()
    SguanQ other_Re;                                // ()

    SguanQ other_offset;                            // ()
}Encoder;

typedef struct{
    SguanQ real_id;                                 // (Current电流)Real实际D轴电流
    SguanQ real_iq;                                 // (Current电流)Real实际Q轴电流

    SguanQ real_ia;                                 // (Current相电流)A相电流
    SguanQ real_ib;                                 // (Current相电流)B相电流
    SguanQ real_ic;                                 // (Current相电流)C相电流

    SguanQ real_ialpha;                             // (Current中间量电流)alpha轴电流
    SguanQ real_ibeta;                              // (Current中间量电流)beta轴电流

    SguanQ real_is;                                 // (Current电流)电流峰值
    SguanQ real_ibus;                               // (Current电流)母线电流

    SguanQ final_gain;                              // (ADC增益)最终的ADC电流采样增益
    int32_t current_offset0;                        // (Current电流偏置)offset偏置位
    int32_t current_offset1;                        // (Current电流偏置)offset偏置位
    int32_t current_offset2;                        // (Current电流偏置)offset偏置位
}Current;

// =================================================|==============================
// 3.全局结构体及函数(声明)
typedef struct{
    uint8_t run;                                    // []
    uint8_t id;                                     // []

    Method method;                                  // []
    Modbus modbus;                                  // []
    MotorStatus motorstatus;                        // []
    Printf printf;                                  // []
    Transfer transfer;                              // []

    Foc foc;                                        // []
    Encoder encoder;                                // []
    Current current;                                // []

    void (*func_high_loop)(void);                   // []
    void (*func_low_loop)(void);                    // []
    void (*func_printf_loop)(uint8_t *,uint16_t);   // []
    void (*func_main_loop)(void);                   // []
}SguanFoc;

// extern SguanFoc sguanfoc[CONFIG_MOTOR];
void __local_high_loop(const uint8_t *cmds, int count);


#define SGUANFOC_HIGH_LOOP(arr) do { \
    int count = (int)(sizeof(arr)/sizeof((arr)[0])); \
    sguanfoc_high_loop((arr), count); \
} while (0)
void sguanfoc_printf_loop(uint8_t *data, uint16_t length);


#endif // SGUANFOC_H

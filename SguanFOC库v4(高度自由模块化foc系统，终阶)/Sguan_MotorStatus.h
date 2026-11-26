/* ===================================================================
 * 文件:     Sguan_MotorStatus.h
 * 项目:     SguanFOC_v4 —— 开源 FOC 电机控制框架
 * 说明:     1.Sguan固件库_电机状态
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
#ifndef __SGUAN_MOTORSTATUS_H
#define __SGUAN_MOTORSTATUS_H

/* SguanFOC配置文件声明 */
#include "Sguan_Config.h"

// =================================================|==============================
// 1.各种标志位宏定义
#define MOTORSTATUS_RUN_READY                0x01   // (system_run_flag)准备
#define MOTORSTATUS_RUN_INITIALIZING         0x02   // (system_run_flag)初始化
#define MOTORSTATUS_RUN_NEXT                 0x03   // (system_run_flag)传递
#define MOTORSTATUS_RUN_STANDBY              0x04   // (system_run_flag)待机

#define MOTORSTATUS_ERROR_ZERO               0x00   // (error_code)错误码
#define MOTORSTATUS_ERROR_OVERVOLTAGE        0x01   // (error_code)错误码
#define MOTORSTATUS_ERROR_UNDERVOLTAGE       0x02   // (error_code)错误码
#define MOTORSTATUS_ERROR_OVERCURRENT_MOMENT 0x03   // (error_code)错误码
#define MOTORSTATUS_ERROR_OVERCURRENT_STABLE 0x04   // (error_code)错误码
#define MOTORSTATUS_ERROR_OVERTEMP_MOTOR     0x05   // (error_code)错误码
#define MOTORSTATUS_ERROR_UNDERTEMP_MOTOR    0x06   // (error_code)错误码
#define MOTORSTATUS_ERROR_OVERTEMP_DRIVER    0x07   // (error_code)错误码
#define MOTORSTATUS_ERROR_UNDERTEMP_DRIVER   0x08   // (error_code)错误码
#define MOTORSTATUS_ERROR_OVERTEMP_PCB       0x09   // (error_code)错误码
#define MOTORSTATUS_ERROR_UNDERTEMP_PCB      0x0A   // (error_code)错误码
#define MOTORSTATUS_ERROR_FAULT0             0x0B   // (error_code)错误码
#define MOTORSTATUS_ERROR_FAULT1             0x0C   // (error_code)错误码
#define MOTORSTATUS_ERROR_FAULT2             0x0D   // (error_code)错误码

// =================================================|==============================
// 2.局部结构体__Local声明
typedef enum{
    status_standby = 0,                             // (待机状态)待机

    status_ready,                                   // (初始化状态)准备
    status_initializing,                            // (初始化状态)启动初始
    status_start0_current,                          // (初始化状态)电流开始
    status_start1_angle,                            // (初始化状态)角度开始

    status_idle,                                    // (正常状态)空闲
    status_value_holding,                           // (正常状态)数值保持中
    status_value_above,                             // (正常状态)数值高于控制值
    status_value_below,                             // (正常状态)数值低于控制值

    status_overvoltage,                             // (错误状态)过压
    status_undervoltage,                            // (错误状态)欠压
    status_overcurrent_moment,                      // (错误状态)瞬态过流
    status_overcurrent_stable,                      // (错误状态)稳态过流
    status_overtemp_motor,                          // (错误状态)电机过温
    status_undertemp_motor,                         // (错误状态)电机过冷
    status_overtemp_driver,                         // (错误状态)功率器件过温
    status_undertemp_driver,                        // (错误状态)功率器件过冷
    status_overtemp_pcb,                            // (错误状态)驱动器过温
    status_undertemp_pcb,                           // (错误状态)驱动器过冷
    status_fault0,                                  // (错误状态)未知错误0
    status_fault1,                                  // (错误状态)未知错误1
    status_fault2                                   // (错误状态)未知错误2
}__LocalStatus;

typedef enum{
    // (分割线)-------------------------------------|-------------------------------
    // standby      ready          start0_current  start1_angle
    // 电流偏置计算||开PWM和栅极驱动||->电流计算开始->角度计算开始->控制器
    // ...........................||->角度偏置计算->xxxxxxxxxxx->xxxxx

    // (分割线)-------------------------------------|-------------------------------
    state_standby = 0,                              // (standby)待机
    state_ready,                                    // (ready)准备
    state_initializing,                             // (initializing)启动初始
    state_start0_current,                           // (current)电流开始
    state_start1_angle,                             // (angle)角度开始
    state_com,                                      // (com)正常
    state_fault                                     // (fault)错误
}__LocalState;

typedef struct{
    SguanQ value_real;                              // (实时输入数据)实际控制数值
    SguanQ value_target;                            // (实时输入数据)期望控制数值

    SguanQ vbus_real;                               // (实时输入数据)实际母线电压
    SguanQ is_real;                                 // (实时输入数据)实际xx电流

    SguanQ temp_motor_real;                         // (实时输入数据)电机温度
    SguanQ temp_driver_real;                        // (实时输入数据)驱动功率器件温度
    SguanQ temp_pcb_real;                           // (实时输入数据)硬件电路板温度

    uint8_t fault_flag0;                            // (实时输入数据)未知错误0
    uint8_t fault_flag1;                            // (实时输入数据)未知错误1
    uint8_t fault_flag2;                            // (实时输入数据)未知错误2

    // (分割线)-------------------------------------|-------------------------------
    uint32_t is_tick_now;                           // (临时数据)xx电流稳态过流计数器
    uint32_t fault_tick_now;                        // (临时数据)错误状态延时计数器

    // (分割线)-------------------------------------|-------------------------------
    uint32_t system_tick_now;                       // (实时输入数据)实时运行时间
    uint8_t system_run_flag;                        // (实时输入数据)手动状态切换
}__LocalData;

typedef struct{
    SguanQ value_interval;                          // (参数)数值浮动区间

    SguanQ vbus_max;                                // (参数)母线电压浮动上限
    SguanQ vbus_min;                                // (参数)母线电压浮动下限

    SguanQ is_moment_max;                           // (参数)瞬态上限
    SguanQ is_stable_max;                           // (参数)稳态上限
    
    SguanQ temp_motor_max;                          // (参数)电机温度上限
    SguanQ temp_motor_min;                          // (参数)电机温度下限
    SguanQ temp_driver_max;                         // (参数)驱动功率器件温度上限
    SguanQ temp_driver_min;                         // (参数)驱动功率器件温度下限
    SguanQ temp_pcb_max;                            // (参数)硬件电路板温度上限
    SguanQ temp_pcb_min;                            // (参数)硬件电路板温度下限

    // (分割线)-------------------------------------|-------------------------------
    uint32_t is_tick_count;                         // (参数)稳态最大维持时间
    uint32_t fault_tick_count;                      // (参数)稳态最大维持时间

    // (分割线)-------------------------------------|-------------------------------
    uint8_t motor_id;                               // (参数)电机当前排序0-n
    uint8_t mode_flag;                              // (参数)电机运行模式0-2
}__LocalParams;

// =================================================|==============================
// 3.全局结构体及函数声明
typedef struct{
    __LocalStatus status;                           // (status)现在状态
    __LocalData data;                               // (data)动态实时数据
    __LocalParams params;                           // (params)固定参数

    __LocalStatus last;                             // (last)过去状态...仅用于判断触发一次“status的init”
    __LocalState state;                             // (state)状况标志位...用于sguanfoc电机判断当前状态

    uint8_t error_code;                             // (error_code)错误码记录
}MotorStatus;

CPLUSPLUS_C_BEGIN // C和C++配置文件声明_BEGIN
void motorstatus_ready_init(MotorStatus *motorstatus);
void motorstatus_initializing_init(MotorStatus *motorstatus);
void motorstatus_next_init(MotorStatus *motorstatus);
void motorstatus_standby_init(MotorStatus *motorstatus);
void motorstatus_loop(MotorStatus *motorstatus);
CPLUSPLUS_C_END // C和C++配置文件声明_END


#endif // SGUAN_MOTORSTATUS_H

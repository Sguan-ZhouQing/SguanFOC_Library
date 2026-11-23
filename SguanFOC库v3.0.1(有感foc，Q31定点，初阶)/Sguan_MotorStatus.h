/* ===================================================================
 * 文件:     Sguan_MotorStatus.h
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
#ifndef __SGUAN_MOTORSTATUS_H
#define __SGUAN_MOTORSTATUS_H

/* SguanFOC配置文件声明 */
#include "UserData_Status.h"
#include "Sguan_Config.h"

// ====== 初始化与运行状态(状态) ======
#define MOTOR_STATUS_STANDBY                0x00    // 待机（未初始化，准备中）
#define MOTOR_STATUS_UNINITIALIZED          0x01    // 未初始化
#define MOTOR_STATUS_INITIALIZING           0x02    // 初始化中（参数加载、外设初始化，编码器零位）
#define MOTOR_STATUS_CALIBRATING            0x03    // 校准（此时SVPWM可用，用于转子校正）

// ====== 运行状态(当前反馈) ======
#define MOTOR_STATUS_IDLE                   0x04    // 空闲（已初始化，使能但零指令）

#define MOTOR_STATUS_TORQUE_INCREASING      0x05    // 力矩增大中~电流模式(下时刻->力矩保持)
#define MOTOR_STATUS_TORQUE_DECREASING      0x06    // 力矩减小中~电流模式(下时刻->力矩保持)
#define MOTOR_STATUS_TORQUE_CONTROL         0x07    // 力矩保持~电流模式(稳态)

#define MOTOR_STATUS_ACCELERATING           0x08    // 加速中~速度模式(下时刻->恒速保持)
#define MOTOR_STATUS_DECELERATING           0x09    // 减速中~速度模式(下时刻->恒速保持)
#define MOTOR_STATUS_CONST_SPEED            0x0A    // 恒速保持~速度模式(稳态)

#define MOTOR_STATUS_POSITION_INCREASING    0x0B    // 位置增加中~位置模式(下时刻->位置保持)
#define MOTOR_STATUS_POSITION_DECREASING    0x0C    // 位置减少中~位置模式(下时刻->位置保持)
#define MOTOR_STATUS_POSITION_HOLD          0x0D    // 位置保持~位置模式(稳态)

// ====== 硬件相关错误(状态) ======
#define MOTOR_STATUS_OVERVOLTAGE            0x0E    // 过压保护(锁定->手动解除进待机)
#define MOTOR_STATUS_UNDERVOLTAGE           0x0F    // 欠压保护(锁定->手动解除进待机)
#define MOTOR_STATUS_OVERTEMPERATURE        0x10    // 过温保护(锁定->手动解除进待机)
#define MOTOR_STATUS_UNDERTEMPERATURE       0x11    // 低温保护(锁定->手动解除进待机)
#define MOTOR_STATUS_OVERCURRENT            0x12    // 过流保护(稳态->电机电流限幅)

#define MOTOR_STATUS_ENCODER_ERROR          0x13    // 编码器故障(锁定->手动解除进待机)
#define MOTOR_STATUS_SENSOR_ERROR           0x14    // 传感器故障(锁定->手动解除进待机)
#define MOTOR_STATUS_PWM_CALC_FAULT         0x15    // PWM计算错误(锁定->手动解除进待机)

// ====== 安全状态(状态) ======
#define MOTOR_STATUS_EMERGENCY_STOP         0x16    // 急停（立即关闭PWM,会立即锁定->手动解除进待机）
#define MOTOR_STATUS_DISABLED               0x17    // 已失能（软关闭,会缓慢进入待机->自动进待机）

// 函数定义声明
void MotorStatus_Loop(uint8_t *status);


#endif // SGUAN_MOTORSTATUS_H

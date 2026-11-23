/* ===================================================================
 * 文件:     Sguan_MotorStatus.c
 * 项目:     SguanFOC_v3.0.0 —— 开源 FOC 电机控制框架（有感FOC · 浮点运算 · 初阶）
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
#include "Sguan_MotorStatus.h"

// 函数指针数组（按枚举顺序排列）
static void (*const status_handlers[])(void) = {
    // 初始化与运行状态
    MOTOR_STATUS_STANDBY_Loop,
    MOTOR_STATUS_UNINITIALIZED_Loop,
    MOTOR_STATUS_INITIALIZING_Loop,
    MOTOR_STATUS_CALIBRATING_Loop,
    
    // 运行状态
    MOTOR_STATUS_IDLE_Loop,
    MOTOR_STATUS_TORQUE_INCREASING_Loop,
    MOTOR_STATUS_TORQUE_DECREASING_Loop,
    MOTOR_STATUS_TORQUE_CONTROL_Loop,
    MOTOR_STATUS_ACCELERATING_Loop,
    MOTOR_STATUS_DECELERATING_Loop,
    MOTOR_STATUS_CONST_SPEED_Loop,
    MOTOR_STATUS_POSITION_INCREASING_Loop,
    MOTOR_STATUS_POSITION_DECREASING_Loop,
    MOTOR_STATUS_POSITION_HOLD_Loop,
    
    // 硬件错误
    MOTOR_STATUS_OVERVOLTAGE_Loop,
    MOTOR_STATUS_UNDERVOLTAGE_Loop,
    MOTOR_STATUS_OVERTEMPERATURE_Loop,
    MOTOR_STATUS_UNDERTEMPERATURE_Loop,
    MOTOR_STATUS_OVERCURRENT_Loop,
    MOTOR_STATUS_ENCODER_ERROR_Loop,
    MOTOR_STATUS_SENSOR_ERROR_Loop,
    MOTOR_STATUS_PWM_CALC_FAULT_Loop,
    
    // 安全状态
    MOTOR_STATUS_EMERGENCY_STOP_Loop,
    MOTOR_STATUS_DISABLED_Loop
};

// 核心函数MotorStatus_Loop函数
void MotorStatus_Loop(uint8_t *status){
    if (*status < 24){
        status_handlers[*status]();
    } else{
        // 错误处理：记录日志并跳转到安全状态
        *status = MOTOR_STATUS_STANDBY;
        status_handlers[MOTOR_STATUS_STANDBY]();
    }
}


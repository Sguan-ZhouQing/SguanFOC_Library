/*
 * @Author: 星必尘Sguan
 * @GitHub: https://github.com/Sguan-ZhouQing
 * @Date: 2026-01-26 22:43:42
 * @LastEditors: 星必尘Sguan|3464647102@qq.com
 * @LastEditTime: 2026-02-04 23:03:00
 * @FilePath: \demo_SguanFOCCode\SguanFOC库\Sguan_MotorStatus.c
 * @Description: SguanFOC库的“电机状态机”实现
 * 
 * Copyright (c) 2026 by $星必尘Sguan, All Rights Reserved. 
 */
#include "Sguan_MotorStatus.h"

/* 外部用户设置函数声明 */
#include "UserData_Status.h"

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
    MOTOR_STATUS_PHASE_LOSS_Loop,
    MOTOR_STATUS_PHASE_SHORT_Loop,
    
    // 安全状态
    MOTOR_STATUS_EMERGENCY_STOP_Loop,
    MOTOR_STATUS_DISABLED_Loop
};

// 核心函数MotorStatus_Loop函数
void MotorStatus_Loop(MOTOR_STATUS_ENUM *status){
    if (*status >= 0 && *status < 25){
        status_handlers[*status]();
    } else{
        // 错误处理：记录日志并跳转到安全状态
        *status = MOTOR_STATUS_STANDBY;
        status_handlers[MOTOR_STATUS_STANDBY]();
    }
}


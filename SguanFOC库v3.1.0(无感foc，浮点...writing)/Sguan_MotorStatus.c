/*
 * @Author: 星必尘Sguan
 * @GitHub: https://github.com/Sguan-ZhouQing
 * @Date: 2026-01-26 22:43:42
 * @LastEditors: 星必尘Sguan|3464647102@qq.com
 * @LastEditTime: 2026-04-19 01:00:38
 * @FilePath: \SguanFOC_Debug\SguanFOC\Sguan_MotorStatus.c
 * @Description: SguanFOC库的“电机状态机”实现
 * 
 * Copyright (c) 2026 by $星必尘Sguan, All Rights Reserved. 
 */
#include "Sguan_MotorStatus.h"

// 函数指针数组（按枚举顺序排列）
static void (*const status_handlers[])(void) = {
    // 初始化与运行状态
    STANDBY_Loop,
    UNINITIALIZED_Loop,
    INITIALIZING_Loop,
    CALIBRATING_Loop,
    
    // 运行状态
    IDLE_Loop,
    TORQUE_INCREASING_Loop,
    TORQUE_DECREASING_Loop,
    TORQUE_CONTROL_Loop,
    ACCELERATING_Loop,
    DECELERATING_Loop,
    CONST_SPEED_Loop,
    POSITION_INCREASING_Loop,
    POSITION_DECREASING_Loop,
    POSITION_HOLD_Loop,
    
    // 硬件错误
    OVERVOLTAGE_Loop,
    UNDERVOLTAGE_Loop,
    OVERTEMPERATURE_Loop,
    UNDERTEMPERATURE_Loop,
    OVERCURRENT_Loop,
    ENCODER_ERROR_Loop,
    SENSOR_ERROR_Loop,
    PWM_CALC_FAULT_Loop,
    
    // 安全状态
    EMERGENCY_STOP_Loop,
    DISABLED_Loop
};

/**
 * @description: 核心函数MotorStatus_Loop函数
 * @param {uint8_t} *status
 * @return {*}
 */
void MotorStatus_Loop(uint8_t *status){
    // 运行状态机任务指示函数
    // 带“输入参数”数值限定
    status_handlers[Value_set(*status,
        MOTOR_STATUS_DISABLED,0)]();
}


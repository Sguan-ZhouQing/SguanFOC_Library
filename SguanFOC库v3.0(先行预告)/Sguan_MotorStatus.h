/*
 * @Author: 星必尘Sguan
 * @GitHub: https://github.com/Sguan-ZhouQing
 * @Date: 2026-01-26 22:43:48
 * @LastEditors: 星必尘Sguan|3464647102@qq.com
 * @LastEditTime: 2026-01-31 14:37:52
 * @FilePath: \demo_SguanFOCCode\SguanFOC库\Sguan_MotorStatus.h
 * @Description: 
 * 
 * Copyright (c) 2026 by $星必尘Sguan, All Rights Reserved. 
 */
#ifndef __SGUAN_MOTORSTATUS_H
#define __SGUAN_MOTORSTATUS_H

typedef enum{
    // ====== 初始化与错误状态 ======
    MOTOR_STATUS_UNINITIALIZED = 0,   // 未初始化
    MOTOR_STATUS_INITIALIZING,        // 初始化中（参数加载、外设初始化）
    MOTOR_STATUS_CALIBRATING,         // 校准中（电阻、电感、编码器零位）
    MOTOR_STATUS_ERROR,               // 错误状态（通用错误）
    
    // ====== 硬件相关错误 ======
    MOTOR_STATUS_OVERCURRENT,         // 过流保护
    MOTOR_STATUS_OVERVOLTAGE,         // 过压保护
    MOTOR_STATUS_UNDERVOLTAGE,        // 欠压保护
    MOTOR_STATUS_OVERTEMPERATURE,     // 过温保护
    MOTOR_STATUS_UNDERTEMPERATURE,    // 低温保护
    MOTOR_STATUS_ENCODER_ERROR,       // 编码器故障
    MOTOR_STATUS_SENSOR_ERROR,        // 传感器故障（电流、温度等）
    MOTOR_STATUS_COMMUTATION_ERROR,   // 换相错误
    
    // ====== 安全状态 ======
    MOTOR_STATUS_FAULT_LOCK,          // 故障锁定（需要复位）
    MOTOR_STATUS_SAFE_STOP,           // 安全停止（快速制动）
    MOTOR_STATUS_EMERGENCY_STOP,      // 急停（立即关闭PWM）
    
    // ====== 准备与使能状态 ======
    MOTOR_STATUS_STANDBY,             // 待机（PWM关闭，但系统就绪）
    MOTOR_STATUS_ENABLING,            // 使能中（软启动过程）
    MOTOR_STATUS_ENABLED,             // 已使能（PWM开启，可接收指令）
    
    // ====== 运行状态 ======
    MOTOR_STATUS_IDLE,                // 空闲（使能但零指令）
    MOTOR_STATUS_ACCELERATING,        // 加速中
    MOTOR_STATUS_DECELERATING,        // 减速中
    MOTOR_STATUS_RUNNING,             // 稳定运行
    MOTOR_STATUS_CONST_SPEED,         // 恒速运行
    MOTOR_STATUS_POSITIONING,         // 定位中
    MOTOR_STATUS_POSITION_HOLD,       // 位置保持
    MOTOR_STATUS_TORQUE_CONTROL,      // 力矩控制中
    
    // ====== 特殊运行模式 ======
    MOTOR_STATUS_HOMING,              // 回零中（寻找机械零点）
    MOTOR_STATUS_TEACHING,            // 示教模式（学习轨迹）
    MOTOR_STATUS_TRAJECTORY_TRACKING, // 轨迹跟踪
    MOTOR_STATUS_SYNCHRONIZING,       // 同步中（多轴同步）
    
    // ====== 停止与复位状态 ======
    MOTOR_STATUS_STOPPING,            // 停止中（平滑停止）
    MOTOR_STATUS_BRAKING,             // 制动中（能耗制动/回馈制动）
    MOTOR_STATUS_DISABLING,           // 失能中（软关闭）
    MOTOR_STATUS_DISABLED,            // 已失能（PWM关闭）
    MOTOR_STATUS_RESETTING,           // 复位中（清除错误，重新初始化）
    
    // ====== 调试与维护状态 ======
    MOTOR_STATUS_TEST_MODE,           // 测试模式
    MOTOR_STATUS_CALIBRATION_MODE,    // 校准模式
    MOTOR_STATUS_DIAGNOSTIC,          // 诊断模式
    MOTOR_STATUS_PARAM_TUNING,        // 参数整定模式
    
    MOTOR_STATUS_COUNT                // 状态总数（用于边界检查）
}MOTOR_STATUS_ENUM;


void MotorStatus_Loop(MOTOR_STATUS_ENUM *status);

#endif // SGUAN_MOTORSTATUS_H

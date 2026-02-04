#ifndef __SGUAN_MOTORSTATUS_H
#define __SGUAN_MOTORSTATUS_H

typedef enum{
    // ====== 初始化与运行状态(状态) ======
    MOTOR_STATUS_STANDBY = 0,           // 待机（未初始化，准备中）
    MOTOR_STATUS_UNINITIALIZED,         // 未初始化
    MOTOR_STATUS_INITIALIZING,          // 初始化中（参数加载、外设初始化）
    MOTOR_STATUS_CALIBRATING,           // 校准中（电阻、电感、编码器零位）

    // ====== 运行状态(当前反馈) ======
    MOTOR_STATUS_IDLE,                  // 空闲（已初始化，使能但零指令）
    
    MOTOR_STATUS_TORQUE_INCREASING,     // 力矩增大中~电流模式(下时刻->力矩保持)
    MOTOR_STATUS_TORQUE_DECREASING,     // 力矩减小中~电流模式(下时刻->力矩保持)
    MOTOR_STATUS_TORQUE_CONTROL,        // 力矩保持~电流模式(稳态)

    MOTOR_STATUS_ACCELERATING,          // 加速中~速度模式(下时刻->恒速保持)
    MOTOR_STATUS_DECELERATING,          // 减速中~速度模式(下时刻->恒速保持)
    MOTOR_STATUS_CONST_SPEED,           // 恒速保持~速度模式(稳态)

    MOTOR_STATUS_POSITION_INCREASING,   // 位置增加中~位置模式(下时刻->位置保持)
    MOTOR_STATUS_POSITION_DECREASING,   // 位置减少中~位置模式(下时刻->位置保持)
    MOTOR_STATUS_POSITION_HOLD,         // 位置保持~位置模式(稳态)
    
    // ====== 硬件相关错误(状态) ======
    MOTOR_STATUS_OVERVOLTAGE,           // 过压保护(锁定->手动解除进待机)
    MOTOR_STATUS_UNDERVOLTAGE,          // 欠压保护(锁定->手动解除进待机)
    MOTOR_STATUS_OVERTEMPERATURE,       // 过温保护(锁定->手动解除进待机)
    MOTOR_STATUS_UNDERTEMPERATURE,      // 低温保护(锁定->手动解除进待机)
    MOTOR_STATUS_OVERCURRENT,           // 过流保护(稳态->电机电流限幅)

    MOTOR_STATUS_ENCODER_ERROR,         // 编码器故障(锁定->手动解除进待机)
    MOTOR_STATUS_SENSOR_ERROR,          // 传感器故障(锁定->手动解除进待机)
    
    MOTOR_STATUS_PHASE_LOSS,            // 缺相错误(锁定->手动解除进待机)
    MOTOR_STATUS_PHASE_SHORT,           // 相线短路错误(锁定->手动解除进待机)
    
    // ====== 安全状态(状态) ======
    MOTOR_STATUS_EMERGENCY_STOP,        // 急停（立即关闭PWM,会立即锁定->手动解除进待机）
    MOTOR_STATUS_DISABLED,              // 已失能（软关闭,会缓慢进入待机->自动进待机）
}MOTOR_STATUS_ENUM;

void MotorStatus_Loop(MOTOR_STATUS_ENUM *status);


#endif // SGUAN_MOTORSTATUS_H

#ifndef __USERDATA_STATUS_H
#define __USERDATA_STATUS_H
#include <stdint.h>
/* 电机控制User用户设置·状态管理 */
/* 用户自己的CODE BEGIN Includes */

/* 用户自己的CODE END Includes */

/* ================= 状态机任务信号(输入) ================= */
static inline uint8_t MOTOR_STATUS_STANDBY_Signal(void){
    uint8_t STANDBY_num = 0;
    /* 解除锁定信号(Emergency stop signal) */
    /* 输出0->不执行 输出1->解除锁定(进入待机状态) */
    return STANDBY_num;
}

static inline uint8_t MOTOR_STATUS_UNINITIALIZED_Signal(void){
    uint8_t UNINITIALIZED_num = 0;
    /* 准备开始初始化信号(Emergency stop signal) */
    /* 输出0->待机 输出1->准备开始初始化 */
    return UNINITIALIZED_num;
}

static inline uint8_t MOTOR_STATUS_ENCODER_ERROR_Signal(void){
    uint8_t ENCODER_num = 0;
    /* 编码器错误信号(Emergency stop signal) */
    /* 输出0->正常运行 输出1->编码器错误(锁定) */
    return ENCODER_num;
}

static inline uint8_t MOTOR_STATUS_SENSOR_ERROR_Signal(void){
    uint8_t SENSOR_num = 0;
    /* 传感器错误信号(Emergency stop signal) */
    /* 输出0->正常运行 输出1->传感器错误(锁定) */
    return SENSOR_num;
}

static inline uint8_t MOTOR_STATUS_EMERGENCY_STOP_Signal(void){
    uint8_t STOP_num = 0;
    /* 急停信号(Emergency stop signal) */
    /* 输出0->正常运行 输出1->启用急停(锁定) */
    return STOP_num;
}

static inline uint8_t MOTOR_STATUS_DISABLED_Signal(void){
    uint8_t DISABLED_num = 0;
    /* 失能信号(Disabling signal) */
    /* 输出0->正常运行 输出1->启用失能 */
    return DISABLED_num;
}


/* ================= 状态机任务处理(执行) ================= */
static inline void MOTOR_STATUS_STANDBY_Loop(void){
    /* Your code for 待机（未初始化，准备中） here */
}

static inline void MOTOR_STATUS_UNINITIALIZED_Loop(void){
    /* Your code for 未初始化 here */
}

static inline void MOTOR_STATUS_INITIALIZING_Loop(void){
    /* Your code for 初始化中（参数加载、外设初始化） here */
}

static inline void MOTOR_STATUS_CALIBRATING_Loop(void){
    /* Your code for 校准中（电阻、电感、编码器零位） here */
}

static inline void MOTOR_STATUS_IDLE_Loop(void){
    /* Your code for 空闲（已初始化，使能但零指令） here */
}

static inline void MOTOR_STATUS_TORQUE_INCREASING_Loop(void){
    /* Your code for 力矩增大中~电流模式(下时刻->力矩保持) here */
}

static inline void MOTOR_STATUS_TORQUE_DECREASING_Loop(void){
    /* Your code for 力矩减小中~电流模式(下时刻->力矩保持) here */
}

static inline void MOTOR_STATUS_TORQUE_CONTROL_Loop(void){
    /* Your code for 力矩保持~电流模式(稳态) here */
}

static inline void MOTOR_STATUS_ACCELERATING_Loop(void){
    /* Your code for 加速中~速度模式(下时刻->恒速保持) here */
}

static inline void MOTOR_STATUS_DECELERATING_Loop(void){
    /* Your code for 减速中~速度模式(下时刻->恒速保持) here */
}

static inline void MOTOR_STATUS_CONST_SPEED_Loop(void){
    /* Your code for 恒速保持~速度模式(稳态) here */
}

static inline void MOTOR_STATUS_POSITION_INCREASING_Loop(void){
    /* Your code for 位置增加中~位置模式(下时刻->位置保持) here */
}

static inline void MOTOR_STATUS_POSITION_DECREASING_Loop(void){
    /* Your code for 位置减少中~位置模式(下时刻->位置保持) here */
}

static inline void MOTOR_STATUS_POSITION_HOLD_Loop(void){
    /* Your code for 位置保持~位置模式(稳态) here */
}

static inline void MOTOR_STATUS_OVERVOLTAGE_Loop(void){
    /* Your code for 过压保护(锁定->手动解除进待机) here */
}

static inline void MOTOR_STATUS_UNDERVOLTAGE_Loop(void){
    /* Your code for 欠压保护(锁定->手动解除进待机) here */
}

static inline void MOTOR_STATUS_OVERTEMPERATURE_Loop(void){
    /* Your code for 过温保护(锁定->手动解除进待机) here */
}

static inline void MOTOR_STATUS_UNDERTEMPERATURE_Loop(void){
    /* Your code for 低温保护(锁定->手动解除进待机) here */
}

static inline void MOTOR_STATUS_OVERCURRENT_Loop(void){
    /* Your code for 过流保护(稳态->电机电流限幅) here */
}

static inline void MOTOR_STATUS_ENCODER_ERROR_Loop(void){
    /* Your code for 编码器故障(锁定->手动解除进待机) here */
}

static inline void MOTOR_STATUS_SENSOR_ERROR_Loop(void){
    /* Your code for 传感器故障(锁定->手动解除进待机) here */
}

static inline void MOTOR_STATUS_PWM_CALC_FAULT_Loop(void){
    /* Your code for PWM计算错误(锁定->手动解除进待机) here */
}

static inline void MOTOR_STATUS_EMERGENCY_STOP_Loop(void){
    /* Your code for 急停（立即关闭PWM,会立即锁定->手动解除进待机） here */
}

static inline void MOTOR_STATUS_DISABLED_Loop(void){
    /* Your code for 已失能（软关闭,会缓慢进入待机->自动进待机） here */
}


#endif // USERDATA_STATUS_H

#ifndef __SGUANFOC_H
#define __SGUANFOC_H

#include <string.h>  // 添加这行，用于memset函数
#include <math.h>    // 添加这行，用于fabsf等数学函数
#include <stdint.h>

/**
 * @description: FOC控制系统核心数据结构
 * 包含所有输入、输出和中间变量，方便在函数间传递数据
 */
typedef struct {
    /* 【输入】目标电压和角度（来自PID控制器） */
    float u_d;       // d轴电压指令（励磁分量，通常控制为0）
    float u_q;       // q轴电压指令（转矩分量，控制电机扭矩）
    float theta;     // 转子电角度（来自编码器或观测器）

    /* 【中间变量】α-β坐标系电压（逆Park变换结果） */
    float u_alpha;   // α轴电压
    float u_beta;    // β轴电压

    /* 【输出】三相PWM占空比（SVPWM计算结果） */
    float t_a;       // A相PWM占空比
    float t_b;       // B相PWM占空比
    float t_c;       // C相PWM占空比

    /* 【输入】三相采样电流（来自ADC） */
    float i_a;       // A相电流
    float i_b;       // B相电流
    float i_c;       // C相电流

    /* 【中间变量】α-β坐标系电流（Clark变换结果） */
    float i_alpha;   // α轴电流
    float i_beta;    // β轴电流

    /* 【输出】d-q坐标系电流（Park变换结果，用于PID反馈） */
    float i_d;       // d轴电流（反馈给d轴PID）
    float i_q;       // q轴电流（反馈给q轴PID）

    /* 【中间变量】三角函数值（避免重复计算） */
    float sine;      // sin(theta)
    float cosine;    // cos(theta)
    float k_svpwm;   // SVPWM调制系数（超调时使用）
} SVPWM_HandleTypeDef;


typedef struct {
    // 1.（单环）位置闭环
    struct {
        float Kp;           // 比例系数
        float Ki;           // 积分系数
        float Kd;           // 微分系数
        float Integral;     // 积分项累加值
        float Prev_error;   // 上一次误差（用于微分项）
        float Output_limit; // 输出限制
    } Position_PID;
    // 2.（单环）速度闭环
    struct {
        float Kp;
        float Ki;
        float Kd;
        float Integral;
        float Prev_error;
        float Output_limit;
    } Velocity_PID;
    // 3.（单环）电流闭环
    struct {
        float Kp;
        float Ki;
        float Kd;
        float Integral;
        float Prev_error;
        float Output_limit;
    } Current_PID;

    // 4.速度-电流双环串级控制(速度控制常选)
    struct {
        struct {
            float Kp;
            float Ki;
            float Kd;
            float Integral;
            float Prev_error;
            float Output_limit;
        } Velocity;             // 速度环
        struct {
            float Kp;
            float Ki;
            float Kd;
            float Integral;
            float Prev_error;
            float Output_limit;
        } Current;              // 电流环
    } Velocity_Current_Cascade;
    
    // 5.位置-速度双环串级控制(位置控制常选)
    struct {
        struct {
            float Kp;
            float Ki;
            float Kd;
            float Integral;
            float Prev_error;
            float Output_limit;
        } Position;             // 位置环
        struct {
            float Kp;
            float Ki;
            float Kd;
            float Integral;
            float Prev_error;
            float Output_limit;
        } Velocity;             // 速度环
    } Position_Velocity_Cascade;

    // 6.位置-速度-电流三环串级控制(主流)
    struct {
        struct {
            float Kp;           // 比例系数
            float Ki;           // 积分系数
            float Kd;           // 微分系数
            float Integral;     // 积分项累加值
            float Prev_error;   // 上一次误差
            float Output_limit; // 输出限制
        } Position;             // 位置环
        struct {
            float Kp;
            float Ki;
            float Kd;
            float Integral;
            float Prev_error;
            float Output_limit;
        } Velocity;             // 速度环
        struct {
            float Kp;
            float Ki;
            float Kd;
            float Integral;
            float Prev_error;
            float Output_limit;
        } Current;              // 电流环
    } Position_Velocity_Current_Cascade;   
} FOC_HandleTypeDef;


// 控制模式枚举
typedef enum {
    FOC_MODE_NONE = 0,
    // 开环
    FOC_MODE_OPEN_POSITION,
    FOC_MODE_OPEN_VELOCITY,
    // 单环闭环
    FOC_MODE_POSITION_SINGLE,
    FOC_MODE_VELOCITY_SINGLE,
    FOC_MODE_CURRENT_SINGLE,
    // 双环闭环
    FOC_MODE_POSITION_VELOCITY_CASCADE,
    FOC_MODE_VELOCITY_CURRENT_CASCADE,
    // 三环闭环
    FOC_MODE_POSITION_VELOCITY_CURRENT_CASCADE
} FOC_Mode_t;


// FOC控制的结构体变量(声明)
extern SVPWM_HandleTypeDef SguanSVPWM;
extern FOC_HandleTypeDef SguanFOC;
// 全局模式变量 & 目标量
extern FOC_Mode_t FOC_Mode;
extern float FOC_Target_Position; // rad
extern float FOC_Target_Speed;    // rad/s
extern float FOC_Target_Current;  // A
extern float FOC_Target_Voltage;  // 0-1 (开环时使用)


// 磁定向控制FOC函数声明汇总
void FOC_Init(void);
// 多环运行调度函数
void FOC_LoopHandler(void);
// PID参数调节函数
void FOC_SetPIDParams(const char *loop, float kp, float ki, float kd, float limit);


#endif // SGUANFOC_H

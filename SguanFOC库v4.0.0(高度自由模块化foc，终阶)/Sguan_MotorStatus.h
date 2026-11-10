#ifndef __SGUAN_MOTORSTATUS_H
#define __SGUAN_MOTORSTATUS_H

/* SguanFOC配置文件声明 */
#include "Sguan_Config.h"

// ================================================================================
// 1.运行标志位run_flag宏定义
#define MOTORSTATUS_RUN_READY                   0x01 // (run_flag)准备
#define MOTORSTATUS_RUN_INITIALIZING            0x02 // (run_flag)初始化
#define MOTORSTATUS_RUN_STANDBY                 0x03 // (run_flag)待机

#define MOTORSTATUS_ERROR_OVERVOLTAGE           0x01 // (error_code)错误码
#define MOTORSTATUS_ERROR_UNDERVOLTAGE          0x02 // (error_code)错误码
#define MOTORSTATUS_ERROR_OVERCURRENT_MOMENT    0x03 // (error_code)错误码
#define MOTORSTATUS_ERROR_OVERCURRENT_STABLE    0x04 // (error_code)错误码
#define MOTORSTATUS_ERROR_OVERTEMP_MOTOR        0x05 // (error_code)错误码
#define MOTORSTATUS_ERROR_UNDERTEMP_MOTOR       0x06 // (error_code)错误码
#define MOTORSTATUS_ERROR_OVERTEMP_DRIVER       0x07 // (error_code)错误码
#define MOTORSTATUS_ERROR_UNDERTEMP_DRIVER      0x08 // (error_code)错误码
#define MOTORSTATUS_ERROR_OVERTEMP_PCB          0x09 // (error_code)错误码
#define MOTORSTATUS_ERROR_UNDERTEMP_PCB         0x0A // (error_code)错误码
#define MOTORSTATUS_ERROR_FAULT0                0x0B // (error_code)错误码
#define MOTORSTATUS_ERROR_FAULT1                0x0C // (error_code)错误码
#define MOTORSTATUS_ERROR_FAULT2                0x0D // (error_code)错误码

// ================================================================================
// 2.局部结构体__Local(声明)
typedef enum{
    status_standby = 0,                         // (待机状态)待机

    status_ready,                               // (初始化状态)准备
    status_initializing,                        // (初始化状态)开始初始
    status_gotoinit0_angle,                     // (初始化状态)角度专项
    status_gotoinit1_current,                   // (初始化状态)电流专项

    status_idle,                                // (正常状态)空闲
    status_torque_increasing,                   // (正常状态)力矩增加中
    status_torque_decreasing,                   // (正常状态)力矩减少中
    status_torque_hold,                         // (正常状态)力矩保持
    status_velocity_increasing,                 // (正常状态)速度增加中
    status_velocity_decreasing,                 // (正常状态)速度减少中
    status_velocity_hold,                       // (正常状态)速度保持
    status_position_increasing,                 // (正常状态)位置增加中
    status_position_decreasing,                 // (正常状态)位置减少中
    status_position_hold,                       // (正常状态)位置保持

    status_overvoltage = 101,                   // (错误状态)过压
    status_undervoltage,                        // (错误状态)欠压
    status_overcurrent_moment,                  // (错误状态)瞬态过流
    status_overcurrent_stable,                  // (错误状态)稳态过流
    status_overtemp_motor,                      // (错误状态)电机过温
    status_undertemp_motor,                     // (错误状态)电机过冷
    status_overtemp_driver,                     // (错误状态)功率器件过温
    status_undertemp_driver,                    // (错误状态)功率器件过冷
    status_overtemp_pcb,                        // (错误状态)驱动器过温
    status_undertemp_pcb,                       // (错误状态)驱动器过冷
    status_fault0,                              // (错误状态)未知错误0
    status_fault1,                              // (错误状态)未知错误1
    status_fault2                               // (错误状态)未知错误2
}__LocalStatus;

typedef enum{
    state_standby = 0,                          // (standby)待机
    state_ready,                                // (ready)准备
    state_initializing,                         // (initializing)开始初始
    state_gotoinit0_angle,                      // (angle)角度初始
    state_gotoinit1_current,                    // (current)电流初始
    state_com,                                  // (com)正常
    state_fault                                 // (fault)错误
}__LocalState;

typedef enum{
    event_zero = 0,                             // (自动事件)零事件

    event_ready,                                // (手动)用户触发准备
    event_initializing,                         // (手动)用户触发初始化
    event_success,                              // (手动)用户触发初始化
    event_standby0,                             // (手动)用户触发待机
    event_standby1,                             // (自动事件)待机事件

    event_torque_increasing,                    // (自动事件)力矩增加事件
    event_torque_decreasing,                    // (自动事件)力矩减少事件
    event_torque_hold,                          // (自动事件)力矩保持事件
    event_velocity_increasing,                  // (自动事件)速度增加事件
    event_velocity_decreasing,                  // (自动事件)速度减少事件
    event_velocity_hold,                        // (自动事件)速度保持事件
    event_position_increasing,                  // (自动事件)位置增加事件
    event_position_decreasing,                  // (自动事件)位置减少事件
    event_position_hold,                        // (自动事件)位置保持事件
    
    event_overvoltage,                          // (自动事件)过压事件
    event_undervoltage,                         // (自动事件)欠压事件
    event_overcurrent_moment,                   // (自动事件)瞬态过流事件
    event_overcurrent_stable,                   // (自动事件)稳态过流事件
    event_overtemp_motor,                       // (自动事件)电机过温事件
    event_undertemp_motor,                      // (自动事件)电机过冷事件
    event_overtemp_driver,                      // (自动事件)功率器件过温事件
    event_undertemp_driver,                     // (自动事件)功率器件过冷事件
    event_overtemp_pcb,                         // (自动事件)驱动器过温事件
    event_undertemp_pcb,                        // (自动事件)驱动器过冷事件
    event_fault0,                               // (手动)未知错误0
    event_fault1,                               // (手动)未知错误1
    event_fault2                                // (手动)未知错误2
}__LocalEvent;

typedef struct{
    SguanQ tor_real;                            // (实时输入数据)实际力矩
    SguanQ tor_target;                          // (实时输入数据)期望力矩
    SguanQ tor_interval;                        // (参数)浮动区间
    
    SguanQ vel_real;                            // (实时输入数据)实际速度
    SguanQ vel_target;                          // (实时输入数据)期望速度
    SguanQ vel_interval;                        // (参数)浮动区间
    
    SguanQ pos_real;                            // (实时输入数据)实际位置
    SguanQ pos_target;                          // (实时输入数据)期望位置
    SguanQ pos_interval;                        // (参数)浮动区间

    SguanQ vbus_real;                           // (实时输入数据)实际母线电压
    SguanQ vbus_max;                            // (参数)浮动上限
    SguanQ vbus_min;                            // (参数)浮动下限

    SguanQ is_real;                             // (实时输入数据)实际线峰值电压
    SguanQ is_moment_max;                       // (参数)瞬态上限
    SguanQ is_stable_max;                       // (参数)稳态上限
    uint32_t is_tick_delay;                     // (参数)稳态最大维持时间
    uint32_t is_tick_count;                     // (临时数据)计数器

    SguanQ temp_motor_real;                     // (实时输入数据)电机温度
    SguanQ temp_motor_max;                      // (参数)温度上限
    SguanQ temp_motor_min;                      // (参数)温度下限
    
    SguanQ temp_driver_real;                    // (实时输入数据)驱动功率器件温度
    SguanQ temp_driver_max;                     // (参数)温度上限
    SguanQ temp_driver_min;                     // (参数)温度下限
    
    SguanQ temp_pcb_real;                       // (实时输入数据)硬件电路板温度
    SguanQ temp_pcb_max;                        // (参数)温度上限
    SguanQ temp_pcb_min;                        // (参数)温度下限

    uint8_t fault_flag0;                        // (实时输入数据)未知错误0
    uint8_t fault_flag1;                        // (实时输入数据)未知错误1
    uint8_t fault_flag2;                        // (实时输入数据)未知错误2

    uint32_t fault_tick_delay;                  // (参数)错误最大维持时间

    // (分割线)---------------------------------------------------------------------
    uint32_t tick_run;                          // (实时输入数据)实时运行时间
    uint32_t tick_count;                        // (临时数据)计数器
    
    uint8_t mode_flag;                          // (固定输入参数)电机运行模式
}__LocalData;

typedef struct{
    uint8_t status_now;                         // (now)现在状态机
    uint8_t event_now;                          // (now)当前状态机可触发的事件
    uint8_t status_future;                      // (future)下一时刻状态机
}__LocalTab;

// ================================================================================
// 3.全局结构体及函数(声明)
typedef struct{
    __LocalStatus status;                       // (status)现在状态
    __LocalStatus last;                         // (last)过去状态
    __LocalState state;                         // (state)状况标志位
    __LocalData data;                           // (data)实时数据

    uint8_t run_flag;                           // (run_flag)手动状态切换
    uint8_t error_code;                         // (error_code)错误码记录
}MotorStatus;

void motorstatus_init(MotorStatus *motorstatus);
void motorstatus_loop(MotorStatus *motorstatus);


#endif // SGUAN_MOTORSTATUS_H

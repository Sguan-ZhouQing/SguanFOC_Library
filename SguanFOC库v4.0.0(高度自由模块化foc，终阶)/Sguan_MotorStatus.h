#ifndef __SGUAN_MOTORSTATUS_H
#define __SGUAN_MOTORSTATUS_H

/* SguanFOC配置文件声明 */
#include "Sguan_Config.h"

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

    status_overvoltage,                         // (错误状态)过压
    status_undervoltage,                        // (错误状态)欠压
    status_overcurrent,                         // (错误状态)过流
    status_overtemp_motor,                      // (错误状态)电机过温
    status_undertemp_motor,                     // (错误状态)电机过冷
    status_overtemp_driver,                     // (错误状态)功率器件过温
    status_undertemp_driver,                    // (错误状态)功率器件过冷
    status_overtemp_pcb,                        // (错误状态)驱动器过温
    status_undertemp_pcb,                       // (错误状态)驱动器过冷
    status_stuck,                               // (错误状态)运行卡死
    status_phase_loss,                          // (错误状态)缺相[暂无，过流保护已可预防]
    status_circuit_short,                       // (错误状态)短路[暂无，过流保护已可预防]
    status_fault                                // (错误状态)未知错误
}HandleStatus;

typedef enum{
    state_standby = 0,                          // (standby)待机
    state_ready,                                // (ready)准备
    state_initializing,                         // (initializing)开始初始
    state_gotoinit0_angle,                      // (angle)角度初始
    state_gotoinit1_current,                    // (current)电流初始
    state_com,                                  // (com)正常
    state_fault                                 // (fault)错误
}HandleState;

typedef enum{
    event_zero = 0,                             // (自动事件)零事件

    event_ready,                                // (手动)用户触发准备
    event_initializing,                         // (手动)用户触发初始化
    event_success,                              // (手动)用户触发初始化
    event_standby0,                             // (手动)用户触发待机
    event_standby1,                             // (自动事件)待机事件

    event_torque_increasing = 11,               // (自动事件)力矩增加事件
    event_torque_decreasing,                    // (自动事件)力矩减少事件
    event_torque_hold,                          // (自动事件)力矩保持事件
    event_velocity_increasing = 21,             // (自动事件)速度增加事件
    event_velocity_decreasing,                  // (自动事件)速度减少事件
    event_velocity_hold,                        // (自动事件)速度保持事件
    event_position_increasing = 31,             // (自动事件)位置增加事件
    event_position_decreasing,                  // (自动事件)位置减少事件
    event_position_hold,                        // (自动事件)位置保持事件
    
    event_overvoltage = 81,                     // (自动事件)过压事件
    event_undervoltage,                         // (自动事件)欠压事件
    event_overcurrent,                          // (自动事件)过流事件
    event_overtemp_motor,                       // (自动事件)电机过温事件
    event_undertemp_motor,                      // (自动事件)电机过冷事件
    event_overtemp_driver,                      // (自动事件)功率器件过温事件
    event_undertemp_driver,                     // (自动事件)功率器件过冷事件
    event_overtemp_pcb,                         // (自动事件)驱动器过温事件
    event_undertemp_pcb,                        // (自动事件)驱动器过冷事件
    event_stuck = 91,                           // (自动事件)运行卡死事件
    event_phase_loss,                           // (自动事件)缺相事件[暂无，过流保护已可预防]
    event_circuit_short,                        // (自动事件)短路事件[暂无，过流保护已可预防]
    event_fault                                 // (手动)未知错误
}HandleEvent;

typedef struct{
    SguanQ tor_real;                            // (实时输入数据)
    SguanQ tor_target;                          // (实时输入数据)
    SguanQ tor_scope;                           // (参数)
    
    SguanQ vel_real;                            // (实时输入数据)
    SguanQ vel_target;                          // (实时输入数据)
    SguanQ vel_scope;                           // (参数)
    
    SguanQ pos_real;                            // (实时输入数据)
    SguanQ pos_target;                          // (实时输入数据)
    SguanQ pos_scope;                           // (参数)

    SguanQ phase_ia;                            // (实时输入数据)
    SguanQ phase_ib;                            // (实时输入数据)
    SguanQ phase_ic;                            // (实时输入数据)

    SguanQ vbus_real;                           // (实时输入数据)
    SguanQ vbus_max;                            // (参数)
    SguanQ vbus_min;                            // (参数)

    SguanQ ibus_real;                           // (实时输入数据)
    SguanQ ibus_instant_max;                    // (参数)
    SguanQ ibus_stable_max;                     // (参数)
    uint32_t ibus_tick_delay;                   // (参数)
    uint32_t ibus_tick_run;                     // (临时数据)

    SguanQ temp_motor_real;                     // (实时输入数据)
    SguanQ temp_motor_max;                      // (参数)
    SguanQ temp_motor_min;                      // (参数)
    
    SguanQ temp_driver_real;                    // (实时输入数据)
    SguanQ temp_driver_max;                     // (参数)
    SguanQ temp_driver_min;                     // (参数)
    
    SguanQ temp_pcb_real;                       // (实时输入数据)
    SguanQ temp_pcb_max;                        // (参数)
    SguanQ temp_pcb_min;                        // (参数)

    uint8_t fault_flag;                         // (意外输入数据)

    uint32_t fault_tick_delay;                  // (参数)

    uint32_t stuck_tick_delay;                  // (参数)
    uint32_t stuck_tick_run;                    // (临时数据)

    // ======================================================
    uint8_t error_code;                         // (被动输出数据)
    uint32_t tick_run;                          // (实时输入数据)
    uint32_t tick_last;                         // (临时数据)
    
    uint8_t mode_flag;                          // (固定输入参数)
}HandleData;

typedef struct{
    uint8_t status_now;                         // ()
    uint8_t event_now;                          // ()
    uint8_t status_future;                      // ()
}MotorTab;

typedef struct{
    HandleStatus status;                        // ()
    HandleStatus last;                          // ()
    HandleState state;                          // ()
    HandleData data;                            // ()
}MotorStatus;

void motorstatus_loop(void *sguan);


#endif // SGUAN_MOTORSTATUS_H

#ifndef __SGUAN_MOTORSTATUS_H
#define __SGUAN_MOTORSTATUS_H

/* SguanFOC配置文件声明 */
#include "Sguan_Config.h"

typedef enum{
    status_standby = 0,                         // (standby)停机状态->待机

    status_ready,                               // (ready)初始化状态->准备
    status_initializing,                        // (initializing)初始化状态->开始初始
    status_gotoinit0_angle,                     // (angle)初始化状态->角度初始专项
    status_gotoinit1_current,                   // (current)初始化状态->电流初始化专项

    status_idle,                                // (idle)正常状态->空闲
    status_torque_increasing,                   // (torque)正常状态->增加中
    status_torque_decreasing,                   // (torque)正常状态->减少中
    status_torque_hold,                         // (torque)正常状态->保持
    status_velocity_increasing,                 // (velocity)正常状态->增加中
    status_velocity_decreasing,                 // (velocity)正常状态->减少中
    status_velocity_hold,                       // (velocity)正常状态->保持
    status_position_increasing,                 // (position)正常状态->增加中
    status_position_decreasing,                 // (position)正常状态->减少中
    status_position_hold,                       // (position)正常状态->保持

    status_overvoltage,                         // (overvoltage)错误状态->过压
    status_undervoltage,                        // (undervoltage)错误状态->欠压
    status_overcurrent,                         // (overcurrent)错误状态->过流
    status_overtemp_motor,                      // (overtemp_motor)错误状态->电机过温
    status_undertemp_motor,                     // (undertemp_motor)错误状态->电机过冷
    status_overtemp_driver,                     // (overtemp_driver)错误状态->功率器件过温
    status_undertemp_driver,                    // (undertemp_driver)错误状态->功率器件过冷
    status_overtemp_pcb,                        // (overtemp_pcb)错误状态->驱动器过温
    status_undertemp_pcb,                       // (undertemp_pcb)错误状态->驱动器过冷
    status_stuck,                               // (stuck)错误状态->运行卡死
    status_fault                                // (fault)错误状态->未知错误
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
    event_zero = 0,                             // (zero)
    event_ready,                                // (ready->手动)
    event_initializing,                         // (initializing->手动)
    event_success,                              // (success->手动)
    event_standby0,                             // (standby0->手动)
    event_standby1,                             // (standby1)

    event_torque_hold = 11,                     // (torque_hold)
    event_torque_increasing,                    // (torque_increasing)
    event_torque_decreasing,                    // (torque_decreasing)

    event_velocity_hold = 21,                   // (velocity_hold)
    event_velocity_increasing,                  // (velocity_increasing)
    event_velocity_decreasing,                  // (velocity_decreasing)

    event_position_hold = 31,                   // (position_hold)
    event_position_increasing,                  // (position_increasing)
    event_position_decreasing,                  // (position_decreasing)

    event_overvoltage = 81,                     // (overvoltage)
    event_undervoltage,                         // (undervoltage)
    event_overcurrent,                          // (overcurrent)
    event_overtemp_motor,                       // (overtemp_motor)
    event_undertemp_motor,                      // (undertemp_motor)
    event_overtemp_driver,                      // (overtemp_driver)
    event_undertemp_driver,                     // (undertemp_driver)
    event_overtemp_pcb,                         // (overtemp_pcb)
    event_undertemp_pcb,                        // (undertemp_pcb)

    event_stuck = 91,                           // (stuck)
    event_fault                                 // (fault->手动)
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

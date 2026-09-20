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
    event_ready,                                // (ready)
    event_initializing,                         // (initializing)
    event_standby0,                             // (standby0手动)
    event_standby1,                             // (standby1自动)

    event_torque_increasing = 11,               // (torque_increasing)
    event_torque_decreasing,                    // (torque_decreasing)
    event_torque_hold,                          // (torque_hold)

    event_velocity_increasing = 21,             // (velocity_increasing)
    event_velocity_decreasing,                  // (velocity_decreasing)
    event_velocity_hold,                        // (velocity_hold)

    event_position_increasing = 31,             // (position_increasing)
    event_position_decreasing,                  // (position_decreasing)
    event_position_hold,                        // (position_hold)

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
    event_fault                                 // (fault)
}HandleEvent;

typedef enum{
    action_null = 0,                            // ()

    action_standby_init,                        // ()
    action_ready_init,                          // ()
    action_initializing_init,                   // ()
    action_goinit0_angle_init,                  // ()
    action_goinit1_current_init,                // ()

    action_idle_init,                           // ()

    action_overvoltage_init,                    // ()
    action_undervoltage_init,                   // ()
    action_overcurrent_init,                    // ()
    action_overtemp_motor_init,                 // ()
    action_undertemp_motor_init,                // ()
    action_overtemp_driver_init,                // ()
    action_undertemp_driver_init,               // ()
    action_overtemp_pcb_init,                   // ()
    action_undertemp_pcb_init,                  // ()
    action_stuck_init,                          // ()
    action_fault_init,                          // ()
}HandleActionInit;

typedef enum{
    action_null = 0,                            // ()

    action_standby_loop,                        // ()
    action_ready_loop,                          // ()
    action_initial_loop,                        // ()
    
    action_increasing_loop,                     // ()
    action_decreasing_loop,                     // ()
    action_hold_loop,                           // ()

    action_overvoltage_loop,                    // ()
    action_undervoltage_loop,                   // ()
    action_overcurrent_loop,                    // ()
    action_overtemp_motor_loop,                 // ()
    action_undertemp_motor_loop,                // ()
    action_overtemp_driver_loop,                // ()
    action_undertemp_driver_loop,               // ()
    action_overtemp_pcb_loop,                   // ()
    action_undertemp_pcb_loop,                  // ()
    action_stuck_loop,                          // ()
    action_fault_loop,                          // ()
}HandleActionLoop;

typedef struct{
    SguanQ tor_real;                            // ()
    SguanQ tor_target;                          // ()
    SguanQ tor_scope;                           // ()
    
    SguanQ vel_real;                            // ()
    SguanQ vel_target;                          // ()
    SguanQ vel_scope;                           // ()
    
    SguanQ pos_real;                            // ()
    SguanQ pos_target;                          // ()
    SguanQ pos_scope;                           // ()

    SguanQ vbus_real;                           // ()
    SguanQ vbus_max;                            // ()
    SguanQ vbus_min;                            // ()

    SguanQ ibus_real;                           // ()
    SguanQ ibus_instant_max;                    // ()
    SguanQ ibus_stable_max;                     // ()
    SguanQ ibus_stable_time;                    // ()

    SguanQ temp_motor_real;                     // ()
    SguanQ temp_motor_max;                      // ()
    SguanQ temp_motor_min;                      // ()
    
    SguanQ temp_driver_real;                    // ()
    SguanQ temp_driver_max;                     // ()
    SguanQ temp_driver_min;                     // ()
    
    SguanQ temp_pcb_real;                       // ()
    SguanQ temp_pcb_max;                        // ()
    SguanQ temp_pcb_min;                        // ()

    uint8_t fault_flag;                         // ()

    SguanQ fault_delay_time;                    // ()
    SguanQ stuck_delay_time;                    // ()

    // ======================================================
    uint8_t mode_flag;                          // ()
    SguanQ run_time;                            // ()
    SguanQ error_code;                          // ()
}HandleData;

typedef struct{
    HandleStatus status_now;                    // ()
    HandleState state_now;                      // ()
    HandleEvent event_now;                      // ()
    void (*action_init)(void *);                // ()
    void (*action_loop)(void);                  // ()
    HandleStatus status_future;                 // ()
}MotorTab;

typedef struct{
    HandleStatus status;                        // ()
    HandleState state;                          // ()
    HandleStatus last;                          // ()
    HandleEvent event;                          // ()
    HandleData data;                            // ()
}MotorStatus;

void motorstatus_action_init11(MotorStatus *motorstatus);
void motorstatus_high_loop(MotorStatus *motorstatus);
void motorstatus_low_loop(MotorStatus *motorstatus);


#endif // SGUAN_MOTORSTATUS_H

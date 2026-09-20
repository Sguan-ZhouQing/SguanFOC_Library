#include "Sguan_MotorStatus.h"
/* SguanFOC配置文件声明 */
#include "UnitLib_Status.h"
#include "UserData_Driver.h"

/* 内部私有变量与函数 */
static void handle_null(void);

static void handle_null(void){
    // 空函数
}

static void handle_overvoltage_init(SguanFoc *sguanfoc);
static void handle_undervoltage_init(SguanFoc *sguanfoc);
static void handle_overcurrent_init(SguanFoc *sguanfoc);
static void handle_overtemp_motor_init(SguanFoc *sguanfoc);
static void handle_undertemp_motor_init(SguanFoc *sguanfoc);
static void handle_overtemp_driver_init(SguanFoc *sguanfoc);
static void handle_undertemp_driver_init(SguanFoc *sguanfoc);
static void handle_overtemp_pcb_init(SguanFoc *sguanfoc);
static void handle_undertemp_pcb_init(SguanFoc *sguanfoc);
static void handle_stuck_init(SguanFoc *sguanfoc);
static void handle_fault_init(SguanFoc *sguanfoc);


// ===================================================
static void handle_overvoltage_init(SguanFoc *sguanfoc){

}

static void handle_undervoltage_init(SguanFoc *sguanfoc){

}

static void handle_overcurrent_init(SguanFoc *sguanfoc){

}

static void handle_overtemp_motor_init(SguanFoc *sguanfoc){

}

static void handle_undertemp_motor_init(SguanFoc *sguanfoc){

}

static void handle_overtemp_driver_init(SguanFoc *sguanfoc){

}

static void handle_undertemp_driver_init(SguanFoc *sguanfoc){

}

static void handle_overtemp_pcb_init(SguanFoc *sguanfoc){

}

static void handle_undertemp_pcb_init(SguanFoc *sguanfoc){

}

static void handle_stuck_init(SguanFoc *sguanfoc){

}

static void handle_fault_init(SguanFoc *sguanfoc){

}





static const MotorTab motor_tab[] = {
    {status_standby,                state_standby,                  event_ready, 
    main_ready_init,                status_standby_loop,            status_ready},

    // ======================================================================================
    {status_ready,                  state_ready,                    event_initializing, 
    main_initializing_init,         status_ready_loop,              status_initializing},
    {status_ready,                  state_ready,                    event_standby0, 
    main_standby_init,              status_ready_loop,              status_standby},

    // ======================================================================================
    {status_initializing,           state_initializing,             event_zero, 
    main_goinit0_angle_init,        status_initial_loop,            status_gotoinit0_angle},
    {status_gotoinit0_angle,        state_gotoinit0_angle,          event_zero, 
    main_goinit1_current_init,      status_initial_loop,            status_gotoinit1_current},
    {status_gotoinit1_current,      state_gotoinit1_current,        event_zero, 
    main_idle_init,                 status_initial_loop,            status_idle},
    
    // =======================================================================================
    {status_idle,                   state_com,                      event_torque_increasing, 
    handle_null,                    status_increasing_loop,         status_torque_increasing},
    {status_idle,                   state_com,                      event_torque_decreasing, 
    handle_null,                    status_decreasing_loop,         status_torque_decreasing},
    {status_idle,                   state_com,                      event_torque_hold, 
    handle_null,                    status_hold_loop,               status_torque_hold},

    {status_idle,                   state_com,                      event_velocity_increasing, 
    handle_null,                    status_increasing_loop,         status_velocity_increasing},
    {status_idle,                   state_com,                      event_velocity_decreasing, 
    handle_null,                    status_decreasing_loop,         status_velocity_decreasing},
    {status_idle,                   state_com,                      event_velocity_hold, 
    handle_null,                    status_hold_loop,               status_velocity_hold},

    {status_idle,                   state_com,                      event_position_increasing, 
    handle_null,                    status_increasing_loop,         status_position_increasing},
    {status_idle,                   state_com,                      event_position_decreasing, 
    handle_null,                    status_decreasing_loop,         status_position_decreasing},
    {status_idle,                   state_com,                      event_position_hold, 
    handle_null,                    status_hold_loop,               status_position_hold},
    
    {status_idle,                   state_com,                      event_standby0, 
    main_standby_init,              status_hold_loop,               status_standby},
    {status_idle,                   state_com,                      event_ready, 
    main_ready_init,                status_hold_loop,               status_ready},

    {status_idle,                   state_com,                      event_overvoltage, 
    handle_overvoltage_init,        status_hold_loop,               status_ready},
    {status_idle,                   state_com,                      event_undervoltage, 
    handle_undervoltage_init,       status_hold_loop,               status_ready},
    {status_idle,                   state_com,                      event_overcurrent, 
    handle_overcurrent_init,        status_hold_loop,               status_ready},
    {status_idle,                   state_com,                      event_overtemp_motor, 
    handle_overtemp_motor_init,     status_hold_loop,               status_ready},
    {status_idle,                   state_com,                      event_undertemp_motor, 
    handle_undertemp_motor_init,    status_hold_loop,               status_ready},
    {status_idle,                   state_com,                      event_overtemp_driver, 
    handle_overtemp_driver_init,    status_hold_loop,               status_ready},
    {status_idle,                   state_com,                      event_undertemp_driver, 
    handle_undertemp_driver_init,   status_hold_loop,               status_ready},
    {status_idle,                   state_com,                      event_overtemp_pcb, 
    handle_overtemp_pcb_init,       status_hold_loop,               status_ready},
    {status_idle,                   state_com,                      event_undertemp_pcb, 
    handle_undertemp_pcb_init,      status_hold_loop,               status_ready},

    {status_idle,                   state_com,                      event_stuck, 
    handle_stuck_init,              status_hold_loop,               status_ready},
    {status_idle,                   state_com,                      event_fault, 
    handle_fault_init,              status_hold_loop,               status_ready},

    // =======================================================================================

};

static MotorStatus (*const motorstatus_initial_init[])(MotorStatus *) = {
    main_standby_init,
    main_ready_init,
    main_initializing_init,
    main_goinit0_angle_init,
    main_goinit1_current_init,
    main_idle_init,
};
static void (*const motorstatus_action_loop[])(MotorStatus *) = {
    status_standby_loop,
    status_ready_loop,
    status_initial_loop,
    status_increasing_loop,
    status_decreasing_loop,
    status_hold_loop,

    status_overvoltage_loop,
    status_undervoltage_loop,
    status_overcurrent_loop,
    status_overtemp_motor_loop,
    status_undertemp_motor_loop,
    status_overtemp_driver_loop,
    status_undertemp_driver_loop,
    status_overtemp_pcb_loop,
    status_undertemp_pcb_loop,
    status_stuck_loop,
    status_fault_loop
};

static MotorStatus motorstatus_initial_init(MotorStatus *motorstatus){

}


void motorstatus_high_loop(MotorStatus *motorstatus){
    switch (motorstatus->status){
    case status_standby:

        break;
    case status_ready:

        break;
    case status_initializing:

        break;
    case status_gotoinit0_angle:

        break;
    case status_gotoinit1_current:

        break;
    case status_idle:

        break;
    case status_torque_increasing:

        break;
    case status_torque_decreasing:

        break;
    case status_torque_hold:

        break;
    case status_velocity_increasing:

        break;
    case status_velocity_decreasing:

        break;
    case status_velocity_hold:

        break;
    case status_position_increasing:

        break;
    case status_position_decreasing:

        break;
    case status_position_hold:

        break;
    case status_overvoltage:

        break;
    case status_overcurrent:

        break;
    case status_overtemp_motor:

        break;
    case status_undertemp_motor:

        break;
    case status_overtemp_driver:

        break;
    case status_undertemp_driver:

        break;
    case status_overtemp_pcb:

        break;
    case status_undertemp_pcb:

        break;
    case status_stuck:

        break;
    case status_fault:

        break;
    
    default:
        break;
    }


}


void motorstatus_low_loop(MotorStatus *motorstatus){

}



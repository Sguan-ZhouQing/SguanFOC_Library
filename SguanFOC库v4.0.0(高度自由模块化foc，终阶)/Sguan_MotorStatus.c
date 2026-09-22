#include "Sguan_MotorStatus.h"
/* SguanFOC配置文件声明 */
#include "UnitLib_Status.h"

/* 内部宏定义设计 */
#define STATUS(x) ((uint8_t)(x))
#define EVENT(x)  ((uint8_t)(x))

#define MODE_TORQUE     0x00
#define MODE_VELOCITY   0x01
#define MODE_POSITION   0x02

/* 内部私有变量与函数 */
static void motorstatus_null0(SguanFoc *sguan);
static void motorstatus_null1(void);
static void motorstatus_idle_init(SguanFoc *sguan);
static void motorstatus_overvoltage_init(SguanFoc *sguan);
static void motorstatus_undervoltage_init(SguanFoc *sguan);
static void motorstatus_overcurrent_init(SguanFoc *sguan);
static void motorstatus_overtemp_motor_init(SguanFoc *sguan);
static void motorstatus_undertemp_motor_init(SguanFoc *sguan);
static void motorstatus_overtemp_driver_init(SguanFoc *sguan);
static void motorstatus_undertemp_driver_init(SguanFoc *sguan);
static void motorstatus_overtemp_pcb_init(SguanFoc *sguan);
static void motorstatus_undertemp_pcb_init(SguanFoc *sguan);
static void motorstatus_stuck_init(SguanFoc *sguan);
static void motorstatus_fault_init(SguanFoc *sguan);
static HandleEvent motorstatus_event_get(uint8_t run);
// ===================================================
static void motorstatus_null0(SguanFoc *sguan){
    // 空函数
}

static void motorstatus_null1(void){
    // 空函数
}

static void motorstatus_idle_init(SguanFoc *sguan){
    sguan->motorstatus.data.error_code = 0x00;
    main_standby_init(sguan);
}

static void motorstatus_overvoltage_init(SguanFoc *sguan){
    sguan->motorstatus.data.error_code = 0x01;
    main_standby_init(sguan);
}

static void motorstatus_undervoltage_init(SguanFoc *sguan){
    sguan->motorstatus.data.error_code = 0x02;
    main_standby_init(sguan);
}

static void motorstatus_overcurrent_init(SguanFoc *sguan){
    sguan->motorstatus.data.error_code = 0x03;
    main_standby_init(sguan);
}

static void motorstatus_overtemp_motor_init(SguanFoc *sguan){
    sguan->motorstatus.data.error_code = 0x04;
    main_standby_init(sguan);
}

static void motorstatus_undertemp_motor_init(SguanFoc *sguan){
    sguan->motorstatus.data.error_code = 0x05;
    main_standby_init(sguan);
}

static void motorstatus_overtemp_driver_init(SguanFoc *sguan){
    sguan->motorstatus.data.error_code = 0x06;
    main_standby_init(sguan);
}

static void motorstatus_undertemp_driver_init(SguanFoc *sguan){
    sguan->motorstatus.data.error_code = 0x07;
    main_standby_init(sguan);
}

static void motorstatus_overtemp_pcb_init(SguanFoc *sguan){
    sguan->motorstatus.data.error_code = 0x08;
    main_standby_init(sguan);
}

static void motorstatus_undertemp_pcb_init(SguanFoc *sguan){
    sguan->motorstatus.data.error_code = 0x09;
    main_standby_init(sguan);
}

static void motorstatus_stuck_init(SguanFoc *sguan){
    sguan->motorstatus.data.error_code = 0x0A;
    main_standby_init(sguan);
}

static void motorstatus_fault_init(SguanFoc *sguan){
    sguan->motorstatus.data.error_code = 0x0B;
    main_standby_init(sguan);
}

static const MotorTab motor_tab[184] = {
    // 1.当前的状态机                    2.当前状态机可触发的事件              3.如果触发此事件，下一时刻的状态机
    {STATUS(status_standby),            EVENT(event_ready),                 STATUS(status_ready)},

    // ======================================================================================
    {STATUS(status_ready),              EVENT(event_initializing),          STATUS(status_initializing)},
    {STATUS(status_ready),              EVENT(event_standby0),              STATUS(status_standby)},

    // ======================================================================================
    {STATUS(status_initializing),       EVENT(event_success),                STATUS(status_gotoinit0_angle)},
    {STATUS(status_gotoinit0_angle),    EVENT(event_success),                STATUS(status_gotoinit1_current)},
    {STATUS(status_gotoinit1_current),  EVENT(event_success),                STATUS(status_idle)},
    
    // =======================================================================================
    {STATUS(status_idle),               EVENT(event_torque_hold),           STATUS(status_torque_hold)},
    {STATUS(status_idle),               EVENT(event_torque_increasing),     STATUS(status_torque_increasing)},
    {STATUS(status_idle),               EVENT(event_torque_decreasing),     STATUS(status_torque_decreasing)},

    {STATUS(status_idle),               EVENT(event_velocity_hold),         STATUS(status_velocity_hold)},
    {STATUS(status_idle),               EVENT(event_velocity_increasing),   STATUS(status_velocity_increasing)},
    {STATUS(status_idle),               EVENT(event_velocity_decreasing),   STATUS(status_velocity_decreasing)},

    {STATUS(status_idle),               EVENT(event_position_hold),         STATUS(status_position_hold)},
    {STATUS(status_idle),               EVENT(event_position_increasing),   STATUS(status_position_increasing)},
    {STATUS(status_idle),               EVENT(event_position_decreasing),   STATUS(status_position_decreasing)},
    
    {STATUS(status_idle),               EVENT(event_standby0),              STATUS(status_standby)},
    {STATUS(status_idle),               EVENT(event_ready),                 STATUS(status_ready)},

    {STATUS(status_idle),               EVENT(event_overvoltage),           STATUS(status_overvoltage)},
    {STATUS(status_idle),               EVENT(event_undervoltage),          STATUS(status_undervoltage)},
    {STATUS(status_idle),               EVENT(event_overcurrent),           STATUS(status_overcurrent)},
    {STATUS(status_idle),               EVENT(event_overtemp_motor),        STATUS(status_overtemp_motor)},
    {STATUS(status_idle),               EVENT(event_undertemp_motor),       STATUS(status_undertemp_motor)},
    {STATUS(status_idle),               EVENT(event_overtemp_driver),       STATUS(status_overtemp_driver)},
    {STATUS(status_idle),               EVENT(event_undertemp_driver),      STATUS(status_undertemp_driver)},
    {STATUS(status_idle),               EVENT(event_overtemp_pcb),          STATUS(status_overtemp_pcb)},
    {STATUS(status_idle),               EVENT(event_undertemp_pcb),         STATUS(status_undertemp_pcb)},

    {STATUS(status_idle),               EVENT(event_fault),                 STATUS(status_fault)},
    
    // =======================================================================================
    {STATUS(status_torque_increasing),  EVENT(event_torque_hold),           STATUS(status_torque_hold)},
    {STATUS(status_torque_increasing),  EVENT(event_torque_decreasing),     STATUS(status_torque_decreasing)},

    {STATUS(status_torque_increasing),  EVENT(event_standby0),              STATUS(status_standby)},
    {STATUS(status_torque_increasing),  EVENT(event_ready),                 STATUS(status_ready)},
    
    {STATUS(status_torque_increasing),  EVENT(event_overvoltage),           STATUS(status_overvoltage)},
    {STATUS(status_torque_increasing),  EVENT(event_undervoltage),          STATUS(status_undervoltage)},
    {STATUS(status_torque_increasing),  EVENT(event_overcurrent),           STATUS(status_overcurrent)},
    {STATUS(status_torque_increasing),  EVENT(event_overtemp_motor),        STATUS(status_overtemp_motor)},
    {STATUS(status_torque_increasing),  EVENT(event_undertemp_motor),       STATUS(status_undertemp_motor)},
    {STATUS(status_torque_increasing),  EVENT(event_overtemp_driver),       STATUS(status_overtemp_driver)},
    {STATUS(status_torque_increasing),  EVENT(event_undertemp_driver),      STATUS(status_undertemp_driver)},
    {STATUS(status_torque_increasing),  EVENT(event_overtemp_pcb),          STATUS(status_overtemp_pcb)},
    {STATUS(status_torque_increasing),  EVENT(event_undertemp_pcb),         STATUS(status_undertemp_pcb)},

    {STATUS(status_torque_increasing),  EVENT(event_stuck),                 STATUS(status_stuck)},
    {STATUS(status_torque_increasing),  EVENT(event_fault),                 STATUS(status_fault)},

    // =======================================================================================
    {STATUS(status_torque_decreasing),  EVENT(event_torque_hold),           STATUS(status_torque_hold)},
    {STATUS(status_torque_decreasing),  EVENT(event_torque_increasing),     STATUS(status_torque_increasing)},

    {STATUS(status_torque_decreasing),  EVENT(event_standby0),              STATUS(status_standby)},
    {STATUS(status_torque_decreasing),  EVENT(event_ready),                 STATUS(status_ready)},
    
    {STATUS(status_torque_decreasing),  EVENT(event_overvoltage),           STATUS(status_overvoltage)},
    {STATUS(status_torque_decreasing),  EVENT(event_undervoltage),          STATUS(status_undervoltage)},
    {STATUS(status_torque_decreasing),  EVENT(event_overcurrent),           STATUS(status_overcurrent)},
    {STATUS(status_torque_decreasing),  EVENT(event_overtemp_motor),        STATUS(status_overtemp_motor)},
    {STATUS(status_torque_decreasing),  EVENT(event_undertemp_motor),       STATUS(status_undertemp_motor)},
    {STATUS(status_torque_decreasing),  EVENT(event_overtemp_driver),       STATUS(status_overtemp_driver)},
    {STATUS(status_torque_decreasing),  EVENT(event_undertemp_driver),      STATUS(status_undertemp_driver)},
    {STATUS(status_torque_decreasing),  EVENT(event_overtemp_pcb),          STATUS(status_overtemp_pcb)},
    {STATUS(status_torque_decreasing),  EVENT(event_undertemp_pcb),         STATUS(status_undertemp_pcb)},

    {STATUS(status_torque_decreasing),  EVENT(event_stuck),                 STATUS(status_stuck)},
    {STATUS(status_torque_decreasing),  EVENT(event_fault),                 STATUS(status_fault)},

    // =======================================================================================
    {STATUS(status_torque_hold),        EVENT(event_torque_increasing),     STATUS(status_torque_increasing)},
    {STATUS(status_torque_hold),        EVENT(event_torque_decreasing),     STATUS(status_torque_decreasing)},

    {STATUS(status_torque_hold),        EVENT(event_standby0),              STATUS(status_standby)},
    {STATUS(status_torque_hold),        EVENT(event_ready),                 STATUS(status_ready)},
    
    {STATUS(status_torque_hold),        EVENT(event_overvoltage),           STATUS(status_overvoltage)},
    {STATUS(status_torque_hold),        EVENT(event_undervoltage),          STATUS(status_undervoltage)},
    {STATUS(status_torque_hold),        EVENT(event_overcurrent),           STATUS(status_overcurrent)},
    {STATUS(status_torque_hold),        EVENT(event_overtemp_motor),        STATUS(status_overtemp_motor)},
    {STATUS(status_torque_hold),        EVENT(event_undertemp_motor),       STATUS(status_undertemp_motor)},
    {STATUS(status_torque_hold),        EVENT(event_overtemp_driver),       STATUS(status_overtemp_driver)},
    {STATUS(status_torque_hold),        EVENT(event_undertemp_driver),      STATUS(status_undertemp_driver)},
    {STATUS(status_torque_hold),        EVENT(event_overtemp_pcb),          STATUS(status_overtemp_pcb)},
    {STATUS(status_torque_hold),        EVENT(event_undertemp_pcb),         STATUS(status_undertemp_pcb)},

    {STATUS(status_torque_hold),        EVENT(event_stuck),                 STATUS(status_stuck)},
    {STATUS(status_torque_hold),        EVENT(event_fault),                 STATUS(status_fault)},

    // =======================================================================================
    {STATUS(status_velocity_increasing),EVENT(event_velocity_hold),         STATUS(status_velocity_hold)},
    {STATUS(status_velocity_increasing),EVENT(event_velocity_decreasing),   STATUS(status_velocity_decreasing)},

    {STATUS(status_velocity_increasing),EVENT(event_standby0),              STATUS(status_standby)},
    {STATUS(status_velocity_increasing),EVENT(event_ready),                 STATUS(status_ready)},
    
    {STATUS(status_velocity_increasing),EVENT(event_overvoltage),           STATUS(status_overvoltage)},
    {STATUS(status_velocity_increasing),EVENT(event_undervoltage),          STATUS(status_undervoltage)},
    {STATUS(status_velocity_increasing),EVENT(event_overcurrent),           STATUS(status_overcurrent)},
    {STATUS(status_velocity_increasing),EVENT(event_overtemp_motor),        STATUS(status_overtemp_motor)},
    {STATUS(status_velocity_increasing),EVENT(event_undertemp_motor),       STATUS(status_undertemp_motor)},
    {STATUS(status_velocity_increasing),EVENT(event_overtemp_driver),       STATUS(status_overtemp_driver)},
    {STATUS(status_velocity_increasing),EVENT(event_undertemp_driver),      STATUS(status_undertemp_driver)},
    {STATUS(status_velocity_increasing),EVENT(event_overtemp_pcb),          STATUS(status_overtemp_pcb)},
    {STATUS(status_velocity_increasing),EVENT(event_undertemp_pcb),         STATUS(status_undertemp_pcb)},

    {STATUS(status_velocity_increasing),EVENT(event_stuck),                 STATUS(status_stuck)},
    {STATUS(status_velocity_increasing),EVENT(event_fault),                 STATUS(status_fault)},

    // =======================================================================================
    {STATUS(status_velocity_decreasing),EVENT(event_velocity_hold),         STATUS(status_velocity_hold)},
    {STATUS(status_velocity_decreasing),EVENT(event_velocity_increasing),   STATUS(status_velocity_increasing)},

    {STATUS(status_velocity_decreasing),EVENT(event_standby0),              STATUS(status_standby)},
    {STATUS(status_velocity_decreasing),EVENT(event_ready),                 STATUS(status_ready)},
    
    {STATUS(status_velocity_decreasing),EVENT(event_overvoltage),           STATUS(status_overvoltage)},
    {STATUS(status_velocity_decreasing),EVENT(event_undervoltage),          STATUS(status_undervoltage)},
    {STATUS(status_velocity_decreasing),EVENT(event_overcurrent),           STATUS(status_overcurrent)},
    {STATUS(status_velocity_decreasing),EVENT(event_overtemp_motor),        STATUS(status_overtemp_motor)},
    {STATUS(status_velocity_decreasing),EVENT(event_undertemp_motor),       STATUS(status_undertemp_motor)},
    {STATUS(status_velocity_decreasing),EVENT(event_overtemp_driver),       STATUS(status_overtemp_driver)},
    {STATUS(status_velocity_decreasing),EVENT(event_undertemp_driver),      STATUS(status_undertemp_driver)},
    {STATUS(status_velocity_decreasing),EVENT(event_overtemp_pcb),          STATUS(status_overtemp_pcb)},
    {STATUS(status_velocity_decreasing),EVENT(event_undertemp_pcb),         STATUS(status_undertemp_pcb)},

    {STATUS(status_velocity_decreasing),EVENT(event_stuck),                 STATUS(status_stuck)},
    {STATUS(status_velocity_decreasing),EVENT(event_fault),                 STATUS(status_fault)},

    // =======================================================================================
    {STATUS(status_velocity_hold),      EVENT(event_velocity_increasing),   STATUS(status_velocity_increasing)},
    {STATUS(status_velocity_hold),      EVENT(event_velocity_decreasing),   STATUS(status_velocity_decreasing)},

    {STATUS(status_velocity_hold),      EVENT(event_standby0),              STATUS(status_standby)},
    {STATUS(status_velocity_hold),      EVENT(event_ready),                 STATUS(status_ready)},
    
    {STATUS(status_velocity_hold),      EVENT(event_overvoltage),           STATUS(status_overvoltage)},
    {STATUS(status_velocity_hold),      EVENT(event_undervoltage),          STATUS(status_undervoltage)},
    {STATUS(status_velocity_hold),      EVENT(event_overcurrent),           STATUS(status_overcurrent)},
    {STATUS(status_velocity_hold),      EVENT(event_overtemp_motor),        STATUS(status_overtemp_motor)},
    {STATUS(status_velocity_hold),      EVENT(event_undertemp_motor),       STATUS(status_undertemp_motor)},
    {STATUS(status_velocity_hold),      EVENT(event_overtemp_driver),       STATUS(status_overtemp_driver)},
    {STATUS(status_velocity_hold),      EVENT(event_undertemp_driver),      STATUS(status_undertemp_driver)},
    {STATUS(status_velocity_hold),      EVENT(event_overtemp_pcb),          STATUS(status_overtemp_pcb)},
    {STATUS(status_velocity_hold),      EVENT(event_undertemp_pcb),         STATUS(status_undertemp_pcb)},

    {STATUS(status_velocity_hold),      EVENT(event_stuck),                 STATUS(status_stuck)},
    {STATUS(status_velocity_hold),      EVENT(event_fault),                 STATUS(status_fault)},

    // =======================================================================================
    {STATUS(status_position_increasing),EVENT(event_position_hold),         STATUS(status_position_hold)},
    {STATUS(status_position_increasing),EVENT(event_position_decreasing),   STATUS(status_position_decreasing)},

    {STATUS(status_position_increasing),EVENT(event_standby0),              STATUS(status_standby)},
    {STATUS(status_position_increasing),EVENT(event_ready),                 STATUS(status_ready)},
    
    {STATUS(status_position_increasing),EVENT(event_overvoltage),           STATUS(status_overvoltage)},
    {STATUS(status_position_increasing),EVENT(event_undervoltage),          STATUS(status_undervoltage)},
    {STATUS(status_position_increasing),EVENT(event_overcurrent),           STATUS(status_overcurrent)},
    {STATUS(status_position_increasing),EVENT(event_overtemp_motor),        STATUS(status_overtemp_motor)},
    {STATUS(status_position_increasing),EVENT(event_undertemp_motor),       STATUS(status_undertemp_motor)},
    {STATUS(status_position_increasing),EVENT(event_overtemp_driver),       STATUS(status_overtemp_driver)},
    {STATUS(status_position_increasing),EVENT(event_undertemp_driver),      STATUS(status_undertemp_driver)},
    {STATUS(status_position_increasing),EVENT(event_overtemp_pcb),          STATUS(status_overtemp_pcb)},
    {STATUS(status_position_increasing),EVENT(event_undertemp_pcb),         STATUS(status_undertemp_pcb)},

    {STATUS(status_position_increasing),EVENT(event_stuck),                 STATUS(status_stuck)},
    {STATUS(status_position_increasing),EVENT(event_fault),                 STATUS(status_fault)},

    // =======================================================================================
    {STATUS(status_position_decreasing),EVENT(event_position_hold),         STATUS(status_position_hold)},
    {STATUS(status_position_decreasing),EVENT(event_position_increasing),   STATUS(status_position_increasing)},

    {STATUS(status_position_decreasing),EVENT(event_standby0),              STATUS(status_standby)},
    {STATUS(status_position_decreasing),EVENT(event_ready),                 STATUS(status_ready)},
    
    {STATUS(status_position_decreasing),EVENT(event_overvoltage),           STATUS(status_overvoltage)},
    {STATUS(status_position_decreasing),EVENT(event_undervoltage),          STATUS(status_undervoltage)},
    {STATUS(status_position_decreasing),EVENT(event_overcurrent),           STATUS(status_overcurrent)},
    {STATUS(status_position_decreasing),EVENT(event_overtemp_motor),        STATUS(status_overtemp_motor)},
    {STATUS(status_position_decreasing),EVENT(event_undertemp_motor),       STATUS(status_undertemp_motor)},
    {STATUS(status_position_decreasing),EVENT(event_overtemp_driver),       STATUS(status_overtemp_driver)},
    {STATUS(status_position_decreasing),EVENT(event_undertemp_driver),      STATUS(status_undertemp_driver)},
    {STATUS(status_position_decreasing),EVENT(event_overtemp_pcb),          STATUS(status_overtemp_pcb)},
    {STATUS(status_position_decreasing),EVENT(event_undertemp_pcb),         STATUS(status_undertemp_pcb)},

    {STATUS(status_position_decreasing),EVENT(event_stuck),                 STATUS(status_stuck)},
    {STATUS(status_position_decreasing),EVENT(event_fault),                 STATUS(status_fault)},

    // =======================================================================================
    {STATUS(status_position_hold),      EVENT(event_position_increasing),   STATUS(status_position_increasing)},
    {STATUS(status_position_hold),      EVENT(event_position_decreasing),   STATUS(status_position_decreasing)},

    {STATUS(status_position_hold),      EVENT(event_standby0),              STATUS(status_standby)},
    {STATUS(status_position_hold),      EVENT(event_ready),                 STATUS(status_ready)},
    
    {STATUS(status_position_hold),      EVENT(event_overvoltage),           STATUS(status_overvoltage)},
    {STATUS(status_position_hold),      EVENT(event_undervoltage),          STATUS(status_undervoltage)},
    {STATUS(status_position_hold),      EVENT(event_overcurrent),           STATUS(status_overcurrent)},
    {STATUS(status_position_hold),      EVENT(event_overtemp_motor),        STATUS(status_overtemp_motor)},
    {STATUS(status_position_hold),      EVENT(event_undertemp_motor),       STATUS(status_undertemp_motor)},
    {STATUS(status_position_hold),      EVENT(event_overtemp_driver),       STATUS(status_overtemp_driver)},
    {STATUS(status_position_hold),      EVENT(event_undertemp_driver),      STATUS(status_undertemp_driver)},
    {STATUS(status_position_hold),      EVENT(event_overtemp_pcb),          STATUS(status_overtemp_pcb)},
    {STATUS(status_position_hold),      EVENT(event_undertemp_pcb),         STATUS(status_undertemp_pcb)},

    {STATUS(status_position_hold),      EVENT(event_stuck),                 STATUS(status_stuck)},
    {STATUS(status_position_hold),      EVENT(event_fault),                 STATUS(status_fault)},

    // =======================================================================================
    {STATUS(status_overvoltage),        EVENT(event_standby0),              STATUS(status_standby)},
    {STATUS(status_overvoltage),        EVENT(event_standby1),              STATUS(status_standby)},

    {STATUS(status_undervoltage),       EVENT(event_standby0),              STATUS(status_standby)},
    {STATUS(status_undervoltage),       EVENT(event_standby1),              STATUS(status_standby)},

    {STATUS(status_overcurrent),        EVENT(event_standby0),              STATUS(status_standby)},
    {STATUS(status_overcurrent),        EVENT(event_standby1),              STATUS(status_standby)},

    {STATUS(status_overtemp_motor),     EVENT(event_standby0),              STATUS(status_standby)},
    {STATUS(status_overtemp_motor),     EVENT(event_standby1),              STATUS(status_standby)},

    {STATUS(status_undertemp_motor),    EVENT(event_standby0),              STATUS(status_standby)},
    {STATUS(status_undertemp_motor),    EVENT(event_standby1),              STATUS(status_standby)},

    {STATUS(status_overtemp_driver),    EVENT(event_standby0),              STATUS(status_standby)},
    {STATUS(status_overtemp_driver),    EVENT(event_standby1),              STATUS(status_standby)},

    {STATUS(status_undertemp_driver),   EVENT(event_standby0),              STATUS(status_standby)},
    {STATUS(status_undertemp_driver),   EVENT(event_standby1),              STATUS(status_standby)},

    {STATUS(status_overtemp_pcb),       EVENT(event_standby0),              STATUS(status_standby)},
    {STATUS(status_overtemp_pcb),       EVENT(event_standby1),              STATUS(status_standby)},

    {STATUS(status_undertemp_pcb),      EVENT(event_standby0),              STATUS(status_standby)},
    {STATUS(status_undertemp_pcb),      EVENT(event_standby1),              STATUS(status_standby)},

    {STATUS(status_stuck),              EVENT(event_standby0),              STATUS(status_standby)},
    {STATUS(status_stuck),              EVENT(event_standby1),              STATUS(status_standby)},

    {STATUS(status_fault),              EVENT(event_standby0),              STATUS(status_standby)},
    {STATUS(status_fault),              EVENT(event_standby1),              STATUS(status_standby)}
};

static void (*const motorstatus_initial_init[])(SguanFoc *) = {
    main_standby_init,

    main_ready_init,
    main_initializing_init,
    main_goinit0_angle_init,
    main_goinit1_current_init,
    
    motorstatus_idle_init,
    motorstatus_null0,
    motorstatus_null0,
    motorstatus_null0,
    motorstatus_null0,
    motorstatus_null0,
    motorstatus_null0,
    motorstatus_null0,
    motorstatus_null0,
    motorstatus_null0,

    motorstatus_overvoltage_init,
    motorstatus_undervoltage_init,
    motorstatus_overcurrent_init,
    motorstatus_overtemp_motor_init,
    motorstatus_undertemp_motor_init,
    motorstatus_overtemp_driver_init,
    motorstatus_undertemp_driver_init,
    motorstatus_overtemp_pcb_init,
    motorstatus_undertemp_pcb_init,
    motorstatus_stuck_init,
    motorstatus_fault_init
};

static void (*const motorstatus_action_loop[])(void) = {
    status_standby_loop,

    status_ready_loop,
    status_initial_loop,
    status_initial_loop,
    status_initial_loop,

    motorstatus_null1,
    status_increasing_loop,
    status_decreasing_loop,
    status_hold_loop,
    status_increasing_loop,
    status_decreasing_loop,
    status_hold_loop,
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

static HandleEvent motorstatus_event_get(uint8_t run){
    switch (run){
    case RUN_READY: // 准备触发
        return event_ready;
    case RUN_INITIALIZING: // 初始化触发
        return event_initializing;
    case RUN_STANDBY: // 待机触发
        return event_standby0;
    case RUN_FAULT: // 错误触发
        return event_fault;
    case RUN_SUCCESS: // [特殊]初始化success触发
        return event_success;
    
    default:
        break;
    }
    return event_zero;
}

void motorstatus_loop(void *sguan){
    // ==================================================================
    // 1.结构体类型转换(SguanFoc结构体)
    SguanFoc *p = (SguanFoc *)sguan;
 
    // ==================================================================
    // 2.遍历状态转换表，给触发事件的条件
    for (uint16_t i = 0; i < 184; i++){
        if (motor_tab[i].status_now == p->motorstatus.status){
            switch ((HandleEvent)motor_tab[i].event_now){
                case event_ready:{
                    // 此处有触发event的代码
                    if (motorstatus_event_get(p->run_flag) == 
                        (HandleEvent)motor_tab[i].event_now){ // 手动触发
                        p->run_flag = 0;

                        p->motorstatus.status = 
                            (HandleStatus)motor_tab[i].status_future;
                    }
                    break;
                }
                case event_initializing:{
                    // 此处有触发event的代码
                    if (motorstatus_event_get(p->run_flag) == 
                        (HandleEvent)motor_tab[i].event_now){ // 手动触发
                        p->run_flag = 0;
                            
                        p->motorstatus.status = 
                            (HandleStatus)motor_tab[i].status_future;
                    }
                    break;
                }
                case event_success:{
                    // 此处有触发event的代码
                    if (motorstatus_event_get(p->run_flag) == 
                        (HandleEvent)motor_tab[i].event_now){ // 手动触发
                        p->run_flag = 0;
                            
                        p->motorstatus.status = 
                            (HandleStatus)motor_tab[i].status_future;
                    }
                    break;
                }
                case event_standby0:{
                    // 此处有触发event的代码
                    if (motorstatus_event_get(p->run_flag) == 
                        (HandleStatus)motor_tab[i].event_now){ // 手动触发
                        p->run_flag = 0;
                            
                        p->motorstatus.status = 
                            (HandleStatus)motor_tab[i].status_future;
                    }
                    break;
                }
                case event_standby1:{
                    // 此处有触发event的代码
                    uint32_t tick = 
                        p->motorstatus.data.tick_run - 
                        p->motorstatus.data.tick_last;

                    if (tick >= p->motorstatus.data.fault_tick_delay){ // 自动触发
                            
                        p->motorstatus.status = 
                            (HandleStatus)motor_tab[i].status_future;
                    }
                    break;
                }
                case event_torque_hold:{
                    // 此处有触发event的代码
                    if (p->motorstatus.data.mode_flag == MODE_TORQUE){
                        SguanQ tor_target_min = iqmath_sub(
                            p->motorstatus.data.tor_target, 
                            p->motorstatus.data.tor_scope);
                        SguanQ tor_target_max = iqmath_add(
                            p->motorstatus.data.tor_target, 
                            p->motorstatus.data.tor_scope);
                        SguanQ tor_real_abs = iqmath_abs(
                            p->motorstatus.data.tor_real);
        
                        if ((tor_real_abs >= tor_target_min) && 
                            (tor_real_abs <= tor_target_max)){ // 自动触发
                            p->motorstatus.data.stuck_tick_run = 
                                p->motorstatus.data.tick_run;

                            p->motorstatus.status = 
                                (HandleStatus)motor_tab[i].status_future;
                        }
                    }
                    break;
                }
                case event_torque_increasing:{
                    // 此处有触发event的代码
                    if (p->motorstatus.data.mode_flag == MODE_TORQUE){                    
                        SguanQ tor_target_min = iqmath_sub(
                            p->motorstatus.data.tor_target, 
                            p->motorstatus.data.tor_scope);
                        SguanQ tor_real_abs = iqmath_abs(
                            p->motorstatus.data.tor_real);
        
                        if (tor_real_abs <= tor_target_min){ // 自动触发
                            
                            p->motorstatus.status = 
                                (HandleStatus)motor_tab[i].status_future;
                        }
                    }
                    break;
                }
                case event_torque_decreasing:{
                    // 此处有触发event的代码
                    if (p->motorstatus.data.mode_flag == MODE_TORQUE){
                        SguanQ tor_target_max = iqmath_add(
                            p->motorstatus.data.tor_target, 
                            p->motorstatus.data.tor_scope);
                        SguanQ tor_real_abs = iqmath_abs(
                            p->motorstatus.data.tor_real);
        
                        if (tor_real_abs >= tor_target_max){ // 自动触发
                            
                            p->motorstatus.status = 
                                (HandleStatus)motor_tab[i].status_future;
                        }
                    }
                    break;
                }
                case event_velocity_hold:{
                    // 此处有触发event的代码
                    if (p->motorstatus.data.mode_flag == MODE_VELOCITY){
                        SguanQ vel_target_min = iqmath_sub(
                            p->motorstatus.data.vel_target, 
                            p->motorstatus.data.vel_scope);
                        SguanQ vel_target_max = iqmath_add(
                            p->motorstatus.data.vel_target, 
                            p->motorstatus.data.vel_scope);
                        SguanQ vel_real_abs = iqmath_abs(
                            p->motorstatus.data.vel_real);
        
                        if ((vel_real_abs >= vel_target_min) && 
                            (vel_real_abs <= vel_target_max)){ // 自动触发
                            p->motorstatus.data.stuck_tick_run = 
                                p->motorstatus.data.tick_run;
                            
                            p->motorstatus.status = 
                                (HandleStatus)motor_tab[i].status_future;
                        }
                    }
                    break;
                }
                case event_velocity_increasing:{
                    // 此处有触发event的代码
                    if (p->motorstatus.data.mode_flag == MODE_VELOCITY){
                        SguanQ vel_target_min = iqmath_sub(
                            p->motorstatus.data.vel_target, 
                            p->motorstatus.data.vel_scope);
                        SguanQ vel_real_abs = iqmath_abs(
                            p->motorstatus.data.vel_real);
        
                        if (vel_real_abs <= vel_target_min){ // 自动触发
                            
                            p->motorstatus.status = 
                                (HandleStatus)motor_tab[i].status_future;
                        }
                    }
                    break;
                }
                case event_velocity_decreasing:{
                    // 此处有触发event的代码
                    if (p->motorstatus.data.mode_flag == MODE_VELOCITY){
                        SguanQ vel_target_max = iqmath_add(
                            p->motorstatus.data.vel_target, 
                            p->motorstatus.data.vel_scope);
                        SguanQ vel_real_abs = iqmath_abs(
                            p->motorstatus.data.vel_real);
        
                        if (vel_real_abs >= vel_target_max){ // 自动触发
                            
                            p->motorstatus.status = 
                                (HandleStatus)motor_tab[i].status_future;
                        }
                    }
                    break;
                }
                case event_position_hold:{
                    // 此处有触发event的代码
                    if (p->motorstatus.data.mode_flag == MODE_POSITION){
                        SguanQ pos_target_min = iqmath_sub(
                            p->motorstatus.data.pos_target, 
                            p->motorstatus.data.pos_scope);
                        SguanQ pos_target_max = iqmath_add(
                            p->motorstatus.data.pos_target, 
                            p->motorstatus.data.pos_scope);
                        SguanQ pos_real_abs = iqmath_abs(
                            p->motorstatus.data.pos_real);
        
                        if ((pos_real_abs >= pos_target_min) && 
                            (pos_real_abs <= pos_target_max)){ // 自动触发
                            p->motorstatus.data.stuck_tick_run = 
                                p->motorstatus.data.tick_run;
                            
                            p->motorstatus.status = 
                                (HandleStatus)motor_tab[i].status_future;
                        }
                    }
                    break;
                }
                case event_position_increasing:{
                    // 此处有触发event的代码
                    if (p->motorstatus.data.mode_flag == MODE_POSITION){
                        SguanQ pos_target_min = iqmath_sub(
                            p->motorstatus.data.pos_target, 
                            p->motorstatus.data.pos_scope);
                        SguanQ pos_real_abs = iqmath_abs(
                            p->motorstatus.data.pos_real);
        
                        if (pos_real_abs <= pos_target_min){ // 自动触发
        
                            
                            p->motorstatus.status = 
                                (HandleStatus)motor_tab[i].status_future;
                        }
                    }
                    break;
                }
                case event_position_decreasing:{
                    // 此处有触发event的代码
                    if (p->motorstatus.data.mode_flag == MODE_POSITION){
                        SguanQ pos_target_max = iqmath_add(
                            p->motorstatus.data.pos_target, 
                            p->motorstatus.data.pos_scope);
                        SguanQ pos_real_abs = iqmath_abs(
                            p->motorstatus.data.pos_real);
        
                        if (pos_real_abs >= pos_target_max){ // 自动触发
                            
                            p->motorstatus.status = 
                                (HandleStatus)motor_tab[i].status_future;
                        }
                    }
                    break;
                }
                case event_overvoltage:{
                    // 此处有触发event的代码
                    if (p->motorstatus.data.vbus_real >= 
                        p->motorstatus.data.vbus_max){ // 自动触发
                            
                        p->motorstatus.status = 
                            (HandleStatus)motor_tab[i].status_future;
                    }
                    break;
                }
                case event_undervoltage:{
                    // 此处有触发event的代码
                    if (p->motorstatus.data.vbus_real <= 
                        p->motorstatus.data.vbus_min){ // 自动触发
                            
                        p->motorstatus.status = 
                            (HandleStatus)motor_tab[i].status_future;
                    }
                    break;
                }
                case event_overcurrent:{
                    // 此处有触发event的代码
                    uint32_t tick = 0;
                    if (p->motorstatus.data.ibus_real <= 
                        p->motorstatus.data.ibus_stable_max){

                        p->motorstatus.data.ibus_tick_run = 
                            p->motorstatus.data.tick_run; 
                    }
                    else{
                        tick = iqmath_sub(
                            p->motorstatus.data.tick_run, 
                            p->motorstatus.data.ibus_tick_run);
                    }

                    if ((p->motorstatus.data.ibus_real >= 
                        p->motorstatus.data.ibus_instant_max) || 
                        // 过流保护“或”条件判断
                        (tick >= p->motorstatus.data.ibus_tick_delay)){ // 自动触发
                            
                        p->motorstatus.status = 
                            (HandleStatus)motor_tab[i].status_future;
                    }
                    break;
                }
                case event_overtemp_motor:{
                    // 此处有触发event的代码
                    if (p->motorstatus.data.temp_motor_real >= 
                        p->motorstatus.data.temp_motor_max){ // 自动触发
                            
                        p->motorstatus.status = 
                            (HandleStatus)motor_tab[i].status_future;
                    }
                    break;
                }
                case event_undertemp_motor:{
                    // 此处有触发event的代码
                    if (p->motorstatus.data.temp_motor_real <= 
                        p->motorstatus.data.temp_motor_min){ // 自动触发
                            
                        p->motorstatus.status = 
                            (HandleStatus)motor_tab[i].status_future;
                    }
                    break;
                }
                case event_overtemp_driver:{
                    // 此处有触发event的代码
                    if (p->motorstatus.data.temp_driver_real >= 
                        p->motorstatus.data.temp_driver_max){ // 自动触发
                            
                        p->motorstatus.status = 
                            (HandleStatus)motor_tab[i].status_future;
                    }
                    break;
                }
                case event_undertemp_driver:{
                    // 此处有触发event的代码
                    if (p->motorstatus.data.temp_driver_real <= 
                        p->motorstatus.data.temp_driver_min){ // 自动触发
                            
                        p->motorstatus.status = 
                            (HandleStatus)motor_tab[i].status_future;
                    }
                    break;
                }
                case event_overtemp_pcb:{
                    // 此处有触发event的代码
                    if (p->motorstatus.data.temp_pcb_real >= 
                        p->motorstatus.data.temp_pcb_max){ // 自动触发
                            
                        p->motorstatus.status = 
                            (HandleStatus)motor_tab[i].status_future;
                    }
                    break;
                }
                case event_undertemp_pcb:{
                    // 此处有触发event的代码
                    if (p->motorstatus.data.temp_pcb_real <= 
                        p->motorstatus.data.temp_pcb_min){ // 自动触发
                            
                        p->motorstatus.status = 
                            (HandleStatus)motor_tab[i].status_future;
                    }
                    break;
                }
                case event_stuck:{
                    // 此处有触发event的代码
                    uint32_t tick = 
                        p->motorstatus.data.tick_run - 
                        p->motorstatus.data.stuck_tick_run;

                    if (tick >= p->motorstatus.data.stuck_tick_delay){ // 自动触发
                            
                        p->motorstatus.status = 
                            (HandleStatus)motor_tab[i].status_future;
                    }
                    break;
                }
                case event_fault:{
                    // 此处有触发event的代码
                    if (motorstatus_event_get(p->run_flag) == 
                        (HandleStatus)motor_tab[i].event_now){ // 手动触发
                        p->run_flag = 0;
                            
                        p->motorstatus.status = 
                            (HandleStatus)motor_tab[i].status_future;
                    }
                    break;
                }

                default:
                    break;
            }
        }
    }

    // ==================================================================
    // 3.运行init和loop函数
    if (p->motorstatus.last != p->motorstatus.status){
        // 状态机发送变化，记录变化时刻的tick时间
        p->motorstatus.data.tick_last = p->motorstatus.data.tick_run;

        // 仅在状态机变化开始，初始化一次init函数并更新last状态机
        motorstatus_initial_init[p->motorstatus.status](p);
        p->motorstatus.last = p->motorstatus.status;
    }
    motorstatus_action_loop[p->motorstatus.status]();
}



/* ===================================================================
 * 文件:     Sguan_MotorStatus.c
 * 项目:     SguanFOC_v4 —— 开源 FOC 电机控制框架
 * 说明:     1.Sguan固件库_电机状态
 * 链接:     https://github.com/Sguan-ZhouQing/SguanFOC_Library
 * 版权:     Copyright 2026 星必尘Sguan <3464647102@qq.com>
 * 许可证:   Apache License, Version 2.0
 * SPDX:     SPDX-License-Identifier: Apache-2.0
 *
 * 许可说明:
 *   [EN] Licensed under the Apache License, Version 2.0. You may use,
 *        copy, modify, and distribute this software, including for
 *        commercial purposes, provided you retain this copyright
 *        notice and the terms of the License.
 *   [中文] 本软件依据 Apache License 2.0 授权：可自由使用、复制、修改、
 *        分发（含商业用途），但必须保留本版权声明与许可证条款。
 *   [DE] Lizenziert unter der Apache License, Version 2.0. Nutzung,
 *        Vervielfaeltigung, Aenderung und Weitergabe (auch kommerziell)
 *        sind erlaubt, sofern dieser Urheberrechtshinweis und die
 *        Lizenzbedingungen erhalten bleiben.
 *   [JA] Apache License, Version 2.0 に基づき許諾されます。使用・複製・
 *        改変・再配布（商用を含む）が可能ですが、本著作権表示と
 *        ライセンス条項を保持する必要があります。
 * 完整许可文本: 参见仓库根目录下的 LICENSE-APACHE 文件。
 * =================================================================== */
#include "Sguan_MotorStatus.h"

/* SguanFOC配置文件声明 */
#include "Zone_Driver.h"

// ================================================================================
// 1.内部宏定义和结构体设计
// (状态事件强转宏)
#define STATUS(x) ((uint8_t)(x))    // uint8_t强制类型转换
#define EVENT(x)  ((uint8_t)(x))    // uint8_t强制类型转换
#define RUN(x)    ((uint8_t)(x))    // uint8_t强制类型转换

// (触发事件的枚举量)
typedef enum{
    event_ready = 0,                                // (手动)用户触发准备
    event_initializing,                             // (手动)用户触发初始化
    event_next,                                     // (手动)用户触发初始化
    event_standby0,                                 // (手动)用户触发待机
    event_standby1,                                 // (自动事件)待机事件

    event_value_holding,                            // (自动事件)数值保持事件
    event_value_above,                              // (自动事件)数值高事件
    event_value_below,                              // (自动事件)数值低事件
    
    event_overvoltage,                              // (自动事件)过压事件
    event_undervoltage,                             // (自动事件)欠压事件
    event_overcurrent_moment,                       // (自动事件)瞬态过流事件
    event_overcurrent_stable,                       // (自动事件)稳态过流事件
    event_overtemp_motor,                           // (自动事件)电机过温事件
    event_undertemp_motor,                          // (自动事件)电机过冷事件
    event_overtemp_driver,                          // (自动事件)功率器件过温事件
    event_undertemp_driver,                         // (自动事件)功率器件过冷事件
    event_overtemp_pcb,                             // (自动事件)驱动器过温事件
    event_undertemp_pcb,                            // (自动事件)驱动器过冷事件
    event_fault0,                                   // (手动)未知错误0
    event_fault1,                                   // (手动)未知错误1
    event_fault2                                    // (手动)未知错误2
}__LocalEvent;

// (状态转移表结构体)
typedef struct{
    uint8_t status_now;             // (now)现在状态机
    uint8_t event_now;              // (now)当前状态机可触发的事件
    uint8_t status_future;          // (future)下一时刻状态机
}__LocalTab;

// (for循环跳出的枚举量)
typedef enum {
    __LOCAL_CONTINUE = 0,           // (状态机，继续for循环)
    __LOCAL_BREAK_LOOP              // (状态机，跳出for循环)
} __LocalLoopFlag;

// (分割线)-------------------------------------------------------------------------
// (空函数与前置声明)
static void __local_null_loop(uint8_t id);
static void __local_null_init(MotorStatus *motorstatus);
static void __local_null_loop(uint8_t id){}
static void __local_null_init(MotorStatus *motorstatus){}
static __LocalStatus __local_next_init(const __LocalTab *tab);
static void __local_event_manual0(
    MotorStatus *motorstatus, 
    uint8_t run_flag, 
    const __LocalTab *tab);
static void __local_event_manual1(
    MotorStatus *motorstatus, 
    uint8_t *fault_flag, 
    const __LocalTab *tab);
static void __local_standby_init(MotorStatus *motorstatus);
static void __local_ready_init(MotorStatus *motorstatus);
static void __local_initializing_init(MotorStatus *motorstatus);
static void __local_start0_current_init(MotorStatus *motorstatus);
static void __local_start1_angle_init(MotorStatus *motorstatus);
static void __local_com_init(MotorStatus *motorstatus);
static void __local_fault_init(MotorStatus *motorstatus);
static __LocalLoopFlag __local_event_handler(
    MotorStatus *motorstatus,
    const __LocalTab *tab,
    uint32_t standby1_temp,
    SguanQ value_temp_max,
    SguanQ value_temp_min,
    uint32_t overcurrent_stable_temp);

// (数组元素个数宏)
#define __LOCAL_ARRAY_SIZE(a) (sizeof(a)/sizeof((a)[0]))

// (状态动作封装宏)
#define __LOCAL_ACTION(name) \
    static void __local_##name(uint8_t id){ driver_ops[id].name(); }

// (生成状态动作函数)
__LOCAL_ACTION(standby_loop)
__LOCAL_ACTION(ready_loop)
__LOCAL_ACTION(initial_loop)
__LOCAL_ACTION(holding_loop)
__LOCAL_ACTION(above_loop)
__LOCAL_ACTION(below_loop)
__LOCAL_ACTION(overvoltage_loop)
__LOCAL_ACTION(undervoltage_loop)
__LOCAL_ACTION(overcurrent_moment_loop)
__LOCAL_ACTION(overcurrent_stable_loop)
__LOCAL_ACTION(overtemp_motor_loop)
__LOCAL_ACTION(undertemp_motor_loop)
__LOCAL_ACTION(overtemp_driver_loop)
__LOCAL_ACTION(undertemp_driver_loop)
__LOCAL_ACTION(overtemp_pcb_loop)
__LOCAL_ACTION(undertemp_pcb_loop)
__LOCAL_ACTION(fault0_loop)
__LOCAL_ACTION(fault1_loop)
__LOCAL_ACTION(fault2_loop)

// ================================================================================
// 2.状态转移表和函数表
// (状态机转移总表)
static const __LocalTab __local_motor_tab[] = {
    // ①.当前的状态机                   ②.当前状态机可触发的事件          ③.如果触发此事件，下一时刻的状态机
    {STATUS(status_standby),           EVENT(event_ready),             STATUS(status_ready)},

    // (分割线)---------------------------------------------------------------------
    {STATUS(status_ready),             EVENT(event_initializing),      STATUS(status_initializing)},
    {STATUS(status_ready),             EVENT(event_standby0),          STATUS(status_standby)},

    // (分割线)---------------------------------------------------------------------
    {STATUS(status_initializing),      EVENT(event_next),              STATUS(status_start0_current)},
    {STATUS(status_start0_current),    EVENT(event_next),              STATUS(status_start1_angle)},
    {STATUS(status_start1_angle),      EVENT(event_next),              STATUS(status_idle)},
    
    // (分割线)---------------------------------------------------------------------
    {STATUS(status_idle),              EVENT(event_value_holding),     STATUS(status_value_holding)},
    {STATUS(status_idle),              EVENT(event_value_above),       STATUS(status_value_above)},
    {STATUS(status_idle),              EVENT(event_value_below),       STATUS(status_value_below)},
    
    {STATUS(status_idle),              EVENT(event_standby0),          STATUS(status_standby)},
    {STATUS(status_idle),              EVENT(event_ready),             STATUS(status_ready)},

    {STATUS(status_idle),              EVENT(event_overvoltage),       STATUS(status_overvoltage)},
    {STATUS(status_idle),              EVENT(event_undervoltage),      STATUS(status_undervoltage)},
    {STATUS(status_idle),              EVENT(event_overcurrent_moment),STATUS(status_overcurrent_moment)},
    {STATUS(status_idle),              EVENT(event_overcurrent_stable),STATUS(status_overcurrent_stable)},
    {STATUS(status_idle),              EVENT(event_overtemp_motor),    STATUS(status_overtemp_motor)},
    {STATUS(status_idle),              EVENT(event_undertemp_motor),   STATUS(status_undertemp_motor)},
    {STATUS(status_idle),              EVENT(event_overtemp_driver),   STATUS(status_overtemp_driver)},
    {STATUS(status_idle),              EVENT(event_undertemp_driver),  STATUS(status_undertemp_driver)},
    {STATUS(status_idle),              EVENT(event_overtemp_pcb),      STATUS(status_overtemp_pcb)},
    {STATUS(status_idle),              EVENT(event_undertemp_pcb),     STATUS(status_undertemp_pcb)},
    {STATUS(status_idle),              EVENT(event_fault0),            STATUS(status_fault0)},
    {STATUS(status_idle),              EVENT(event_fault1),            STATUS(status_fault1)},
    {STATUS(status_idle),              EVENT(event_fault2),            STATUS(status_fault2)},
    
    // (分割线)---------------------------------------------------------------------
    {STATUS(status_value_holding),     EVENT(event_value_above),       STATUS(status_value_above)},
    {STATUS(status_value_holding),     EVENT(event_value_below),       STATUS(status_value_below)},

    {STATUS(status_value_holding),     EVENT(event_standby0),          STATUS(status_standby)},
    {STATUS(status_value_holding),     EVENT(event_ready),             STATUS(status_ready)},
    
    {STATUS(status_value_holding),     EVENT(event_overvoltage),       STATUS(status_overvoltage)},
    {STATUS(status_value_holding),     EVENT(event_undervoltage),      STATUS(status_undervoltage)},
    {STATUS(status_value_holding),     EVENT(event_overcurrent_moment),STATUS(status_overcurrent_moment)},
    {STATUS(status_value_holding),     EVENT(event_overcurrent_stable),STATUS(status_overcurrent_stable)},
    {STATUS(status_value_holding),     EVENT(event_overtemp_motor),    STATUS(status_overtemp_motor)},
    {STATUS(status_value_holding),     EVENT(event_undertemp_motor),   STATUS(status_undertemp_motor)},
    {STATUS(status_value_holding),     EVENT(event_overtemp_driver),   STATUS(status_overtemp_driver)},
    {STATUS(status_value_holding),     EVENT(event_undertemp_driver),  STATUS(status_undertemp_driver)},
    {STATUS(status_value_holding),     EVENT(event_overtemp_pcb),      STATUS(status_overtemp_pcb)},
    {STATUS(status_value_holding),     EVENT(event_undertemp_pcb),     STATUS(status_undertemp_pcb)},
    {STATUS(status_value_holding),     EVENT(event_fault0),            STATUS(status_fault0)},
    {STATUS(status_value_holding),     EVENT(event_fault1),            STATUS(status_fault1)},
    {STATUS(status_value_holding),     EVENT(event_fault2),            STATUS(status_fault2)},

    // (分割线)---------------------------------------------------------------------
    {STATUS(status_value_above),       EVENT(event_value_holding),     STATUS(status_value_holding)},
    {STATUS(status_value_above),       EVENT(event_value_below),       STATUS(status_value_below)},

    {STATUS(status_value_above),       EVENT(event_standby0),          STATUS(status_standby)},
    {STATUS(status_value_above),       EVENT(event_ready),             STATUS(status_ready)},
    
    {STATUS(status_value_above),       EVENT(event_overvoltage),       STATUS(status_overvoltage)},
    {STATUS(status_value_above),       EVENT(event_undervoltage),      STATUS(status_undervoltage)},
    {STATUS(status_value_above),       EVENT(event_overcurrent_moment),STATUS(status_overcurrent_moment)},
    {STATUS(status_value_above),       EVENT(event_overcurrent_stable),STATUS(status_overcurrent_stable)},
    {STATUS(status_value_above),       EVENT(event_overtemp_motor),    STATUS(status_overtemp_motor)},
    {STATUS(status_value_above),       EVENT(event_undertemp_motor),   STATUS(status_undertemp_motor)},
    {STATUS(status_value_above),       EVENT(event_overtemp_driver),   STATUS(status_overtemp_driver)},
    {STATUS(status_value_above),       EVENT(event_undertemp_driver),  STATUS(status_undertemp_driver)},
    {STATUS(status_value_above),       EVENT(event_overtemp_pcb),      STATUS(status_overtemp_pcb)},
    {STATUS(status_value_above),       EVENT(event_undertemp_pcb),     STATUS(status_undertemp_pcb)},
    {STATUS(status_value_above),       EVENT(event_fault0),            STATUS(status_fault0)},
    {STATUS(status_value_above),       EVENT(event_fault1),            STATUS(status_fault1)},
    {STATUS(status_value_above),       EVENT(event_fault2),            STATUS(status_fault2)},

    // (分割线)---------------------------------------------------------------------
    {STATUS(status_value_below),       EVENT(event_value_holding),     STATUS(status_value_holding)},
    {STATUS(status_value_below),       EVENT(event_value_above),       STATUS(status_value_above)},

    {STATUS(status_value_below),       EVENT(event_standby0),          STATUS(status_standby)},
    {STATUS(status_value_below),       EVENT(event_ready),             STATUS(status_ready)},
    
    {STATUS(status_value_below),       EVENT(event_overvoltage),       STATUS(status_overvoltage)},
    {STATUS(status_value_below),       EVENT(event_undervoltage),      STATUS(status_undervoltage)},
    {STATUS(status_value_below),       EVENT(event_overcurrent_moment),STATUS(status_overcurrent_moment)},
    {STATUS(status_value_below),       EVENT(event_overcurrent_stable),STATUS(status_overcurrent_stable)},
    {STATUS(status_value_below),       EVENT(event_overtemp_motor),    STATUS(status_overtemp_motor)},
    {STATUS(status_value_below),       EVENT(event_undertemp_motor),   STATUS(status_undertemp_motor)},
    {STATUS(status_value_below),       EVENT(event_overtemp_driver),   STATUS(status_overtemp_driver)},
    {STATUS(status_value_below),       EVENT(event_undertemp_driver),  STATUS(status_undertemp_driver)},
    {STATUS(status_value_below),       EVENT(event_overtemp_pcb),      STATUS(status_overtemp_pcb)},
    {STATUS(status_value_below),       EVENT(event_undertemp_pcb),     STATUS(status_undertemp_pcb)},
    {STATUS(status_value_below),       EVENT(event_fault0),            STATUS(status_fault0)},
    {STATUS(status_value_below),       EVENT(event_fault1),            STATUS(status_fault1)},
    {STATUS(status_value_below),       EVENT(event_fault2),            STATUS(status_fault2)},

    // (分割线)---------------------------------------------------------------------
    {STATUS(status_overvoltage),       EVENT(event_standby0),          STATUS(status_standby)},
    {STATUS(status_overvoltage),       EVENT(event_standby1),          STATUS(status_standby)},

    {STATUS(status_undervoltage),      EVENT(event_standby0),          STATUS(status_standby)},
    {STATUS(status_undervoltage),      EVENT(event_standby1),          STATUS(status_standby)},

    {STATUS(status_overcurrent_moment),EVENT(event_standby0),          STATUS(status_standby)},
    {STATUS(status_overcurrent_moment),EVENT(event_standby1),          STATUS(status_standby)},

    {STATUS(status_overcurrent_stable),EVENT(event_standby0),          STATUS(status_standby)},
    {STATUS(status_overcurrent_stable),EVENT(event_standby1),          STATUS(status_standby)},

    {STATUS(status_overtemp_motor),    EVENT(event_standby0),          STATUS(status_standby)},
    {STATUS(status_overtemp_motor),    EVENT(event_standby1),          STATUS(status_standby)},

    {STATUS(status_undertemp_motor),   EVENT(event_standby0),          STATUS(status_standby)},
    {STATUS(status_undertemp_motor),   EVENT(event_standby1),          STATUS(status_standby)},

    {STATUS(status_overtemp_driver),   EVENT(event_standby0),          STATUS(status_standby)},
    {STATUS(status_overtemp_driver),   EVENT(event_standby1),          STATUS(status_standby)},

    {STATUS(status_undertemp_driver),  EVENT(event_standby0),          STATUS(status_standby)},
    {STATUS(status_undertemp_driver),  EVENT(event_standby1),          STATUS(status_standby)},

    {STATUS(status_overtemp_pcb),      EVENT(event_standby0),          STATUS(status_standby)},
    {STATUS(status_overtemp_pcb),      EVENT(event_standby1),          STATUS(status_standby)},

    {STATUS(status_undertemp_pcb),     EVENT(event_standby0),          STATUS(status_standby)},
    {STATUS(status_undertemp_pcb),     EVENT(event_standby1),          STATUS(status_standby)},

    {STATUS(status_fault0),            EVENT(event_standby0),          STATUS(status_standby)},
    {STATUS(status_fault0),            EVENT(event_standby1),          STATUS(status_standby)},

    {STATUS(status_fault1),            EVENT(event_standby0),          STATUS(status_standby)},
    {STATUS(status_fault1),            EVENT(event_standby1),          STATUS(status_standby)},

    {STATUS(status_fault2),            EVENT(event_standby0),          STATUS(status_standby)},
    {STATUS(status_fault2),            EVENT(event_standby1),          STATUS(status_standby)}
};

// (状态初始化函数表)
static void (*const __local_action_init[])(MotorStatus *) = {
    __local_standby_init,

    __local_ready_init,
    __local_initializing_init,
    __local_start0_current_init,
    __local_start1_angle_init,
    
    __local_com_init,
    __local_null_init,
    __local_null_init,
    __local_null_init,

    __local_fault_init,
    __local_fault_init,
    __local_fault_init,
    __local_fault_init,
    __local_fault_init,
    __local_fault_init,
    __local_fault_init,
    __local_fault_init,
    __local_fault_init,
    __local_fault_init,
    __local_fault_init,
    __local_fault_init,
    __local_fault_init
};

// (状态循环函数表)
static void (*const __local_action_loop[])(uint8_t id) = {
    __local_standby_loop,

    __local_ready_loop,
    __local_initial_loop,
    __local_initial_loop,
    __local_initial_loop,

    __local_null_loop,
    __local_holding_loop,
    __local_above_loop,
    __local_below_loop,

    __local_overvoltage_loop,
    __local_undervoltage_loop,
    __local_overcurrent_moment_loop,
    __local_overcurrent_stable_loop,
    __local_overtemp_motor_loop,
    __local_undertemp_motor_loop,
    __local_overtemp_driver_loop,
    __local_undertemp_driver_loop,
    __local_overtemp_pcb_loop,
    __local_undertemp_pcb_loop,
    __local_fault0_loop,
    __local_fault1_loop,
    __local_fault2_loop
};

// ================================================================================
// 3.局部静态函数__local实现
// (获取下一状态)
static __LocalStatus __local_next_init(const __LocalTab *tab){
    return (__LocalStatus)tab->status_future;
}

// (手动事件跳转)
static void __local_event_manual0(
    MotorStatus *motorstatus, 
    uint8_t run_flag, 
    const __LocalTab *tab){
    if (motorstatus->data.system_run_flag == run_flag){
        motorstatus->status = __local_next_init(tab);
        motorstatus->data.system_run_flag = 0;
    }
}

// (故障事件跳转)
static void __local_event_manual1(
    MotorStatus *motorstatus, 
    uint8_t *fault_flag, 
    const __LocalTab *tab){
    if (*fault_flag){
        motorstatus->status = __local_next_init(tab);
        (*fault_flag) = 0;
    }
}

// (分割线)-------------------------------------------------------------------------
// (待机状态初始化)
static void __local_standby_init(MotorStatus *motorstatus){
    driver_ops[motorstatus->params.motor_id].stop();
    motorstatus->state = state_standby;
}

// (就绪状态初始化)
static void __local_ready_init(MotorStatus *motorstatus){
    driver_ops[motorstatus->params.motor_id].start();
    motorstatus->state = state_ready;
}

// (初始化状态入口)
static void __local_initializing_init(MotorStatus *motorstatus){
    motorstatus->state = state_initializing;
}

// (电流启动初始化)
static void __local_start0_current_init(MotorStatus *motorstatus){
    motorstatus->state = state_start0_current;
}

// (角度启动初始化)
static void __local_start1_angle_init(MotorStatus *motorstatus){
    motorstatus->state = state_start1_angle;
}

// (通讯状态初始化)
static void __local_com_init(MotorStatus *motorstatus){
    motorstatus->error_code = STATUS(MOTORSTATUS_ERROR_ZERO);
    motorstatus->state = state_com;
}

// (故障状态初始化)
static void __local_fault_init(MotorStatus *motorstatus){
    driver_ops[motorstatus->params.motor_id].stop();
    motorstatus->error_code = (STATUS(motorstatus->status) - 8);
    motorstatus->state = state_fault;
}

// (分割线)-------------------------------------------------------------------------
// (局部函数for循环跳出处理)
static __LocalLoopFlag __local_event_handler(
    MotorStatus *motorstatus,
    const __LocalTab *tab,
    uint32_t standby1_temp,
    SguanQ value_temp_max,
    SguanQ value_temp_min,
    uint32_t overcurrent_stable_temp){
    switch ((__LocalEvent)tab->event_now) {
        case event_ready:
            // 手动触发
            __local_event_manual0(
                motorstatus, 
                RUN(MOTORSTATUS_RUN_READY), 
                tab);
            break;

        case event_initializing:
            // 手动触发
            __local_event_manual0(
                motorstatus, 
                RUN(MOTORSTATUS_RUN_INITIALIZING), 
                tab);
            break;

        case event_next:
            // 手动触发
            __local_event_manual0(
                motorstatus, 
                RUN(MOTORSTATUS_RUN_NEXT), 
                tab);
            break;

        case event_standby0:
            // 手动触发
            __local_event_manual0(
                motorstatus, 
                RUN(MOTORSTATUS_RUN_STANDBY), 
                tab);
            break;

        case event_standby1:
            // 自动event的事件
            if (standby1_temp >= 
                motorstatus->params.fault_tick_count) {
                motorstatus->status = __local_next_init(tab);
            }
            break;

        case event_value_holding:
            // 自动event的事件
            if ((motorstatus->data.value_real < value_temp_max) &&
                (motorstatus->data.value_real > value_temp_min)) {
                motorstatus->status = __local_next_init(tab);
            }
            break;

        case event_value_above:
            // 自动event的事件
            if (motorstatus->data.value_real >= value_temp_max) {
                motorstatus->status = __local_next_init(tab);
            }
            break;

        case event_value_below:
            // 自动event的事件
            if (motorstatus->data.value_real <= value_temp_min) {
                motorstatus->status = __local_next_init(tab);
            }
            break;

        case event_overvoltage:
            // 自动event的事件
            if (motorstatus->data.vbus_real >= 
                motorstatus->params.vbus_max) {
                motorstatus->status = __local_next_init(tab);
            }
            break;

        case event_undervoltage:
            // 自动event的事件
            if (motorstatus->data.vbus_real <= 
                motorstatus->params.vbus_min) {
                motorstatus->status = __local_next_init(tab);
            }
            break;

        case event_overcurrent_moment:
            // 自动event的事件
            if (motorstatus->data.is_real >= 
                motorstatus->params.is_moment_max) {
                motorstatus->status = __local_next_init(tab);
            }
            break;

        case event_overcurrent_stable:
            // 自动event的事件
            if ((motorstatus->data.is_real >= 
                motorstatus->params.is_stable_max) &&
                // (俩条件，需要同时满足)
                (overcurrent_stable_temp >= 
                motorstatus->params.is_tick_count)) {
                motorstatus->status = __local_next_init(tab);
            }
            break;

        case event_overtemp_motor:
            // 自动event的事件
            if (motorstatus->data.temp_motor_real >= 
                motorstatus->params.temp_motor_max) {
                motorstatus->status = __local_next_init(tab);
            }
            break;

        case event_undertemp_motor:
            // 自动event的事件
            if (motorstatus->data.temp_motor_real <= 
                motorstatus->params.temp_motor_min) {
                motorstatus->status = __local_next_init(tab);
            }
            break;

        case event_overtemp_driver:
            // 自动event的事件
            if (motorstatus->data.temp_driver_real >= 
                motorstatus->params.temp_driver_max) {
                motorstatus->status = __local_next_init(tab);
            }
            break;

        case event_undertemp_driver:
            // 自动event的事件
            if (motorstatus->data.temp_driver_real <= 
                motorstatus->params.temp_driver_min) {
                motorstatus->status = __local_next_init(tab);
            }
            break;

        case event_overtemp_pcb:
            // 自动event的事件
            if (motorstatus->data.temp_pcb_real >= 
                motorstatus->params.temp_pcb_max) {
                motorstatus->status = __local_next_init(tab);
            }
            break;

        case event_undertemp_pcb:
            // 自动event的事件
            if (motorstatus->data.temp_pcb_real <= 
                motorstatus->params.temp_pcb_min) {
                motorstatus->status = __local_next_init(tab);
            }
            break;

        case event_fault0:
            // 手动触发
            __local_event_manual1(
                motorstatus, 
                &motorstatus->data.fault_flag0, 
                tab);
            break;

        case event_fault1:
            // 手动触发
            __local_event_manual1(
                motorstatus, 
                &motorstatus->data.fault_flag1, 
                tab);
            break;

        case event_fault2:
            // 手动触发
            __local_event_manual1(
                motorstatus, 
                &motorstatus->data.fault_flag2, 
                tab);
            break;

        default:
            break;
    }

    if (motorstatus->last != motorstatus->status){
        // 仅在状态机变化开始，初始化一次init函数并更新last状态机
        __local_action_init[motorstatus->status](motorstatus);
        motorstatus->last = motorstatus->status;
        // 并跳出整个for循环
        return __LOCAL_BREAK_LOOP;
    }
    return __LOCAL_CONTINUE;
}

// ================================================================================
// 4.全局函数motorstatus实现
// (就绪标志置位)
void motorstatus_ready_init(MotorStatus *motorstatus){
    motorstatus->data.system_run_flag = RUN(MOTORSTATUS_RUN_READY);
}

// (初始化标志置位)
void motorstatus_initializing_init(MotorStatus *motorstatus){
    motorstatus->data.system_run_flag = RUN(MOTORSTATUS_RUN_INITIALIZING);
}

// (下一步标志置位)
void motorstatus_next_init(MotorStatus *motorstatus){
    motorstatus->data.system_run_flag = RUN(MOTORSTATUS_RUN_NEXT);
}

// (待机标志置位)
void motorstatus_standby_init(MotorStatus *motorstatus){
    motorstatus->data.system_run_flag = RUN(MOTORSTATUS_RUN_STANDBY);
}

// (状态机主循环)
void motorstatus_loop(MotorStatus *motorstatus){
    if (motorstatus->status < event_overvoltage){
        motorstatus->data.fault_tick_now = motorstatus->data.system_tick_now;
    }
    if (motorstatus->data.is_real < motorstatus->params.is_stable_max){
        motorstatus->data.is_tick_now = motorstatus->data.system_tick_now;
    }

    uint32_t standby1_temp = motorstatus->data.system_tick_now - 
        motorstatus->data.fault_tick_now;

    SguanQ value_temp_max = math_add(
        motorstatus->data.value_target, 
        motorstatus->params.value_interval);
    SguanQ value_temp_min = math_sub(
        motorstatus->data.value_target, 
        motorstatus->params.value_interval);

    uint32_t overcurrent_stable_temp = motorstatus->data.system_tick_now - 
        motorstatus->data.is_tick_now;

    // (分割线)---------------------------------------------------------------------
    for (uint16_t i = 0; i < __LOCAL_ARRAY_SIZE(__local_motor_tab); i++){
        if (__local_motor_tab[i].status_now == motorstatus->status){
            if (__local_event_handler(
                    motorstatus,
                    &__local_motor_tab[i],
                    standby1_temp,
                    value_temp_max,
                    value_temp_min,
                    overcurrent_stable_temp) == __LOCAL_BREAK_LOOP){
                break;
            }
        }
    }

    // (分割线)---------------------------------------------------------------------
    __local_action_loop[motorstatus->status](motorstatus->params.motor_id);
}

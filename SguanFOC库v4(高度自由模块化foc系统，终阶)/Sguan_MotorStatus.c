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
// ()
#define STATUS(x) ((uint8_t)(x))
#define EVENT(x)  ((uint8_t)(x))

// ()
static void __local_null_loop(uint8_t id);
static void __local_null_init(MotorStatus *motorstatus);
static void __local_null_loop(uint8_t id){}
static void __local_null_init(MotorStatus *motorstatus){}
static void __local_standby_init(MotorStatus *motorstatus);
static void __local_ready_init(MotorStatus *motorstatus);
static void __local_initializing_init(MotorStatus *motorstatus);
static void __local_start0_current_init(MotorStatus *motorstatus);
static void __local_start1_angle_init(MotorStatus *motorstatus);
static void __local_com_init(MotorStatus *motorstatus);
static void __local_fault_init(MotorStatus *motorstatus);

// ()
#define __LOCAL_ACTION(name) \
    static void __local_##name(uint8_t id){ driver_ops[id].name(); }

// ()
__LOCAL_ACTION(standby_loop)
__LOCAL_ACTION(ready_loop)
__LOCAL_ACTION(initial_loop)
__LOCAL_ACTION(increasing_loop)
__LOCAL_ACTION(decreasing_loop)
__LOCAL_ACTION(hold_loop)
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

// ()
typedef struct{
    uint8_t status_now;     // (now)现在状态机
    uint8_t event_now;      // (now)当前状态机可触发的事件
    uint8_t status_future;  // (future)下一时刻状态机
}__LocalTab;

// ================================================================================
// 2.状态转移表和函数表
// ()
static const __LocalTab __local_motor_tab[206] = {
    // ①.当前的状态机                    ②.当前状态机可触发的事件           ③.如果触发此事件，下一时刻的状态机
    {STATUS(status_standby),            EVENT(event_ready),              STATUS(status_ready)},

    // (分割线)---------------------------------------------------------------------
    {STATUS(status_ready),              EVENT(event_initializing),       STATUS(status_initializing)},
    {STATUS(status_ready),              EVENT(event_standby0),           STATUS(status_standby)},

    // (分割线)---------------------------------------------------------------------
    {STATUS(status_initializing),       EVENT(event_next),               STATUS(status_start0_current)},
    {STATUS(status_start0_current),     EVENT(event_next),               STATUS(status_start1_angle)},
    {STATUS(status_start1_angle),       EVENT(event_next),               STATUS(status_idle)},
    
    // (分割线)---------------------------------------------------------------------
    {STATUS(status_idle),               EVENT(event_torque_hold),        STATUS(status_torque_hold)},
    {STATUS(status_idle),               EVENT(event_torque_increasing),  STATUS(status_torque_increasing)},
    {STATUS(status_idle),               EVENT(event_torque_decreasing),  STATUS(status_torque_decreasing)},

    {STATUS(status_idle),               EVENT(event_velocity_hold),      STATUS(status_velocity_hold)},
    {STATUS(status_idle),               EVENT(event_velocity_increasing),STATUS(status_velocity_increasing)},
    {STATUS(status_idle),               EVENT(event_velocity_decreasing),STATUS(status_velocity_decreasing)},

    {STATUS(status_idle),               EVENT(event_position_hold),      STATUS(status_position_hold)},
    {STATUS(status_idle),               EVENT(event_position_increasing),STATUS(status_position_increasing)},
    {STATUS(status_idle),               EVENT(event_position_decreasing),STATUS(status_position_decreasing)},
    
    {STATUS(status_idle),               EVENT(event_standby0),           STATUS(status_standby)},
    {STATUS(status_idle),               EVENT(event_ready),              STATUS(status_ready)},

    {STATUS(status_idle),               EVENT(event_overvoltage),        STATUS(status_overvoltage)},
    {STATUS(status_idle),               EVENT(event_undervoltage),       STATUS(status_undervoltage)},
    {STATUS(status_idle),               EVENT(event_overcurrent_moment), STATUS(status_overcurrent_moment)},
    {STATUS(status_idle),               EVENT(event_overcurrent_stable), STATUS(status_overcurrent_stable)},
    {STATUS(status_idle),               EVENT(event_overtemp_motor),     STATUS(status_overtemp_motor)},
    {STATUS(status_idle),               EVENT(event_undertemp_motor),    STATUS(status_undertemp_motor)},
    {STATUS(status_idle),               EVENT(event_overtemp_driver),    STATUS(status_overtemp_driver)},
    {STATUS(status_idle),               EVENT(event_undertemp_driver),   STATUS(status_undertemp_driver)},
    {STATUS(status_idle),               EVENT(event_overtemp_pcb),       STATUS(status_overtemp_pcb)},
    {STATUS(status_idle),               EVENT(event_undertemp_pcb),      STATUS(status_undertemp_pcb)},
    {STATUS(status_idle),               EVENT(event_fault0),             STATUS(status_fault0)},
    {STATUS(status_idle),               EVENT(event_fault1),             STATUS(status_fault1)},
    {STATUS(status_idle),               EVENT(event_fault2),             STATUS(status_fault2)},
    
    // (分割线)---------------------------------------------------------------------
    {STATUS(status_torque_increasing),  EVENT(event_torque_hold),        STATUS(status_torque_hold)},
    {STATUS(status_torque_increasing),  EVENT(event_torque_decreasing),  STATUS(status_torque_decreasing)},

    {STATUS(status_torque_increasing),  EVENT(event_standby0),           STATUS(status_standby)},
    {STATUS(status_torque_increasing),  EVENT(event_ready),              STATUS(status_ready)},
    
    {STATUS(status_torque_increasing),  EVENT(event_overvoltage),        STATUS(status_overvoltage)},
    {STATUS(status_torque_increasing),  EVENT(event_undervoltage),       STATUS(status_undervoltage)},
    {STATUS(status_torque_increasing),  EVENT(event_overcurrent_moment), STATUS(status_overcurrent_moment)},
    {STATUS(status_torque_increasing),  EVENT(event_overcurrent_stable), STATUS(status_overcurrent_stable)},
    {STATUS(status_torque_increasing),  EVENT(event_overtemp_motor),     STATUS(status_overtemp_motor)},
    {STATUS(status_torque_increasing),  EVENT(event_undertemp_motor),    STATUS(status_undertemp_motor)},
    {STATUS(status_torque_increasing),  EVENT(event_overtemp_driver),    STATUS(status_overtemp_driver)},
    {STATUS(status_torque_increasing),  EVENT(event_undertemp_driver),   STATUS(status_undertemp_driver)},
    {STATUS(status_torque_increasing),  EVENT(event_overtemp_pcb),       STATUS(status_overtemp_pcb)},
    {STATUS(status_torque_increasing),  EVENT(event_undertemp_pcb),      STATUS(status_undertemp_pcb)},
    {STATUS(status_torque_increasing),  EVENT(event_fault0),             STATUS(status_fault0)},
    {STATUS(status_torque_increasing),  EVENT(event_fault1),             STATUS(status_fault1)},
    {STATUS(status_torque_increasing),  EVENT(event_fault2),             STATUS(status_fault2)},

    // (分割线)---------------------------------------------------------------------
    {STATUS(status_torque_decreasing),  EVENT(event_torque_hold),        STATUS(status_torque_hold)},
    {STATUS(status_torque_decreasing),  EVENT(event_torque_increasing),  STATUS(status_torque_increasing)},

    {STATUS(status_torque_decreasing),  EVENT(event_standby0),           STATUS(status_standby)},
    {STATUS(status_torque_decreasing),  EVENT(event_ready),              STATUS(status_ready)},
    
    {STATUS(status_torque_decreasing),  EVENT(event_overvoltage),        STATUS(status_overvoltage)},
    {STATUS(status_torque_decreasing),  EVENT(event_undervoltage),       STATUS(status_undervoltage)},
    {STATUS(status_torque_decreasing),  EVENT(event_overcurrent_moment), STATUS(status_overcurrent_moment)},
    {STATUS(status_torque_decreasing),  EVENT(event_overcurrent_stable), STATUS(status_overcurrent_stable)},
    {STATUS(status_torque_decreasing),  EVENT(event_overtemp_motor),     STATUS(status_overtemp_motor)},
    {STATUS(status_torque_decreasing),  EVENT(event_undertemp_motor),    STATUS(status_undertemp_motor)},
    {STATUS(status_torque_decreasing),  EVENT(event_overtemp_driver),    STATUS(status_overtemp_driver)},
    {STATUS(status_torque_decreasing),  EVENT(event_undertemp_driver),   STATUS(status_undertemp_driver)},
    {STATUS(status_torque_decreasing),  EVENT(event_overtemp_pcb),       STATUS(status_overtemp_pcb)},
    {STATUS(status_torque_decreasing),  EVENT(event_undertemp_pcb),      STATUS(status_undertemp_pcb)},
    {STATUS(status_torque_decreasing),  EVENT(event_fault0),             STATUS(status_fault0)},
    {STATUS(status_torque_decreasing),  EVENT(event_fault1),             STATUS(status_fault1)},
    {STATUS(status_torque_decreasing),  EVENT(event_fault2),             STATUS(status_fault2)},

    // (分割线)---------------------------------------------------------------------
    {STATUS(status_torque_hold),        EVENT(event_torque_increasing),  STATUS(status_torque_increasing)},
    {STATUS(status_torque_hold),        EVENT(event_torque_decreasing),  STATUS(status_torque_decreasing)},

    {STATUS(status_torque_hold),        EVENT(event_standby0),           STATUS(status_standby)},
    {STATUS(status_torque_hold),        EVENT(event_ready),              STATUS(status_ready)},
    
    {STATUS(status_torque_hold),        EVENT(event_overvoltage),        STATUS(status_overvoltage)},
    {STATUS(status_torque_hold),        EVENT(event_undervoltage),       STATUS(status_undervoltage)},
    {STATUS(status_torque_hold),        EVENT(event_overcurrent_moment), STATUS(status_overcurrent_moment)},
    {STATUS(status_torque_hold),        EVENT(event_overcurrent_stable), STATUS(status_overcurrent_stable)},
    {STATUS(status_torque_hold),        EVENT(event_overtemp_motor),     STATUS(status_overtemp_motor)},
    {STATUS(status_torque_hold),        EVENT(event_undertemp_motor),    STATUS(status_undertemp_motor)},
    {STATUS(status_torque_hold),        EVENT(event_overtemp_driver),    STATUS(status_overtemp_driver)},
    {STATUS(status_torque_hold),        EVENT(event_undertemp_driver),   STATUS(status_undertemp_driver)},
    {STATUS(status_torque_hold),        EVENT(event_overtemp_pcb),       STATUS(status_overtemp_pcb)},
    {STATUS(status_torque_hold),        EVENT(event_undertemp_pcb),      STATUS(status_undertemp_pcb)},
    {STATUS(status_torque_hold),        EVENT(event_fault0),             STATUS(status_fault0)},
    {STATUS(status_torque_hold),        EVENT(event_fault1),             STATUS(status_fault1)},
    {STATUS(status_torque_hold),        EVENT(event_fault2),             STATUS(status_fault2)},

    // (分割线)---------------------------------------------------------------------
    {STATUS(status_velocity_increasing),EVENT(event_velocity_hold),      STATUS(status_velocity_hold)},
    {STATUS(status_velocity_increasing),EVENT(event_velocity_decreasing),STATUS(status_velocity_decreasing)},

    {STATUS(status_velocity_increasing),EVENT(event_standby0),           STATUS(status_standby)},
    {STATUS(status_velocity_increasing),EVENT(event_ready),              STATUS(status_ready)},
    
    {STATUS(status_velocity_increasing),EVENT(event_overvoltage),        STATUS(status_overvoltage)},
    {STATUS(status_velocity_increasing),EVENT(event_undervoltage),       STATUS(status_undervoltage)},
    {STATUS(status_velocity_increasing),EVENT(event_overcurrent_moment), STATUS(status_overcurrent_moment)},
    {STATUS(status_velocity_increasing),EVENT(event_overcurrent_stable), STATUS(status_overcurrent_stable)},
    {STATUS(status_velocity_increasing),EVENT(event_overtemp_motor),     STATUS(status_overtemp_motor)},
    {STATUS(status_velocity_increasing),EVENT(event_undertemp_motor),    STATUS(status_undertemp_motor)},
    {STATUS(status_velocity_increasing),EVENT(event_overtemp_driver),    STATUS(status_overtemp_driver)},
    {STATUS(status_velocity_increasing),EVENT(event_undertemp_driver),   STATUS(status_undertemp_driver)},
    {STATUS(status_velocity_increasing),EVENT(event_overtemp_pcb),       STATUS(status_overtemp_pcb)},
    {STATUS(status_velocity_increasing),EVENT(event_undertemp_pcb),      STATUS(status_undertemp_pcb)},
    {STATUS(status_velocity_increasing),EVENT(event_fault0),             STATUS(status_fault0)},
    {STATUS(status_velocity_increasing),EVENT(event_fault1),             STATUS(status_fault1)},

    // (分割线)---------------------------------------------------------------------
    {STATUS(status_velocity_decreasing),EVENT(event_velocity_hold),      STATUS(status_velocity_hold)},
    {STATUS(status_velocity_decreasing),EVENT(event_velocity_increasing),STATUS(status_velocity_increasing)},

    {STATUS(status_velocity_decreasing),EVENT(event_standby0),           STATUS(status_standby)},
    {STATUS(status_velocity_decreasing),EVENT(event_ready),              STATUS(status_ready)},
    
    {STATUS(status_velocity_decreasing),EVENT(event_overvoltage),        STATUS(status_overvoltage)},
    {STATUS(status_velocity_decreasing),EVENT(event_undervoltage),       STATUS(status_undervoltage)},
    {STATUS(status_velocity_decreasing),EVENT(event_overcurrent_moment), STATUS(status_overcurrent_moment)},
    {STATUS(status_velocity_decreasing),EVENT(event_overcurrent_stable), STATUS(status_overcurrent_stable)},
    {STATUS(status_velocity_decreasing),EVENT(event_overtemp_motor),     STATUS(status_overtemp_motor)},
    {STATUS(status_velocity_decreasing),EVENT(event_undertemp_motor),    STATUS(status_undertemp_motor)},
    {STATUS(status_velocity_decreasing),EVENT(event_overtemp_driver),    STATUS(status_overtemp_driver)},
    {STATUS(status_velocity_decreasing),EVENT(event_undertemp_driver),   STATUS(status_undertemp_driver)},
    {STATUS(status_velocity_decreasing),EVENT(event_overtemp_pcb),       STATUS(status_overtemp_pcb)},
    {STATUS(status_velocity_decreasing),EVENT(event_undertemp_pcb),      STATUS(status_undertemp_pcb)},
    {STATUS(status_velocity_decreasing),EVENT(event_fault0),             STATUS(status_fault0)},
    {STATUS(status_velocity_decreasing),EVENT(event_fault1),             STATUS(status_fault1)},
    {STATUS(status_velocity_decreasing),EVENT(event_fault2),             STATUS(status_fault2)},

    // (分割线)---------------------------------------------------------------------
    {STATUS(status_velocity_hold),      EVENT(event_velocity_increasing),STATUS(status_velocity_increasing)},
    {STATUS(status_velocity_hold),      EVENT(event_velocity_decreasing),STATUS(status_velocity_decreasing)},

    {STATUS(status_velocity_hold),      EVENT(event_standby0),           STATUS(status_standby)},
    {STATUS(status_velocity_hold),      EVENT(event_ready),              STATUS(status_ready)},
    
    {STATUS(status_velocity_hold),      EVENT(event_overvoltage),        STATUS(status_overvoltage)},
    {STATUS(status_velocity_hold),      EVENT(event_undervoltage),       STATUS(status_undervoltage)},
    {STATUS(status_velocity_hold),      EVENT(event_overcurrent_moment), STATUS(status_overcurrent_moment)},
    {STATUS(status_velocity_hold),      EVENT(event_overcurrent_stable), STATUS(status_overcurrent_stable)},
    {STATUS(status_velocity_hold),      EVENT(event_overtemp_motor),     STATUS(status_overtemp_motor)},
    {STATUS(status_velocity_hold),      EVENT(event_undertemp_motor),    STATUS(status_undertemp_motor)},
    {STATUS(status_velocity_hold),      EVENT(event_overtemp_driver),    STATUS(status_overtemp_driver)},
    {STATUS(status_velocity_hold),      EVENT(event_undertemp_driver),   STATUS(status_undertemp_driver)},
    {STATUS(status_velocity_hold),      EVENT(event_overtemp_pcb),       STATUS(status_overtemp_pcb)},
    {STATUS(status_velocity_hold),      EVENT(event_undertemp_pcb),      STATUS(status_undertemp_pcb)},
    {STATUS(status_velocity_hold),      EVENT(event_fault0),             STATUS(status_fault0)},
    {STATUS(status_velocity_hold),      EVENT(event_fault1),             STATUS(status_fault1)},
    {STATUS(status_velocity_hold),      EVENT(event_fault2),             STATUS(status_fault2)},

    // (分割线)---------------------------------------------------------------------
    {STATUS(status_position_increasing),EVENT(event_position_hold),      STATUS(status_position_hold)},
    {STATUS(status_position_increasing),EVENT(event_position_decreasing),STATUS(status_position_decreasing)},

    {STATUS(status_position_increasing),EVENT(event_standby0),           STATUS(status_standby)},
    {STATUS(status_position_increasing),EVENT(event_ready),              STATUS(status_ready)},
    
    {STATUS(status_position_increasing),EVENT(event_overvoltage),        STATUS(status_overvoltage)},
    {STATUS(status_position_increasing),EVENT(event_undervoltage),       STATUS(status_undervoltage)},
    {STATUS(status_position_increasing),EVENT(event_overcurrent_moment), STATUS(status_overcurrent_moment)},
    {STATUS(status_position_increasing),EVENT(event_overcurrent_stable), STATUS(status_overcurrent_stable)},
    {STATUS(status_position_increasing),EVENT(event_overtemp_motor),     STATUS(status_overtemp_motor)},
    {STATUS(status_position_increasing),EVENT(event_undertemp_motor),    STATUS(status_undertemp_motor)},
    {STATUS(status_position_increasing),EVENT(event_overtemp_driver),    STATUS(status_overtemp_driver)},
    {STATUS(status_position_increasing),EVENT(event_undertemp_driver),   STATUS(status_undertemp_driver)},
    {STATUS(status_position_increasing),EVENT(event_overtemp_pcb),       STATUS(status_overtemp_pcb)},
    {STATUS(status_position_increasing),EVENT(event_undertemp_pcb),      STATUS(status_undertemp_pcb)},
    {STATUS(status_position_increasing),EVENT(event_fault0),             STATUS(status_fault0)},

    // (分割线)---------------------------------------------------------------------
    {STATUS(status_position_decreasing),EVENT(event_position_hold),      STATUS(status_position_hold)},
    {STATUS(status_position_decreasing),EVENT(event_position_increasing),STATUS(status_position_increasing)},

    {STATUS(status_position_decreasing),EVENT(event_standby0),           STATUS(status_standby)},
    {STATUS(status_position_decreasing),EVENT(event_ready),              STATUS(status_ready)},
    
    {STATUS(status_position_decreasing),EVENT(event_overvoltage),        STATUS(status_overvoltage)},
    {STATUS(status_position_decreasing),EVENT(event_undervoltage),       STATUS(status_undervoltage)},
    {STATUS(status_position_decreasing),EVENT(event_overcurrent_moment), STATUS(status_overcurrent_moment)},
    {STATUS(status_position_decreasing),EVENT(event_overcurrent_stable), STATUS(status_overcurrent_stable)},
    {STATUS(status_position_decreasing),EVENT(event_overtemp_motor),     STATUS(status_overtemp_motor)},
    {STATUS(status_position_decreasing),EVENT(event_undertemp_motor),    STATUS(status_undertemp_motor)},
    {STATUS(status_position_decreasing),EVENT(event_overtemp_driver),    STATUS(status_overtemp_driver)},
    {STATUS(status_position_decreasing),EVENT(event_undertemp_driver),   STATUS(status_undertemp_driver)},
    {STATUS(status_position_decreasing),EVENT(event_overtemp_pcb),       STATUS(status_overtemp_pcb)},
    {STATUS(status_position_decreasing),EVENT(event_undertemp_pcb),      STATUS(status_undertemp_pcb)},
    {STATUS(status_position_decreasing),EVENT(event_fault0),             STATUS(status_fault0)},
    {STATUS(status_position_decreasing),EVENT(event_fault1),             STATUS(status_fault1)},
    {STATUS(status_position_decreasing),EVENT(event_fault2),             STATUS(status_fault2)},

    // (分割线)---------------------------------------------------------------------
    {STATUS(status_position_hold),      EVENT(event_position_increasing),STATUS(status_position_increasing)},
    {STATUS(status_position_hold),      EVENT(event_position_decreasing),STATUS(status_position_decreasing)},

    {STATUS(status_position_hold),      EVENT(event_standby0),           STATUS(status_standby)},
    {STATUS(status_position_hold),      EVENT(event_ready),              STATUS(status_ready)},
    
    {STATUS(status_position_hold),      EVENT(event_overvoltage),        STATUS(status_overvoltage)},
    {STATUS(status_position_hold),      EVENT(event_undervoltage),       STATUS(status_undervoltage)},
    {STATUS(status_position_hold),      EVENT(event_overcurrent_moment), STATUS(status_overcurrent_moment)},
    {STATUS(status_position_hold),      EVENT(event_overcurrent_stable), STATUS(status_overcurrent_stable)},
    {STATUS(status_position_hold),      EVENT(event_overtemp_motor),     STATUS(status_overtemp_motor)},
    {STATUS(status_position_hold),      EVENT(event_undertemp_motor),    STATUS(status_undertemp_motor)},
    {STATUS(status_position_hold),      EVENT(event_overtemp_driver),    STATUS(status_overtemp_driver)},
    {STATUS(status_position_hold),      EVENT(event_undertemp_driver),   STATUS(status_undertemp_driver)},
    {STATUS(status_position_hold),      EVENT(event_overtemp_pcb),       STATUS(status_overtemp_pcb)},
    {STATUS(status_position_hold),      EVENT(event_undertemp_pcb),      STATUS(status_undertemp_pcb)},
    {STATUS(status_position_hold),      EVENT(event_fault0),             STATUS(status_fault0)},
    {STATUS(status_position_hold),      EVENT(event_fault1),             STATUS(status_fault1)},
    {STATUS(status_position_hold),      EVENT(event_fault2),             STATUS(status_fault2)},

    // (分割线)---------------------------------------------------------------------
    {STATUS(status_overvoltage),        EVENT(event_standby0),           STATUS(status_standby)},
    {STATUS(status_overvoltage),        EVENT(event_standby1),           STATUS(status_standby)},

    {STATUS(status_undervoltage),       EVENT(event_standby0),           STATUS(status_standby)},
    {STATUS(status_undervoltage),       EVENT(event_standby1),           STATUS(status_standby)},

    {STATUS(status_overcurrent_moment), EVENT(event_standby0),           STATUS(status_standby)},
    {STATUS(status_overcurrent_moment), EVENT(event_standby1),           STATUS(status_standby)},

    {STATUS(status_overcurrent_stable), EVENT(event_standby0),           STATUS(status_standby)},
    {STATUS(status_overcurrent_stable), EVENT(event_standby1),           STATUS(status_standby)},

    {STATUS(status_overtemp_motor),     EVENT(event_standby0),           STATUS(status_standby)},
    {STATUS(status_overtemp_motor),     EVENT(event_standby1),           STATUS(status_standby)},

    {STATUS(status_undertemp_motor),    EVENT(event_standby0),           STATUS(status_standby)},
    {STATUS(status_undertemp_motor),    EVENT(event_standby1),           STATUS(status_standby)},

    {STATUS(status_overtemp_driver),    EVENT(event_standby0),           STATUS(status_standby)},
    {STATUS(status_overtemp_driver),    EVENT(event_standby1),           STATUS(status_standby)},

    {STATUS(status_undertemp_driver),   EVENT(event_standby0),           STATUS(status_standby)},
    {STATUS(status_undertemp_driver),   EVENT(event_standby1),           STATUS(status_standby)},

    {STATUS(status_overtemp_pcb),       EVENT(event_standby0),           STATUS(status_standby)},
    {STATUS(status_overtemp_pcb),       EVENT(event_standby1),           STATUS(status_standby)},

    {STATUS(status_undertemp_pcb),      EVENT(event_standby0),           STATUS(status_standby)},
    {STATUS(status_undertemp_pcb),      EVENT(event_standby1),           STATUS(status_standby)},

    {STATUS(status_fault0),             EVENT(event_standby0),           STATUS(status_standby)},
    {STATUS(status_fault0),             EVENT(event_standby1),           STATUS(status_standby)},

    {STATUS(status_fault1),             EVENT(event_standby0),           STATUS(status_standby)},
    {STATUS(status_fault1),             EVENT(event_standby1),           STATUS(status_standby)},

    {STATUS(status_fault2),             EVENT(event_standby0),           STATUS(status_standby)},
    {STATUS(status_fault2),             EVENT(event_standby1),           STATUS(status_standby)}
};

// ()
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
    __local_null_init,
    __local_null_init,
    __local_null_init,
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

// ()
static void (*const __local_action_loop[])(uint8_t id) = {
    __local_standby_loop,

    __local_ready_loop,
    __local_initial_loop,
    __local_initial_loop,
    __local_initial_loop,

    __local_null_loop,
    __local_increasing_loop,
    __local_decreasing_loop,
    __local_hold_loop,
    __local_increasing_loop,
    __local_decreasing_loop,
    __local_hold_loop,
    __local_increasing_loop,
    __local_decreasing_loop,
    __local_hold_loop,

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
// ()
static void __local_standby_init(MotorStatus *motorstatus){

}

// ()
static void __local_ready_init(MotorStatus *motorstatus){
    
}

// ()
static void __local_initializing_init(MotorStatus *motorstatus){

}

// ()
static void __local_start0_current_init(MotorStatus *motorstatus){
    
}

// ()
static void __local_start1_angle_init(MotorStatus *motorstatus){

}

// ()
static void __local_com_init(MotorStatus *motorstatus){

}

// ()
static void __local_fault_init(MotorStatus *motorstatus){

}

// ================================================================================
// 4.全局函数motorstatus实现
void motorstatus_ready_init(MotorStatus *motorstatus){

}

void motorstatus_initializing_init(MotorStatus *motorstatus){

}

void motorstatus_next_init(MotorStatus *motorstatus){

}

void motorstatus_standby_init(MotorStatus *motorstatus){

}

void motorstatus_loop(MotorStatus *motorstatus){
    for (uint16_t i = 0; i < 184; i++){
        if (__local_motor_tab[i].status_now == motorstatus->status){
            switch ((__LocalEvent)__local_motor_tab[i].event_now){
                case event_ready:{
                    // 此处有触发event的代码

                    break;
                }
                case event_initializing:{
                    // 此处有触发event的代码

                    break;
                }
                case event_next:{
                    // 此处有触发event的代码

                    break;
                }
                case event_standby0:{
                    // 此处有触发event的代码

                    break;
                }
                case event_standby1:{
                    // 此处有触发event的代码

                    break;
                }
                case event_torque_hold:{
                    // 此处有触发event的代码

                    break;
                }
                case event_torque_increasing:{
                    // 此处有触发event的代码

                    break;
                }
                case event_torque_decreasing:{
                    // 此处有触发event的代码

                    break;
                }
                case event_velocity_hold:{
                    // 此处有触发event的代码

                    break;
                }
                case event_velocity_increasing:{
                    // 此处有触发event的代码

                    break;
                }
                case event_velocity_decreasing:{
                    // 此处有触发event的代码

                    break;
                }
                case event_position_hold:{
                    // 此处有触发event的代码

                    break;
                }
                case event_position_increasing:{
                    // 此处有触发event的代码

                    break;
                }
                case event_position_decreasing:{
                    // 此处有触发event的代码

                    break;
                }
                case event_overvoltage:{
                    // 此处有触发event的代码

                    break;
                }
                case event_undervoltage:{
                    // 此处有触发event的代码

                    break;
                }
                case event_overcurrent_moment:{
                    // 此处有触发event的代码

                    break;
                }
                case event_overcurrent_stable:{
                    // 此处有触发event的代码

                    break;
                }
                case event_overtemp_motor:{
                    // 此处有触发event的代码

                    break;
                }
                case event_undertemp_motor:{
                    // 此处有触发event的代码

                    break;
                }
                case event_overtemp_driver:{
                    // 此处有触发event的代码

                    break;
                }
                case event_undertemp_driver:{
                    // 此处有触发event的代码

                    break;
                }
                case event_overtemp_pcb:{
                    // 此处有触发event的代码

                    break;
                }
                case event_undertemp_pcb:{
                    // 此处有触发event的代码

                    break;
                }
                case event_fault0:{
                    // 此处有触发event的代码

                    break;
                }
                case event_fault1:{
                    // 此处有触发event的代码

                    break;
                }
                case event_fault2:{
                    // 此处有触发event的代码

                    break;
                }

                default:
                    break;
            }
        }
    }

    if (motorstatus->last != motorstatus->status){
        // 仅在状态机变化开始，初始化一次init函数并更新last状态机
        __local_action_init[motorstatus->status](motorstatus);
        motorstatus->last = motorstatus->status;
    }
    __local_action_loop[motorstatus->status](motorstatus->params.motor_id);
}



































































































// void motorstatus_loop00(void *sguan){
//     // ==================================================================
//     // 1.结构体类型转换(SguanFoc结构体)
//     SguanFoc *p = (SguanFoc *)sguan;
 
//     // ==================================================================
//     // 2.遍历状态转换表，给触发事件的条件
//     for (uint16_t i = 0; i < 184; i++){
//         if (__local_motor_tab[i].status_now == p->motorstatus.status){
//             switch ((__LocalEvent)__local_motor_tab[i].event_now){
//                 case event_ready:{
//                     // 此处有触发event的代码
//                     if (__local_event_get(p->run_flag) == 
//                         (__LocalEvent)__local_motor_tab[i].event_now){ // 手动触发
//                         p->run_flag = 0;

//                         p->motorstatus.status = 
//                             (__LocalStatus)__local_motor_tab[i].status_future;
//                     }
//                     break;
//                 }
//                 case event_initializing:{
//                     // 此处有触发event的代码
//                     if (__local_event_get(p->run_flag) == 
//                         (__LocalEvent)__local_motor_tab[i].event_now){ // 手动触发
//                         p->run_flag = 0;
                            
//                         p->motorstatus.status = 
//                             (__LocalStatus)__local_motor_tab[i].status_future;
//                     }
//                     break;
//                 }
//                 case event_success:{
//                     // 此处有触发event的代码
//                     if (__local_event_get(p->run_flag) == 
//                         (__LocalEvent)__local_motor_tab[i].event_now){ // 手动触发
//                         p->run_flag = 0;
                            
//                         p->motorstatus.status = 
//                             (__LocalStatus)__local_motor_tab[i].status_future;
//                     }
//                     break;
//                 }
//                 case event_standby0:{
//                     // 此处有触发event的代码
//                     if (__local_event_get(p->run_flag) == 
//                         (__LocalStatus)__local_motor_tab[i].event_now){ // 手动触发
//                         p->run_flag = 0;
                            
//                         p->motorstatus.status = 
//                             (__LocalStatus)__local_motor_tab[i].status_future;
//                     }
//                     break;
//                 }
//                 case event_standby1:{
//                     // 此处有触发event的代码
//                     // uint32_t tick = 
//                     //     p->motorstatus.data.tick_run - 
//                     //     p->motorstatus.data.tick_last;

//                     // if (tick >= p->motorstatus.data.fault_tick_delay){ // 自动触发
                            
//                     //     p->motorstatus.status = 
//                     //         (__LocalStatus)__local_motor_tab[i].status_future;
//                     // }



//                     break;
//                 }
//                 case event_torque_hold:{
//                     // 此处有触发event的代码
//                     // if (p->motorstatus.data.mode_flag == __LOCAL_MODE_TORQUE){
//                     //     SguanQ tor_target_min = iqmath_sub(
//                     //         p->motorstatus.data.tor_target, 
//                     //         p->motorstatus.data.tor_interval);
//                     //     SguanQ tor_target_max = iqmath_add(
//                     //         p->motorstatus.data.tor_target, 
//                     //         p->motorstatus.data.tor_interval);
//                     //     SguanQ tor_real_abs = iqmath_abs(
//                     //         p->motorstatus.data.tor_real);
        
//                     //     if ((tor_real_abs >= tor_target_min) && 
//                     //         (tor_real_abs <= tor_target_max)){ // 自动触发
//                     //         p->motorstatus.data.stuck_tick_run = 
//                     //             p->motorstatus.data.tick_run;

//                     //         p->motorstatus.status = 
//                     //             (__LocalStatus)__local_motor_tab[i].status_future;
//                     //     }
//                     // }



//                     break;
//                 }
//                 case event_torque_increasing:{
//                     // 此处有触发event的代码
//                     // if (p->motorstatus.data.mode_flag == __LOCAL_MODE_TORQUE){                    
//                     //     SguanQ tor_target_min = iqmath_sub(
//                     //         p->motorstatus.data.tor_target, 
//                     //         p->motorstatus.data.tor_interval);
//                     //     SguanQ tor_real_abs = iqmath_abs(
//                     //         p->motorstatus.data.tor_real);
        
//                     //     if (tor_real_abs <= tor_target_min){ // 自动触发
                            
//                     //         p->motorstatus.status = 
//                     //             (__LocalStatus)__local_motor_tab[i].status_future;
//                     //     }
//                     // }



//                     break;
//                 }
//                 case event_torque_decreasing:{
//                     // 此处有触发event的代码
//                     // if (p->motorstatus.data.mode_flag == __LOCAL_MODE_TORQUE){
//                     //     SguanQ tor_target_max = iqmath_add(
//                     //         p->motorstatus.data.tor_target, 
//                     //         p->motorstatus.data.tor_interval);
//                     //     SguanQ tor_real_abs = iqmath_abs(
//                     //         p->motorstatus.data.tor_real);
        
//                     //     if (tor_real_abs >= tor_target_max){ // 自动触发
                            
//                     //         p->motorstatus.status = 
//                     //             (__LocalStatus)__local_motor_tab[i].status_future;
//                     //     }
//                     // }


                    
//                     break;
//                 }
//                 case event_velocity_hold:{
//                     // 此处有触发event的代码
//                     if (p->motorstatus.data.mode_flag == __LOCAL_MODE_VELOCITY){
//                         SguanQ vel_target_min = iqmath_sub(
//                             p->motorstatus.data.vel_target, 
//                             p->motorstatus.data.vel_interval);
//                         SguanQ vel_target_max = iqmath_add(
//                             p->motorstatus.data.vel_target, 
//                             p->motorstatus.data.vel_interval);
//                         SguanQ vel_real_abs = iqmath_abs(
//                             p->motorstatus.data.vel_real);
        
//                         if ((vel_real_abs >= vel_target_min) && 
//                             (vel_real_abs <= vel_target_max)){ // 自动触发

                            
//                             p->motorstatus.status = 
//                                 (__LocalStatus)__local_motor_tab[i].status_future;
//                         }
//                     }
//                     break;
//                 }
//                 case event_velocity_increasing:{
//                     // 此处有触发event的代码
//                     if (p->motorstatus.data.mode_flag == __LOCAL_MODE_VELOCITY){
//                         SguanQ vel_target_min = iqmath_sub(
//                             p->motorstatus.data.vel_target, 
//                             p->motorstatus.data.vel_interval);
//                         SguanQ vel_real_abs = iqmath_abs(
//                             p->motorstatus.data.vel_real);
        
//                         if (vel_real_abs <= vel_target_min){ // 自动触发
                            
//                             p->motorstatus.status = 
//                                 (__LocalStatus)__local_motor_tab[i].status_future;
//                         }
//                     }
//                     break;
//                 }
//                 case event_velocity_decreasing:{
//                     // 此处有触发event的代码
//                     if (p->motorstatus.data.mode_flag == __LOCAL_MODE_VELOCITY){
//                         SguanQ vel_target_max = iqmath_add(
//                             p->motorstatus.data.vel_target, 
//                             p->motorstatus.data.vel_interval);
//                         SguanQ vel_real_abs = iqmath_abs(
//                             p->motorstatus.data.vel_real);
        
//                         if (vel_real_abs >= vel_target_max){ // 自动触发
                            
//                             p->motorstatus.status = 
//                                 (__LocalStatus)__local_motor_tab[i].status_future;
//                         }
//                     }
//                     break;
//                 }
//                 case event_position_hold:{
//                     // 此处有触发event的代码
//                     if (p->motorstatus.data.mode_flag == __LOCAL_MODE_POSITION){
//                         SguanQ pos_target_min = iqmath_sub(
//                             p->motorstatus.data.pos_target, 
//                             p->motorstatus.data.pos_interval);
//                         SguanQ pos_target_max = iqmath_add(
//                             p->motorstatus.data.pos_target, 
//                             p->motorstatus.data.pos_interval);
//                         SguanQ pos_real_abs = iqmath_abs(
//                             p->motorstatus.data.pos_real);
        
//                         if ((pos_real_abs >= pos_target_min) && 
//                             (pos_real_abs <= pos_target_max)){ // 自动触发
                            
//                             p->motorstatus.status = 
//                                 (__LocalStatus)__local_motor_tab[i].status_future;
//                         }
//                     }
//                     break;
//                 }
//                 case event_position_increasing:{
//                     // 此处有触发event的代码
//                     if (p->motorstatus.data.mode_flag == __LOCAL_MODE_POSITION){
//                         SguanQ pos_target_min = iqmath_sub(
//                             p->motorstatus.data.pos_target, 
//                             p->motorstatus.data.pos_interval);
//                         SguanQ pos_real_abs = iqmath_abs(
//                             p->motorstatus.data.pos_real);
        
//                         if (pos_real_abs <= pos_target_min){ // 自动触发
        
                            
//                             p->motorstatus.status = 
//                                 (__LocalStatus)__local_motor_tab[i].status_future;
//                         }
//                     }
//                     break;
//                 }
//                 case event_position_decreasing:{
//                     // 此处有触发event的代码
//                     if (p->motorstatus.data.mode_flag == __LOCAL_MODE_POSITION){
//                         SguanQ pos_target_max = iqmath_add(
//                             p->motorstatus.data.pos_target, 
//                             p->motorstatus.data.pos_interval);
//                         SguanQ pos_real_abs = iqmath_abs(
//                             p->motorstatus.data.pos_real);
        
//                         if (pos_real_abs >= pos_target_max){ // 自动触发
                            
//                             p->motorstatus.status = 
//                                 (__LocalStatus)__local_motor_tab[i].status_future;
//                         }
//                     }
//                     break;
//                 }
//                 case event_overvoltage:{
//                     // 此处有触发event的代码
//                     if (p->motorstatus.data.vbus_real >= 
//                         p->motorstatus.data.vbus_max){ // 自动触发
                            
//                         p->motorstatus.status = 
//                             (__LocalStatus)__local_motor_tab[i].status_future;
//                     }
//                     break;
//                 }
//                 case event_undervoltage:{
//                     // 此处有触发event的代码
//                     if (p->motorstatus.data.vbus_real <= 
//                         p->motorstatus.data.vbus_min){ // 自动触发
                            
//                         p->motorstatus.status = 
//                             (__LocalStatus)__local_motor_tab[i].status_future;
//                     }
//                     break;
//                 }
//                 case event_overcurrent_moment:{
//                     // 此处有触发event的代码
//                     uint32_t tick = 0;
//                     if (p->motorstatus.data.is_real <= 
//                         p->motorstatus.data.is_stable_max){

//                         p->motorstatus.data.is_tick_count = 
//                             p->motorstatus.data.tick_run; 
//                     }
//                     else{
//                         tick = iqmath_sub(
//                             p->motorstatus.data.tick_run, 
//                             p->motorstatus.data.is_tick_count);
//                     }

//                     if ((p->motorstatus.data.is_real >= 
//                         p->motorstatus.data.is_moment_max) || 
//                         // 过流保护“或”条件判断
//                         (tick >= p->motorstatus.data.is_tick_delay)){ // 自动触发
                            
//                         p->motorstatus.status = 
//                             (__LocalStatus)__local_motor_tab[i].status_future;
//                     }
//                     /////////////////////////////////////////////////////////////////
//                     /////////////////////////////////////////////////////////////////
//                     /////////////////////////////////////////////////////////////////
//                     /////////////////////////////////////////////////////////////////
//                     break;
//                 }
//                 case event_overcurrent_stable:{
//                     // 此处有触发event的代码
//                     uint32_t tick = 0;
//                     if (p->motorstatus.data.is_real <= 
//                         p->motorstatus.data.is_stable_max){

//                         p->motorstatus.data.is_tick_count = 
//                             p->motorstatus.data.tick_run; 
//                     }
//                     else{
//                         tick = iqmath_sub(
//                             p->motorstatus.data.tick_run, 
//                             p->motorstatus.data.is_tick_count);
//                     }

//                     if ((p->motorstatus.data.is_real >= 
//                         p->motorstatus.data.is_moment_max) || 
//                         // 过流保护“或”条件判断
//                         (tick >= p->motorstatus.data.is_tick_delay)){ // 自动触发
                            
//                         p->motorstatus.status = 
//                             (__LocalStatus)__local_motor_tab[i].status_future;
//                     }
//                     /////////////////////////////////////////////////////////////////////
//                     /////////////////////////////////////////////////////////////////////
//                     /////////////////////////////////////////////////////////////////////
//                     /////////////////////////////////////////////////////////////////////
//                     break;
//                 }
//                 case event_overtemp_motor:{
//                     // 此处有触发event的代码
//                     if (p->motorstatus.data.temp_motor_real >= 
//                         p->motorstatus.data.temp_motor_max){ // 自动触发
                            
//                         p->motorstatus.status = 
//                             (__LocalStatus)__local_motor_tab[i].status_future;
//                     }
//                     break;
//                 }
//                 case event_undertemp_motor:{
//                     // 此处有触发event的代码
//                     if (p->motorstatus.data.temp_motor_real <= 
//                         p->motorstatus.data.temp_motor_min){ // 自动触发
                            
//                         p->motorstatus.status = 
//                             (__LocalStatus)__local_motor_tab[i].status_future;
//                     }
//                     break;
//                 }
//                 case event_overtemp_driver:{
//                     // 此处有触发event的代码
//                     if (p->motorstatus.data.temp_driver_real >= 
//                         p->motorstatus.data.temp_driver_max){ // 自动触发
                            
//                         p->motorstatus.status = 
//                             (__LocalStatus)__local_motor_tab[i].status_future;
//                     }
//                     break;
//                 }
//                 case event_undertemp_driver:{
//                     // 此处有触发event的代码
//                     if (p->motorstatus.data.temp_driver_real <= 
//                         p->motorstatus.data.temp_driver_min){ // 自动触发
                            
//                         p->motorstatus.status = 
//                             (__LocalStatus)__local_motor_tab[i].status_future;
//                     }
//                     break;
//                 }
//                 case event_overtemp_pcb:{
//                     // 此处有触发event的代码
//                     if (p->motorstatus.data.temp_pcb_real >= 
//                         p->motorstatus.data.temp_pcb_max){ // 自动触发
                            
//                         p->motorstatus.status = 
//                             (__LocalStatus)__local_motor_tab[i].status_future;
//                     }
//                     break;
//                 }
//                 case event_undertemp_pcb:{
//                     // 此处有触发event的代码
//                     if (p->motorstatus.data.temp_pcb_real <= 
//                         p->motorstatus.data.temp_pcb_min){ // 自动触发
                            
//                         p->motorstatus.status = 
//                             (__LocalStatus)__local_motor_tab[i].status_future;
//                     }
//                     break;
//                 }
//                 case event_fault0:{
//                     // 此处有触发event的代码
//                     if (__local_event_get(p->run_flag) == 
//                         (__LocalStatus)__local_motor_tab[i].event_now){ // 手动触发
//                         p->run_flag = 0;
                            
//                         p->motorstatus.status = 
//                             (__LocalStatus)__local_motor_tab[i].status_future;
//                     }
//                     break;
//                 }
//                 case event_fault1:{
//                     // 此处有触发event的代码
//                     if (__local_event_get(p->run_flag) == 
//                         (__LocalStatus)__local_motor_tab[i].event_now){ // 手动触发
//                         p->run_flag = 0;
                            
//                         p->motorstatus.status = 
//                             (__LocalStatus)__local_motor_tab[i].status_future;
//                     }
//                     break;
//                 }
//                 case event_fault2:{
//                     // 此处有触发event的代码
//                     if (__local_event_get(p->run_flag) == 
//                         (__LocalStatus)__local_motor_tab[i].event_now){ // 手动触发
//                         p->run_flag = 0;
                            
//                         p->motorstatus.status = 
//                             (__LocalStatus)__local_motor_tab[i].status_future;
//                     }
//                     break;
//                 }

//                 default:
//                     break;
//             }
//         }
//     }

//     // ==================================================================
//     // 3.运行init和loop函数
//     if (p->motorstatus.last != p->motorstatus.status){
//         // 状态机发送变化，记录变化时刻的tick时间
//         p->motorstatus.data.tick_count = p->motorstatus.data.tick_run;

//         // 仅在状态机变化开始，初始化一次init函数并更新last状态机
//         __local_initial_init[p->motorstatus.status](p);
//         p->motorstatus.last = p->motorstatus.status;
//     }
//     __local_action_loop[p->motorstatus.status]();
// }



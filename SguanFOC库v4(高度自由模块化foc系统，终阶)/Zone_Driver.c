/* ===================================================================
 * 文件:     Zone_Driver.c
 * 项目:     SguanFOC_v4 —— 开源 FOC 电机控制框架
 * 说明:     4.Zone用户驱动层_用户驱动（精炼版1：表驱动直接调用）
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
#include "Zone_Driver.h"

// (分割线)-------------------------------------------------------------------------
// 用户驱动文件包含，如有则include驱动
// 代码包含在此处：
// (分割线)-------------------------------------------------------------------------

// ================================================================================
// 1.电机数量裁剪(m1~m6)
#if CONFIG_MOTOR > SGUANFOC_MOTOR_TWO
#define __LOCAL_MOTOR_2(f)     f(2)
#else
#define __LOCAL_MOTOR_2(f)
#endif
#if CONFIG_MOTOR > SGUANFOC_MOTOR_THREE
#define __LOCAL_MOTOR_3(f)     f(3)
#else
#define __LOCAL_MOTOR_3(f)
#endif
#if CONFIG_MOTOR > SGUANFOC_MOTOR_FOUR
#define __LOCAL_MOTOR_4(f)     f(4)
#else
#define __LOCAL_MOTOR_4(f)
#endif
#if CONFIG_MOTOR > SGUANFOC_MOTOR_FIVE
#define __LOCAL_MOTOR_5(f)     f(5)
#else
#define __LOCAL_MOTOR_5(f)
#endif
#if CONFIG_MOTOR > SGUANFOC_MOTOR_SIX
#define __LOCAL_MOTOR_6(f)     f(6)
#else
#define __LOCAL_MOTOR_6(f)
#endif

// ================================================================================
// 2.生成宏和本地驱动函数声明
// (电机遍历宏定义)
#define __LOCAL_MOTOR_FOREACH(f) \
    f(1) __LOCAL_MOTOR_2(f) __LOCAL_MOTOR_3(f) \
    __LOCAL_MOTOR_4(f) __LOCAL_MOTOR_5(f) __LOCAL_MOTOR_6(f)

// (本地函数声明宏)
#define __LOCAL_DECLARE(n) \
    static void   __local_set_pwm_m##n(const uint32_t duty[8]); \
    static SguanQ __local_read_adc0_m##n(void); \
    static SguanQ __local_read_adc1_m##n(void); \
    static SguanQ __local_read_adc2_m##n(void); \
    static SguanQ __local_read_adc3_m##n(void); \
    static SguanQ __local_read_vbus_m##n(void); \
    static SguanQ __local_read_temp_motor_m##n(void); \
    static SguanQ __local_read_temp_driver_m##n(void); \
    static SguanQ __local_read_temp_pcb_m##n(void); \
    static void   __local_start_m##n(void); \
    static void   __local_stop_m##n(void); \
    static void   __local_standby_loop_m##n(void); \
    static void   __local_ready_loop_m##n(void); \
    static void   __local_initial_loop_m##n(void); \
    static void   __local_increasing_loop_m##n(void); \
    static void   __local_decreasing_loop_m##n(void); \
    static void   __local_hold_loop_m##n(void); \
    static void   __local_overvoltage_loop_m##n(void); \
    static void   __local_undervoltage_loop_m##n(void); \
    static void   __local_overcurrent_moment_loop_m##n(void); \
    static void   __local_overcurrent_stable_loop_m##n(void); \
    static void   __local_overtemp_motor_loop_m##n(void); \
    static void   __local_undertemp_motor_loop_m##n(void); \
    static void   __local_overtemp_driver_loop_m##n(void); \
    static void   __local_undertemp_driver_loop_m##n(void); \
    static void   __local_overtemp_pcb_loop_m##n(void); \
    static void   __local_undertemp_pcb_loop_m##n(void); \
    static void   __local_fault0_loop_m##n(void); \
    static void   __local_fault1_loop_m##n(void); \
    static void   __local_fault2_loop_m##n(void);

// (展开全部函数声明)
__LOCAL_MOTOR_FOREACH(__LOCAL_DECLARE)

// ================================================================================
// 3.局部电机的驱动能力表
// (能力表项生成宏)
#define __LOCAL_OPS_ENTRY(n) { \
    __local_set_pwm_m##n, \
    __local_read_adc0_m##n, \
    __local_read_adc1_m##n, \
    __local_read_adc2_m##n, \
    __local_read_adc3_m##n, \
    __local_read_vbus_m##n, \
    __local_read_temp_motor_m##n, \
    __local_read_temp_driver_m##n, \
    __local_read_temp_pcb_m##n, \
    __local_start_m##n, \
    __local_stop_m##n, \
    __local_standby_loop_m##n, \
    __local_ready_loop_m##n, \
    __local_initial_loop_m##n, \
    __local_increasing_loop_m##n, \
    __local_decreasing_loop_m##n, \
    __local_hold_loop_m##n, \
    __local_overvoltage_loop_m##n, \
    __local_undervoltage_loop_m##n, \
    __local_overcurrent_moment_loop_m##n, \
    __local_overcurrent_stable_loop_m##n, \
    __local_overtemp_motor_loop_m##n, \
    __local_undertemp_motor_loop_m##n, \
    __local_overtemp_driver_loop_m##n, \
    __local_undertemp_driver_loop_m##n, \
    __local_overtemp_pcb_loop_m##n, \
    __local_undertemp_pcb_loop_m##n, \
    __local_fault0_loop_m##n, \
    __local_fault1_loop_m##n, \
    __local_fault2_loop_m##n \
},

// (驱动能力表总表)
const __LocalOps driver_ops[] = {
    __LOCAL_MOTOR_FOREACH(__LOCAL_OPS_ENTRY)
};

// ================================================================================
// 4.用户驱动代码书写
// (读取运行节拍)
uint32_t driver_read_tick(void){
    // Write_your_driver_code
    return 0;
}

// (毫秒延时函数)
void driver_delay_ms(uint32_t ms){
    // Write_your_driver_code
    (void)ms;
}

// (串口发送数据)
void driver_send_uart(uint8_t *ch, uint16_t size){
    // Write_your_driver_code
    (void)ch;
    (void)size;
}

// (Modbus帧发送)
void driver_send_modbus(uint8_t data[8]){
    // Write_your_driver_code
    (void)data;
}

// (驱动层初始化)
void driver_initial(void){
    // Write_your_driver_code
}

// (分割线)-------------------------------------------------------------------------
// (下发三相PWM)
static void __local_set_pwm_m1(const uint32_t duty[8]){ 
    // Write_your_driver_code
    (void)duty; 
}

// (ADC0采样值)
static SguanQ __local_read_adc0_m1(void){ 
    // Write_your_driver_code
    return (SguanQ)0; 
}

// (ADC1采样值)
static SguanQ __local_read_adc1_m1(void){ 
    // Write_your_driver_code
    return (SguanQ)0; 
}

// (ADC2采样值)
static SguanQ __local_read_adc2_m1(void){ 
    // Write_your_driver_code
    return (SguanQ)0; 
}

// (ADC3采样值)
static SguanQ __local_read_adc3_m1(void){ 
    // Write_your_driver_code
    return (SguanQ)0; 
}

// (母线电压采样)
static SguanQ __local_read_vbus_m1(void){ 
    // Write_your_driver_code
    return (SguanQ)0; 
}

// (电机温度采样)
static SguanQ __local_read_temp_motor_m1(void){ 
    // Write_your_driver_code
    return (SguanQ)0; 
}

// (驱动温度采样)
static SguanQ __local_read_temp_driver_m1(void){ 
    // Write_your_driver_code
    return (SguanQ)0; 
}

// (电路板温度采样)
static SguanQ __local_read_temp_pcb_m1(void){ 
    // Write_your_driver_code
    return (SguanQ)0; 
}

// (启动电机输出)
static void __local_start_m1(void){ 
    // Write_your_driver_code
}

// (停止电机输出)
static void __local_stop_m1(void){ 
    // Write_your_driver_code
}

// (待机状态循环)
static void __local_standby_loop_m1(void){ 
    // Write_your_driver_code
}

// (就绪状态循环)
static void __local_ready_loop_m1(void){ 
    // Write_your_driver_code
}

// (初始化状态循环)
static void __local_initial_loop_m1(void){ 
    // Write_your_driver_code
}

// (加速状态循环)
static void __local_increasing_loop_m1(void){ 
    // Write_your_driver_code
}

// (减速状态循环)
static void __local_decreasing_loop_m1(void){ 
    // Write_your_driver_code
}

// (保持状态循环)
static void __local_hold_loop_m1(void){ 
    // Write_your_driver_code
}

// (过压故障处理)
static void __local_overvoltage_loop_m1(void){ 
    // Write_your_driver_code
}

// (欠压故障处理)
static void __local_undervoltage_loop_m1(void){ 
    // Write_your_driver_code
}

// (瞬态过流处理)
static void __local_overcurrent_moment_loop_m1(void){ 
    // Write_your_driver_code
}

// (稳态过流处理)
static void __local_overcurrent_stable_loop_m1(void){ 
    // Write_your_driver_code
}

// (电机过温处理)
static void __local_overtemp_motor_loop_m1(void){ 
    // Write_your_driver_code
}

// (电机欠温处理)
static void __local_undertemp_motor_loop_m1(void){ 
    // Write_your_driver_code
}

// (驱动过温处理)
static void __local_overtemp_driver_loop_m1(void){ 
    // Write_your_driver_code
}

// (驱动欠温处理)
static void __local_undertemp_driver_loop_m1(void){ 
    // Write_your_driver_code
}

// (电路板过温处理)
static void __local_overtemp_pcb_loop_m1(void){ 
    // Write_your_driver_code
}

// (电路板欠温处理)
static void __local_undertemp_pcb_loop_m1(void){ 
    // Write_your_driver_code
}

// (故障0处理)
static void __local_fault0_loop_m1(void){ 
    // Write_your_driver_code
}

// (故障1处理)
static void __local_fault1_loop_m1(void){ 
    // Write_your_driver_code
}

// (故障2处理)
static void __local_fault2_loop_m1(void){ 
    // Write_your_driver_code
}

#if CONFIG_MOTOR > SGUANFOC_MOTOR_TWO
static void   __local_set_pwm_m2(const uint32_t duty[8]){ (void)duty; }
static SguanQ __local_read_adc0_m2(void){ return (SguanQ)0; }
static SguanQ __local_read_adc1_m2(void){ return (SguanQ)0; }
static SguanQ __local_read_adc2_m2(void){ return (SguanQ)0; }
static SguanQ __local_read_adc3_m2(void){ return (SguanQ)0; }
static SguanQ __local_read_vbus_m2(void){ return (SguanQ)0; }
static SguanQ __local_read_temp_motor_m2(void){ return (SguanQ)0; }
static SguanQ __local_read_temp_driver_m2(void){ return (SguanQ)0; }
static SguanQ __local_read_temp_pcb_m2(void){ return (SguanQ)0; }
static void   __local_start_m2(void){ }
static void   __local_stop_m2(void){ }
static void   __local_standby_loop_m2(void){ }
static void   __local_ready_loop_m2(void){ }
static void   __local_initial_loop_m2(void){ }
static void   __local_increasing_loop_m2(void){ }
static void   __local_decreasing_loop_m2(void){ }
static void   __local_hold_loop_m2(void){ }
static void   __local_overvoltage_loop_m2(void){ }
static void   __local_undervoltage_loop_m2(void){ }
static void   __local_overcurrent_moment_loop_m2(void){ }
static void   __local_overcurrent_stable_loop_m2(void){ }
static void   __local_overtemp_motor_loop_m2(void){ }
static void   __local_undertemp_motor_loop_m2(void){ }
static void   __local_overtemp_driver_loop_m2(void){ }
static void   __local_undertemp_driver_loop_m2(void){ }
static void   __local_overtemp_pcb_loop_m2(void){ }
static void   __local_undertemp_pcb_loop_m2(void){ }
static void   __local_fault0_loop_m2(void){ }
static void   __local_fault1_loop_m2(void){ }
static void   __local_fault2_loop_m2(void){ }
#endif

#if CONFIG_MOTOR > SGUANFOC_MOTOR_THREE
static void   __local_set_pwm_m3(const uint32_t duty[8]){ (void)duty; }
static SguanQ __local_read_adc0_m3(void){ return (SguanQ)0; }
static SguanQ __local_read_adc1_m3(void){ return (SguanQ)0; }
static SguanQ __local_read_adc2_m3(void){ return (SguanQ)0; }
static SguanQ __local_read_adc3_m3(void){ return (SguanQ)0; }
static SguanQ __local_read_vbus_m3(void){ return (SguanQ)0; }
static SguanQ __local_read_temp_motor_m3(void){ return (SguanQ)0; }
static SguanQ __local_read_temp_driver_m3(void){ return (SguanQ)0; }
static SguanQ __local_read_temp_pcb_m3(void){ return (SguanQ)0; }
static void   __local_start_m3(void){ }
static void   __local_stop_m3(void){ }
static void   __local_standby_loop_m3(void){ }
static void   __local_ready_loop_m3(void){ }
static void   __local_initial_loop_m3(void){ }
static void   __local_increasing_loop_m3(void){ }
static void   __local_decreasing_loop_m3(void){ }
static void   __local_hold_loop_m3(void){ }
static void   __local_overvoltage_loop_m3(void){ }
static void   __local_undervoltage_loop_m3(void){ }
static void   __local_overcurrent_moment_loop_m3(void){ }
static void   __local_overcurrent_stable_loop_m3(void){ }
static void   __local_overtemp_motor_loop_m3(void){ }
static void   __local_undertemp_motor_loop_m3(void){ }
static void   __local_overtemp_driver_loop_m3(void){ }
static void   __local_undertemp_driver_loop_m3(void){ }
static void   __local_overtemp_pcb_loop_m3(void){ }
static void   __local_undertemp_pcb_loop_m3(void){ }
static void   __local_fault0_loop_m3(void){ }
static void   __local_fault1_loop_m3(void){ }
static void   __local_fault2_loop_m3(void){ }
#endif

#if CONFIG_MOTOR > SGUANFOC_MOTOR_FOUR
static void   __local_set_pwm_m4(const uint32_t duty[8]){ (void)duty; }
static SguanQ __local_read_adc0_m4(void){ return (SguanQ)0; }
static SguanQ __local_read_adc1_m4(void){ return (SguanQ)0; }
static SguanQ __local_read_adc2_m4(void){ return (SguanQ)0; }
static SguanQ __local_read_adc3_m4(void){ return (SguanQ)0; }
static SguanQ __local_read_vbus_m4(void){ return (SguanQ)0; }
static SguanQ __local_read_temp_motor_m4(void){ return (SguanQ)0; }
static SguanQ __local_read_temp_driver_m4(void){ return (SguanQ)0; }
static SguanQ __local_read_temp_pcb_m4(void){ return (SguanQ)0; }
static void   __local_start_m4(void){ }
static void   __local_stop_m4(void){ }
static void   __local_standby_loop_m4(void){ }
static void   __local_ready_loop_m4(void){ }
static void   __local_initial_loop_m4(void){ }
static void   __local_increasing_loop_m4(void){ }
static void   __local_decreasing_loop_m4(void){ }
static void   __local_hold_loop_m4(void){ }
static void   __local_overvoltage_loop_m4(void){ }
static void   __local_undervoltage_loop_m4(void){ }
static void   __local_overcurrent_moment_loop_m4(void){ }
static void   __local_overcurrent_stable_loop_m4(void){ }
static void   __local_overtemp_motor_loop_m4(void){ }
static void   __local_undertemp_motor_loop_m4(void){ }
static void   __local_overtemp_driver_loop_m4(void){ }
static void   __local_undertemp_driver_loop_m4(void){ }
static void   __local_overtemp_pcb_loop_m4(void){ }
static void   __local_undertemp_pcb_loop_m4(void){ }
static void   __local_fault0_loop_m4(void){ }
static void   __local_fault1_loop_m4(void){ }
static void   __local_fault2_loop_m4(void){ }
#endif

#if CONFIG_MOTOR > SGUANFOC_MOTOR_FIVE
static void   __local_set_pwm_m5(const uint32_t duty[8]){ (void)duty; }
static SguanQ __local_read_adc0_m5(void){ return (SguanQ)0; }
static SguanQ __local_read_adc1_m5(void){ return (SguanQ)0; }
static SguanQ __local_read_adc2_m5(void){ return (SguanQ)0; }
static SguanQ __local_read_adc3_m5(void){ return (SguanQ)0; }
static SguanQ __local_read_vbus_m5(void){ return (SguanQ)0; }
static SguanQ __local_read_temp_motor_m5(void){ return (SguanQ)0; }
static SguanQ __local_read_temp_driver_m5(void){ return (SguanQ)0; }
static SguanQ __local_read_temp_pcb_m5(void){ return (SguanQ)0; }
static void   __local_start_m5(void){ }
static void   __local_stop_m5(void){ }
static void   __local_standby_loop_m5(void){ }
static void   __local_ready_loop_m5(void){ }
static void   __local_initial_loop_m5(void){ }
static void   __local_increasing_loop_m5(void){ }
static void   __local_decreasing_loop_m5(void){ }
static void   __local_hold_loop_m5(void){ }
static void   __local_overvoltage_loop_m5(void){ }
static void   __local_undervoltage_loop_m5(void){ }
static void   __local_overcurrent_moment_loop_m5(void){ }
static void   __local_overcurrent_stable_loop_m5(void){ }
static void   __local_overtemp_motor_loop_m5(void){ }
static void   __local_undertemp_motor_loop_m5(void){ }
static void   __local_overtemp_driver_loop_m5(void){ }
static void   __local_undertemp_driver_loop_m5(void){ }
static void   __local_overtemp_pcb_loop_m5(void){ }
static void   __local_undertemp_pcb_loop_m5(void){ }
static void   __local_fault0_loop_m5(void){ }
static void   __local_fault1_loop_m5(void){ }
static void   __local_fault2_loop_m5(void){ }
#endif

#if CONFIG_MOTOR > SGUANFOC_MOTOR_SIX
static void   __local_set_pwm_m6(const uint32_t duty[8]){ (void)duty; }
static SguanQ __local_read_adc0_m6(void){ return (SguanQ)0; }
static SguanQ __local_read_adc1_m6(void){ return (SguanQ)0; }
static SguanQ __local_read_adc2_m6(void){ return (SguanQ)0; }
static SguanQ __local_read_adc3_m6(void){ return (SguanQ)0; }
static SguanQ __local_read_vbus_m6(void){ return (SguanQ)0; }
static SguanQ __local_read_temp_motor_m6(void){ return (SguanQ)0; }
static SguanQ __local_read_temp_driver_m6(void){ return (SguanQ)0; }
static SguanQ __local_read_temp_pcb_m6(void){ return (SguanQ)0; }
static void   __local_start_m6(void){ }
static void   __local_stop_m6(void){ }
static void   __local_standby_loop_m6(void){ }
static void   __local_ready_loop_m6(void){ }
static void   __local_initial_loop_m6(void){ }
static void   __local_increasing_loop_m6(void){ }
static void   __local_decreasing_loop_m6(void){ }
static void   __local_hold_loop_m6(void){ }
static void   __local_overvoltage_loop_m6(void){ }
static void   __local_undervoltage_loop_m6(void){ }
static void   __local_overcurrent_moment_loop_m6(void){ }
static void   __local_overcurrent_stable_loop_m6(void){ }
static void   __local_overtemp_motor_loop_m6(void){ }
static void   __local_undertemp_motor_loop_m6(void){ }
static void   __local_overtemp_driver_loop_m6(void){ }
static void   __local_undertemp_driver_loop_m6(void){ }
static void   __local_overtemp_pcb_loop_m6(void){ }
static void   __local_undertemp_pcb_loop_m6(void){ }
static void   __local_fault0_loop_m6(void){ }
static void   __local_fault1_loop_m6(void){ }
static void   __local_fault2_loop_m6(void){ }
#endif

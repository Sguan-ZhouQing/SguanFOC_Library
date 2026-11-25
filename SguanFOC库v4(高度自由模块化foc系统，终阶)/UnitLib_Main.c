/* ===================================================================
 * 文件:     UnitLib_Main.c
 * 项目:     SguanFOC_v4 —— 开源 FOC 电机控制框架
 * 说明:     2.UnitLib应用库_(应用)算法核心
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
#include "UnitLib_main.h"

/* SguanFOC配置文件声明 */
#include "UserMotor_Parameter.h"
#include "Zone_Driver.h"


void main_high_loop_one(SguanFoc *sguan){
    
}

void main_high_loop_two(SguanFoc *sguan){

}

void main_high_loop_three(SguanFoc *sguan){

}

void main_high_loop_four(SguanFoc *sguan){

}

void main_high_loop_five(SguanFoc *sguan){

}

void main_high_loop_six(SguanFoc *sguan){

}

// ===============================================================================
void main_high_loop(SguanFoc *sguan){
    
}

// ===============================================================================
void main_low_loop(SguanFoc *sguan){
    // 1.初始化状态机数据
    sguan->motorstatus.data.tor_real        = sguan->current.real_iq;
    sguan->motorstatus.data.tor_target      = sguan->foc.target_iq;
    sguan->motorstatus.data.vel_real        = sguan->encoder.real_speed;
    sguan->motorstatus.data.vel_target      = sguan->foc.target_speed;
    sguan->motorstatus.data.pos_real        = sguan->encoder.real_position;
    sguan->motorstatus.data.pos_target      = sguan->foc.target_pos;
    sguan->motorstatus.data.vbus_real       = driver_read_vbus(sguan->id_flag);
    // sguan->motorstatus.data.is_real         = driver_read_ibus(sguan->id_flag);
    sguan->motorstatus.data.temp_motor_real = driver_read_temp_motor(sguan->id_flag);
    sguan->motorstatus.data.temp_driver_real= driver_read_temp_driver(sguan->id_flag);
    sguan->motorstatus.data.temp_pcb_real   = driver_read_temp_pcb(sguan->id_flag);
    sguan->motorstatus.data.tick_run        = driver_read_tick();
    
    // 2.根据电机状态给赋值参数
    // (现在默认给速度控制)
    // sguan->motorstatus.data.mode_flag       = __LOCAL_CONTROL_VELOCITY;

    // 3.运行状态机处理函数
    motorstatus_loop(sguan);
}

// ===============================================================================
void main_main_loop_one(SguanFoc *sguan){
    
}

void main_main_loop_two(SguanFoc *sguan){

}

void main_main_loop_three(SguanFoc *sguan){

}

void main_main_loop_four(SguanFoc *sguan){

}

void main_main_loop_five(SguanFoc *sguan){

}

void main_main_loop_six(SguanFoc *sguan){

}

// ===============================================================================
void main_main_loop(SguanFoc *sguan){

}



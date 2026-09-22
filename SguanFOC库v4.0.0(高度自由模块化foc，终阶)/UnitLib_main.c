#include "UnitLib_main.h"

/* SguanFOC配置文件声明 */
#include "UserData_Driver.h"
#include "Zone_Master.h"
#include "Zone_Parameter.h"

void main_standby_init(SguanFoc *sguanfoc){

}

void main_ready_init(SguanFoc *sguan){

}

void main_initializing_init(SguanFoc *sguan){

    // 最后赋值run_flag，切换到下一个状态
    sguan->run_flag = RUN_SUCCESS;
}

void main_goinit0_angle_init(SguanFoc *sguan){

    // 最后赋值run_flag，切换到下一个状态
    sguan->run_flag = RUN_SUCCESS;
}

void main_goinit1_current_init(SguanFoc *sguan){

    // 最后赋值run_flag，切换到下一个状态
    sguan->run_flag = RUN_SUCCESS;
}

// ===============================================================================
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

void main_low_loop(SguanFoc *sguan){
    // 1.初始化状态机数据
    sguan->motorstatus.data.tor_real        = sguan->current.real_iq;
    sguan->motorstatus.data.tor_target      = sguan->foc.target_iq;
    sguan->motorstatus.data.vel_real        = sguan->encoder.real_speed;
    sguan->motorstatus.data.vel_target      = sguan->foc.target_speed;
    sguan->motorstatus.data.pos_real        = sguan->encoder.real_position;
    sguan->motorstatus.data.pos_target      = sguan->foc.target_pos;
    sguan->motorstatus.data.vbus_real       = driver_read_vbus(sguan->id_flag);
    sguan->motorstatus.data.ibus_real       = driver_read_ibus(sguan->id_flag);
    sguan->motorstatus.data.temp_motor_real = driver_read_temp_motor(sguan->id_flag);
    sguan->motorstatus.data.temp_driver_real= driver_read_temp_driver(sguan->id_flag);
    sguan->motorstatus.data.temp_pcb_real   = driver_read_temp_pcb(sguan->id_flag);
    sguan->motorstatus.data.tick_run        = driver_read_tick();
    
    // 2.根据电机状态给赋值参数
    // (现在默认给速度控制)
    sguan->motorstatus.data.mode_flag       = CONTROL_VELOCITY;

    // 3.运行状态机处理函数
    motorstatus_loop(sguan);
}


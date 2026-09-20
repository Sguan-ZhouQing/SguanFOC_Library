#ifndef __UNITLIB_USERCONTROL_H
#define __UNITLIB_USERCONTROL_H
/* SguanFOC配置文件声明 */
#include "SguanFOC.h"
#include "Sguan_Printf.h"

// 指令处理回调函数库->MOTOR
static inline void usercontrol_motor(float value){

}

// 指令处理回调函数库->Speed
static inline void usercontrol_speed(float value){

}

// 指令处理回调函数库->Position
static inline void usercontrol_position(float value){

}

// 指令处理回调函数库->Id
static inline void usercontrol_id(float value){

}

// 指令处理回调函数库->Iq
static inline void usercontrol_iq(float value){

}

// 指令处理回调函数库->Ud
static inline void usercontrol_ud(float value){

}

// 指令处理回调函数库->Uq
static inline void usercontrol_uq(float value){

}


// 动态可修改的指令字典（预留 hash_val 位置为 0）
static CmdMap cmd_dict[] = {
    {"MOTOR",       0, usercontrol_motor},
    {"Speed",       0, usercontrol_speed},
    {"Position",    0, usercontrol_position},
    {"Id",          0, usercontrol_id},
    {"Iq",          0, usercontrol_iq},
    {"Ud",          0, usercontrol_ud},
    {"Uq",          0, usercontrol_uq}
};






#endif // UNITLIB_USERCONTROL_H

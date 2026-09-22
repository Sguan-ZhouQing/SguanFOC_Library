#ifndef __UNITLIB_USERCONTROL_H
#define __UNITLIB_USERCONTROL_H
/* SguanFOC配置文件声明 */
#include "SguanFOC.h"
#include "Sguan_Printf.h"

void usercontrol_motor(float value);
void usercontrol_speed(float value);
void usercontrol_position(float value);
void usercontrol_id(float value);
void usercontrol_iq(float value);
void usercontrol_ud(float value);
void usercontrol_uq(float value);

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

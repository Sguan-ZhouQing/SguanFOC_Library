#ifndef __UNITLIB_USERCONTROL_H
#define __UNITLIB_USERCONTROL_H
/* SguanFOC配置文件声明 */
#include "SguanFOC.h"
#include "Sguan_Printf.h"

void unitlib_usercontrol_motor(float value);
void unitlib_usercontrol_speed(float value);
void unitlib_usercontrol_position(float value);
void unitlib_usercontrol_id(float value);
void unitlib_usercontrol_iq(float value);
void unitlib_usercontrol_ud(float value);
void unitlib_usercontrol_uq(float value);

// 动态可修改的指令字典（预留 hash_val 位置为 0）
static CmdMap cmd_dict[] = {
    {"MOTOR",       0, unitlib_usercontrol_motor},
    {"Speed",       0, unitlib_usercontrol_speed},
    {"Position",    0, unitlib_usercontrol_position},
    {"Id",          0, unitlib_usercontrol_id},
    {"Iq",          0, unitlib_usercontrol_iq},
    {"Ud",          0, unitlib_usercontrol_ud},
    {"Uq",          0, unitlib_usercontrol_uq}
};






#endif // UNITLIB_USERCONTROL_H

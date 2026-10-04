#ifndef __SGUAN_MOTORSTATUS_H
#define __SGUAN_MOTORSTATUS_H

/* SguanFOC配置文件声明 */
#include "Sguan_Config.h"

// 如果触发错误状态，电机直接变成Standby状态
#define STATUS_Standby              0x00    // (Standby)待机
#define STATUS_Ready                0x01    // (Ready)准备
#define STATUS_Initializing0        0x02    // (Initializing0)初始化
#define STATUS_Initializing1        0x03    // (Initializing1)初始化
#define STATUS_COM                  0x04    // (COM)正常运行

// 错误状态记录
#define ERROR_Zreo                  0x00    // (Zreo)正常运行，无错误
#define ERROR_OverVoltage           0x01    // (OverVoltage)过压
#define ERROR_UnderVoltage          0x02    // (UnderVoltage)欠压
#define ERROR_OverTemp              0x03    // (OverTemp)过温
#define ERROR_UnderTemp             0x04    // (UnderTemp)低温
#define ERROR_OverCurrent           0x05    // (OverCurrent)过流


#endif // SGUAN_MOTORSTATUS_H

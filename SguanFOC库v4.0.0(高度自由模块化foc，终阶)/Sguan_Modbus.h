#ifndef __SGUAN_MODBUS_H
#define __SGUAN_MODBUS_H

/* SguanFOC配置文件声明 */
#include "UnitLib_UserRTU.h"

void modbus_master_send(void);
void modbus_master_read(void);
void modbus_slave_send(void);
void modbus_slave_read(void);


#endif // SGUAN_MODBUS_H

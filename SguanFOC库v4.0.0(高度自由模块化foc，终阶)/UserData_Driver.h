#ifndef __USERDATA_DRIVER_H
#define __USERDATA_DRIVER_H
/* SguanFOC配置文件声明 */
#include "SguanFOC.h"

uint32_t driver_read_tick(void);
SguanQ driver_read_vbus(uint8_t CH);
SguanQ driver_read_ibus(uint8_t CH);
SguanQ driver_read_temp_motor(uint8_t CH);
SguanQ driver_read_temp_driver(uint8_t CH);
SguanQ driver_read_temp_pcb(uint8_t CH);




#endif // USERDATA_DRIVER_H

#ifndef __USERDATA_DRIVER_H
#define __USERDATA_DRIVER_H
/* SguanFOC配置文件声明 */
#include "SguanFOC.h"

uint32_t userdata_driver_read_tick(void);
SguanQ userdata_driver_read_vbus(uint8_t CH);
SguanQ userdata_driver_read_ibus(uint8_t CH);
SguanQ userdata_driver_read_temp_motor(uint8_t CH);
SguanQ userdata_driver_read_temp_driver(uint8_t CH);
SguanQ userdata_driver_read_temp_pcb(uint8_t CH);
void userdata_driver_set_uart(uint8_t *ch, uint16_t size);
void userdata_driver_set_pwm(uint8_t motor, 
    uint16_t du_0, uint16_t du_1, 
    uint16_t dv_0, uint16_t dv_1, 
    uint16_t dw_0, uint16_t dw_1);
void userdata_driver_set_adc(uint8_t motor, uint16_t x);



#endif // USERDATA_DRIVER_H

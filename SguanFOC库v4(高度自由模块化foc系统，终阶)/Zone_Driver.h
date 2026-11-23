#ifndef __ZONE_DRIVER_H
#define __ZONE_DRIVER_H

/* SguanFOC配置文件声明 */
#include "SguanFOC.h"

void driver_initial_init(void);
void driver_start_init(void);
void driver_stop_init(void);

uint32_t driver_read_tick(void);
SguanQ driver_read_vbus(uint8_t CH);
SguanQ driver_read_ibus(uint8_t CH);
SguanQ driver_read_temp_motor(uint8_t CH);
SguanQ driver_read_temp_driver(uint8_t CH);
SguanQ driver_read_temp_pcb(uint8_t CH);
void driver_read_adc(uint8_t motor, uint16_t x);
void driver_set_pwm(uint8_t motor, 
    uint16_t du_0, uint16_t du_1, 
    uint16_t dv_0, uint16_t dv_1, 
    uint16_t dw_0, uint16_t dw_1,
    uint16_t dx_0, uint16_t dx_1);
void driver_send_uart(uint8_t *ch, uint16_t size);


#endif // ZONE_DRIVER_H

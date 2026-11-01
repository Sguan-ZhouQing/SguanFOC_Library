#ifndef __driver_H
#define __driver_H
/* SguanFOC配置文件声明 */
#include "SguanFOC.h"

uint32_t driver_read_tick(void);
SguanQ driver_read_vbus(uint8_t CH);
SguanQ driver_read_ibus(uint8_t CH);
SguanQ driver_read_temp_motor(uint8_t CH);
SguanQ driver_read_temp_driver(uint8_t CH);
SguanQ driver_read_temp_pcb(uint8_t CH);
void driver_set_uart(uint8_t *ch, uint16_t size);
void driver_set_pwm(uint8_t motor, 
    uint16_t du_0, uint16_t du_1, 
    uint16_t dv_0, uint16_t dv_1, 
    uint16_t dw_0, uint16_t dw_1);
void driver_set_adc(uint8_t motor, uint16_t x);



#endif // driver_H

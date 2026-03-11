#ifndef __SGUAN_MODBUSRTU_H
#define __SGUAN_MODBUSRTU_H

/* 外部函数声明 */
#include "Sguan_Calculate.h"

#define Reg_MAX 32

typedef struct{
    uint8_t slave_addr;
    uint16_t reg_values[Reg_MAX];
    uint8_t reg_count;
}USER_REGBUFF_STRUCT;

void Modbus_Init(void);
void ModbusRTU_Send(uint8_t *data,uint8_t len);
void ModbusRTU_Read_03(USER_REGBUFF_STRUCT *user_buff,
                    uint8_t slave_addr,
                    uint16_t start_addr,
                    uint16_t num_regs);
void ModbusRTU_Write_06(uint8_t slave_addr,uint16_t reg_addr,uint16_t value);
void Modbus_Write_16(uint8_t slave_addr,uint16_t start_addr,uint16_t num_regs,uint16_t *values);


#endif // SGUAN_MODBUSRTU_H

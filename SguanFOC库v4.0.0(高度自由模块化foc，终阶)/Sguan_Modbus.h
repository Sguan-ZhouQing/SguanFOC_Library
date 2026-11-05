#ifndef __SGUAN_MODBUS_H
#define __SGUAN_MODBUS_H

/* SguanFOC配置文件声明 */
#include "UnitLib_UserRTU.h"

typedef enum{   // 地址定义
    
}__LocalAddress;

typedef enum{   // 功能码定义

}__LocalCode;

typedef struct{ // 寄存器定义
    uint8_t coil_8;
    uint16_t register0;
}__LocalRegister;

typedef struct{
    uint8_t slave_address;      // 从站地址
    uint8_t function_code;      // 功能码
    uint8_t starting_address[2];// 起始地址
    uint8_t quantity[2];        // 数量
    uint8_t crc_check[2];       // CRC校验
}__LocalMaster;

typedef struct{
    uint8_t slave_address;      // 从站地址
    uint8_t function_code;      // 功能码
    uint8_t byte_count;         // 字节计数
    uint8_t data_segment[10];   // 数据段
    uint8_t crc_check[2];       // CRC校验
}__LocalSlave;

typedef struct{
    uint8_t flag;               // 接收完成标志位
    __LocalMaster master;       // 主站
    __LocalSlave slave;         // 从站
}Modbus;

// (主站send 延时 read)
// (从站read 延时 send)
void modbus_master_send(void);
void modbus_master_read(void);
void modbus_slave_send(void);
void modbus_slave_read(void);


#endif // SGUAN_MODBUS_H

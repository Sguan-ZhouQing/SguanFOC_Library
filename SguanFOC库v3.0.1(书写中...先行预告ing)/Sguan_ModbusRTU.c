/*
 * @Author: 星必尘Sguan
 * @GitHub: https://github.com/Sguan-ZhouQing
 * @Date: 2026-02-19 22:29:53
 * @LastEditors: 星必尘Sguan|3464647102@qq.com
 * @LastEditTime: 2026-02-20 15:07:37
 * @FilePath: \stm_SguanFOCtest\SguanFOC\Sguan_ModbusRTU.c
 * @Description: SguanFOC库的“Modbus RTU通讯协议”实现
 * 
 * Copyright (c) 2026 by $星必尘Sguan, All Rights Reserved. 
 */
#include "Sguan_ModbusRTU.h"
// 外部函数文件声明
#include <stdint.h>
#include "UserData_Correspond.h"
// 全局指针变量声明
USER_REGBUFF_STRUCT *slave_buffs[Reg_MAX];
// 内部函数文件声明
static uint16_t ModbusRTU_CRC16(uint8_t *data, uint8_t len);


// RTU协议的CRC16校验函数实现
static uint16_t ModbusRTU_CRC16(uint8_t *data, uint8_t len){
    uint16_t crc = 0xFFFF;  // 初始值为0xFFFF
    uint8_t i, j;
    
    for (i = 0; i < len; i++){
        crc ^= data[i];  // 与当前字节异或
        for (j = 0; j < 8; j++){
            if (crc & 0x0001){ // 如果最低位为1,右移一位并与多项式0xA001异或
                crc >>= 1;
                crc ^= 0xA001;
            }
            else{
                crc >>= 1;     // 仅右移一位
            }
        }
    }
    return crc;  // 返回CRC16校验值
}

void Modbus_Init(void){
    for (uint8_t i = 0; i < Reg_MAX; i++){
        slave_buffs[i] = NULL;
    }
    // 这里写你的初始化函数
}


// 发送协议报文的Send函数实现
void ModbusRTU_Send(uint8_t *data,uint8_t len){
    /* your code for transmit UART */
}

// 功能码03,Modbus读取保持寄存器功能码
void ModbusRTU_Read_03(USER_REGBUFF_STRUCT *user_buff,
                    uint8_t slave_addr,
                    uint16_t start_addr,
                    uint16_t num_regs){
    uint8_t tx_data[8];
    uint16_t crc;

    tx_data[0] = slave_addr;
    tx_data[1] = 0x03;
    tx_data[2] = (start_addr >> 8) & 0xFF;
    tx_data[3] = start_addr & 0xFF;
    tx_data[4] = (num_regs >> 8) &0xFF;
    tx_data[5] = num_regs &0xFF;

    crc = ModbusRTU_CRC16(tx_data,6);
    tx_data[6] = crc &0xFF;
    tx_data[7] = (crc >> 8) & 0xFF;

    for (uint8_t i = 0; i < Reg_MAX; i++){
        if (slave_buffs[i] == NULL){
            slave_buffs[i] = user_buff;
            break;
        }
    }

    ModbusRTU_Send(tx_data,6);
}

// 功能码06,Modbus写单路保持寄存器功能码
void ModbusRTU_Write_06(uint8_t slave_addr,uint16_t reg_addr,uint16_t value){
    uint8_t tx_data[8];
    uint16_t crc;

    tx_data[0] = slave_addr;
    tx_data[1] = 0x06;
    tx_data[2] = (reg_addr >> 8) & 0xFF;
    tx_data[3] = reg_addr & 0xFF;
    tx_data[4] = (value >> 8) &0xFF;
    tx_data[5] = value &0xFF;

    crc = ModbusRTU_CRC16(tx_data,6);
    tx_data[6] = crc &0xFF;
    tx_data[7] = (crc >> 8) & 0xFF;

    ModbusRTU_Send(tx_data,6);
}

// 功能码16,Modbus写多路保持寄存器功能码
void Modbus_Write_16(uint8_t slave_addr,uint16_t start_addr,uint16_t num_regs,uint16_t *values){
    uint8_t tx_data[9 + 2*Reg_MAX];
    uint16_t crc;

    tx_data[0] = slave_addr;
    tx_data[1] = 0x10;
    tx_data[2] = (start_addr >> 8) & 0xFF;
    tx_data[3] = start_addr & 0xFF;
    tx_data[4] = (num_regs >> 8) &0xFF;
    tx_data[5] = num_regs &0xFF;
    tx_data[6] = num_regs * 2;

    for (uint8_t i = 0; i < num_regs; i++){
        tx_data[7 + 2*i] = (values[i] >> 8) & 0xFF;
        tx_data[8 + 2*i] = values[i] & 0xFF;
    }
    
    crc = ModbusRTU_CRC16(tx_data,6);
    tx_data[7 + 2*num_regs] = crc &0xFF;
    tx_data[8 + 2*num_regs] = (crc >> 8) & 0xFF;

    ModbusRTU_Send(tx_data,6);
}

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




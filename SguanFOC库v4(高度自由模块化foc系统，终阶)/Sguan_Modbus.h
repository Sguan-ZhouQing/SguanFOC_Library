/* ===================================================================
 * 文件:     Sguan_Modbus.h
 * 项目:     SguanFOC_v4 —— 开源 FOC 电机控制框架
 * 链接:     https://github.com/Sguan-ZhouQing/SguanFOC_Library
 * 版权:     Copyright 2026 星必尘Sguan <3464647102@qq.com>
 * 许可证:   Apache License, Version 2.0
 * SPDX:     SPDX-License-Identifier: Apache-2.0
 *
 * 许可说明:
 *   [EN] Licensed under the Apache License, Version 2.0. You may use,
 *        copy, modify, and distribute this software, including for
 *        commercial purposes, provided you retain this copyright
 *        notice and the terms of the License.
 *   [中文] 本软件依据 Apache License 2.0 授权：可自由使用、复制、修改、
 *        分发（含商业用途），但必须保留本版权声明与许可证条款。
 *   [DE] Lizenziert unter der Apache License, Version 2.0. Nutzung,
 *        Vervielfaeltigung, Aenderung und Weitergabe (auch kommerziell)
 *        sind erlaubt, sofern dieser Urheberrechtshinweis und die
 *        Lizenzbedingungen erhalten bleiben.
 *   [JA] Apache License, Version 2.0 に基づき許諾されます。使用・複製・
 *        改変・再配布（商用を含む）が可能ですが、本著作権表示と
 *        ライセンス条項を保持する必要があります。
 * 完整许可文本: 参见本目录下的 LICENSE 文件。
 * =================================================================== */
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

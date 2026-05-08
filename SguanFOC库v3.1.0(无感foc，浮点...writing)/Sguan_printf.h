#ifndef __SGUAN_PRINTF_H
#define __SGUAN_PRINTF_H

#include "Sguan_Config.h"
#include <stdint.h>

/* 配置区 */
#define CH_COUNT      12    // JustFloat 通道数
#define CMD_MAX_LEN   20    // 最大支持的指令长度
#define HASH_SIZE     32    // 哈希表大小

// 指令回调函数原型
typedef void (*CmdCallback)(float value);

// 指令映射结构体
typedef struct {
    const char* name;       // 指令名称
    uint32_t hash_val;      // 哈希值 (运行时计算并缓存)
    CmdCallback callback;   // 对应的处理函数
} CMD_MAP_STRUCT;

// JustFloat 数据帧结构
typedef struct {
    float fdata[CH_COUNT];
    uint8_t tail[4];
} PRINTF_STRUCT;

extern uint8_t Sguan_PrintfBuff[200];

/* 函数定义声明 */
void Printf_Hash_Init(void); // 【新增】上电初始化哈希表
void Printf_TX_Init(PRINTF_STRUCT *str);
void Printf_TX_Loop(PRINTF_STRUCT *str);
void Printf_RX_Loop(uint8_t *data, uint16_t length);

#endif // SGUAN_PRINTF_H

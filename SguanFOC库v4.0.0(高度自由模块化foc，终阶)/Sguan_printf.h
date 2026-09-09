#ifndef __SGUAN_PRINTF_H
#define __SGUAN_PRINTF_H

#include "Sguan_Config.h"

#define CH_COUNT      10    // JustFloat 通道数
#define CMD_MAX_LEN   20    // 最大支持的指令长度
#define HASH_SIZE     32    // 哈希表大小
typedef void (*cmd_callback)(float value);

typedef struct{
    const char* name;       // 指令名称
    uint32_t hash_val;      // 哈希值 (运行时计算并缓存)
    cmd_callback callback;   // 对应的处理函数
}CmdMap;

typedef struct{
    float fdata[CH_COUNT];
    uint8_t tail[4];
}Printf;

extern uint8_t sguan_printfBuff[64];

void Printf_TX_Init(Printf *str);
void Printf_TX_Loop(Printf *str);
void Printf_RX_Init(void);
void Printf_RX_Loop(uint8_t *data, uint16_t length);


#endif // SGUAN_PRINTF_H

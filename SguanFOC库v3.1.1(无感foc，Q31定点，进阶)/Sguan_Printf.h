#ifndef C595657C_D4CC_42CF_95C0_DA30423E701F
#define C595657C_D4CC_42CF_95C0_DA30423E701F
#ifndef __SGUAN_PRINTF_H
#define __SGUAN_PRINTF_H

#include "Sguan_Config.h"
#include <stdint.h>

#define CH_COUNT      6    // JustFloat 通道数
#define CMD_MAX_LEN   20    // 最大支持的指令长度
#define HASH_SIZE     32    // 哈希表大小
typedef void (*CmdCallback)(float value);

typedef struct {
    const char* name;       // 指令名称
    uint32_t hash_val;      // 哈希值 (运行时计算并缓存)
    CmdCallback callback;   // 对应的处理函数
}CMD_MAP_STRUCT;

typedef struct {
    uint8_t tail[4];
    float fdata[CH_COUNT];
}PRINTF_STRUCT;

extern uint8_t Sguan_PrintfBuff[64];

void Printf_TX_Init(PRINTF_STRUCT *str);
void Printf_TX_Loop(PRINTF_STRUCT *str);
void Printf_RX_Init(void);
void Printf_RX_Loop(uint8_t *data, uint16_t length);


#endif // SGUAN_PRINTF_H


#endif /* C595657C_D4CC_42CF_95C0_DA30423E701F */

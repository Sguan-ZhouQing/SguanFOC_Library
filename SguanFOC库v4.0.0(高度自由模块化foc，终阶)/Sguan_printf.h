#ifndef __SGUAN_PRINTF_H
#define __SGUAN_PRINTF_H

/* SguanFOC配置文件声明 */
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
    uint8_t tail[4];
    float fdata[CH_COUNT];
}Printf;

extern uint8_t sguan_printfbuff[64];

void printf_tx_init(Printf *str);
void printf_tx_loop(Printf *str);
void printf_rx_init(void);
void printf_rx_loop(uint8_t *data, uint16_t length);


#endif // SGUAN_PRINTF_H

#ifndef __SGUAN_PRINTF_H
#define __SGUAN_PRINTF_H

/* SguanFOC配置文件声明 */
#include "Sguan_Config.h"

#define LOCAL_CH_COUNT      10  // JustFloat 通道数
typedef void (*Cmd_Callback)(float value);

typedef struct{
    const char* name;           // 指令名称
    uint32_t hash_val;          // 哈希值 (运行时计算并缓存)
    Cmd_Callback callback;      // 对应的处理函数
}__LocalMap;

typedef struct{
    uint8_t tail[4];
    float fdata[LOCAL_CH_COUNT];
}Printf;

extern uint8_t sguan_printfbuff[64];

void printf_tx_init(Printf *str);
void printf_tx_loop(Printf *str);
void printf_rx_init(void);
void printf_rx_loop(uint8_t *data, uint16_t length);


#endif // SGUAN_PRINTF_H

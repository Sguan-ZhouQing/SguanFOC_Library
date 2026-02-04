#ifndef __SGUAN_PRINTF_H
#define __SGUAN_PRINTF_H

#include <stdint.h>
#include <stdio.h>

/**
 * @description: 自定义数据发送的个数(默认为8)
 * @return {*}
 */
#define CH_COUNT 8 // UART或者CAN发送的数据个数

typedef struct{
    float fdata[CH_COUNT];
    uint8_t tail[4];
}PRINTF_STRUCT;

extern uint8_t Printf_Buff[200];

void Printf_Init(PRINTF_STRUCT *str);
void Printf_Loop(PRINTF_STRUCT *str);
void Printf_Adjust(void);


#endif // SGUAN_PRINTF_H

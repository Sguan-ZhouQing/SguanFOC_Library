#ifndef __CORRESPOND_H
#define __CORRESPOND_H
#include <stdint.h>
/* 电机控制User用户设置·通信协议设定 */
#include "main.h"
extern UART_HandleTypeDef huart1;


/* ================= 驱动代码(驱动层) ================= */
static inline void User_CorrespondSet(uint8_t *ch, uint16_t size){
    /* Your code for UART or CAN Signal Transmit Driver */
    HAL_UART_Transmit(&huart1, ch, size, 0xFFFF);
}


#endif // CORRESPOND_H

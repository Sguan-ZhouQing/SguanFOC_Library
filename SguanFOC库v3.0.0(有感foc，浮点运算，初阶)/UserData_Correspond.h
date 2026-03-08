#ifndef __CORRESPOND_H
#define __CORRESPOND_H
/* 电机控制User用户设置·通信协议设定 */
#include "main.h"
extern UART_HandleTypeDef huart1;

/* ================= 驱动代码(驱动层) ================= */
static inline void User_CorrespondSet(unsigned char *ch, unsigned short int size){
    /* Your code for UART or CAN Signal Transmit Driver */
    HAL_UART_Transmit(&huart1, ch, size, 0xFFFF);
}


#endif // CORRESPOND_H

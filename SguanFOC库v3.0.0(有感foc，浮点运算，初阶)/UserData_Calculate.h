#ifndef __USERDATA_CALCULATE_H
#define __USERDATA_CALCULATE_H
/* 电机控制User用户设置·数据计算 */

/**
 * @description: 宏定义0或1决定“闭环控制系统”是否使用PI控制(默认开启)
 * @reminder: 0->电流环“PI控制”，转速环“LADRC”，位置环“PD控制” 
 * @reminder: 1->电流环“PI控制”，转速环“PI控制”，位置环“PD控制”
 * @return {*}
 */
#define Open_PI_Control 0

/**
 * @description: 宏定义0或1决定“FW弱磁控制”是否开启(默认开启)
 * @reminder: 0->不开启IPMSM的弱磁控制
 * @reminder: 1->开启基于MTPA的弱磁控制
 * @return {*}
 */
#define Open_FW_Calculate 0

/**
 * @description: 宏定义决定UART或者CAN发送数据的模式
 * @reminder: (Printf_Send)0->发送正常数据
 * @reminder: 1->仅发送Debug数据，不发送正常数据
 * @return {*}
 */
#define Printf_Debug 0


#endif // USERDATA_CALCULATE_H

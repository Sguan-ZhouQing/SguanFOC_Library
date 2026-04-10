#ifndef __USERDATA_CONFIG_H
#define __USERDATA_CONFIG_H
/* 电机控制User用户设置·数据计算 */

/**
 * @description: 宏定义0或1决定“闭环控制系统”是否使用PI控制(默认关闭)
 * @reminder: 0->电流环“PI控制”，转速环“STA二阶滑膜控制”，位置环“PD控制” 
 * @reminder: 1->电流环“PI控制”，转速环“PI控制”，位置环“PD控制”
 * @reminder: 2->电流环“PI控制”，转速环“PI控制”，位置环“PD控制”
 * @reminder: 3->电流环“PI控制”，转速环“PI控制”，位置环“PD控制”
 * @return {*}
 */
#define Switch_Control_Calculate 0

/**
 * @description: 宏定义0或1决定“MTPA最大转矩控制控制”是否开启(开启最优)
 * @reminder: 0->不开启IPMSM的MTPA最大转矩控制
 * @reminder: 1->开启最大转矩控制控制
 * @return {*}
 */
#define Open_Current_Feedforward 1

/**
 * @description: 宏定义0或1决定“MTPA最大转矩控制控制”是否开启(开启最优)
 * @reminder: 0->不开启IPMSM的MTPA最大转矩控制
 * @reminder: 1->开启最大转矩控制控制
 * @return {*}
 */
#define Open_Velocity_Feedforward 1

/**
 * @description: 宏定义0或1决定“MTPA最大转矩控制控制”是否开启(默认关闭)
 * @reminder: 0->不开启IPMSM的MTPA最大转矩控制
 * @reminder: 1->开启最大转矩控制控制
 * @return {*}
 */
#define Open_MTPA_Calculate 0

/**
 * @description: 宏定义0或1决定“FW弱磁控制”是否开启(默认关闭)
 * @reminder: 0->不开启IPMSM的弱磁控制
 * @reminder: 1->开启弱磁控制
 * @return {*}
 */
#define Open_FW_Calculate 0

/**
 * @description: 宏定义决定UART或者CAN发送数据的模式
 * @reminder: (Open_Printf_Debug)0->发送正常数据
 * @reminder: 1->仅发送Debug数据，不发送正常数据
 * @return {*}
 */
#define Open_Printf_Debug 0


// 定时器中断参数设计
#define TIM_T 1e-4                          // 最大频率的控制周期


#endif // USERDATA_CONFIG_H

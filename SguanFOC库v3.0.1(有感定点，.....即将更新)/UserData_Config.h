#ifndef __USERDATA_CONFIG_H
#define __USERDATA_CONFIG_H
/* 电机控制User用户设置·数据计算 */

/**
 * @description: 宏定义0或1决定“闭环控制系统”是否使用PI控制(默认关闭)
 * @reminder: 0->电流环“PI控制”，转速环“STA二阶滑膜控制”，位置环“PD控制” 
 * @reminder: 1->电流环“PI控制”，转速环“PI控制”，位置环“PD控制”
 * @return {*}
 */
#define Open_PI_Control 0

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

/**
 * @description: 宏定义0或1决定“Q31定点运算”是否开启(默认关闭)
 * @reminder: 0->不开启Q31浮点转定点运算
 * @reminder: 1->开启浮点转定点的运算(量纲，需要参数标幺化)
 * @return {*}
 */
#define Open_Q31_Calculate 1

/**
 * @description: Q31定点化运算的数据标幺(基值设计)
 * @reminder:BASE需要->电压，电流，电阻，磁链，电感，弧度，时间，频率
 * @return {*}
 */
#define Q_Resistor 0.5f                     // 电阻数据设定(单位为Ω欧姆)

#define Q_Time 0.0078125f                 	// 常量标幺化(单位为s)
#define Q_Rad 128.0f                        // 角度大小(单位为rad弧度)
#define Q_Speed (Q_Rad/Q_Time)              // 角速度大小(单位rad/s)
#define Q_Hz (1.0f/Q_Time)                  // 频率Hz的大小(单位1/s)
#define Q_Current 64.0f                     // 电流数据设定(单位为A安培)
#define Q_Voltage 128.0f                    // 电压数据设定(单位为V伏特)
#define Q_Inductor ((Q_Voltage*Q_Time)/Q_Current) // 电感数据设定(单位为H亨利)
#define Q_Flux (Q_Voltage*Q_Time)           // 磁链大小(单位为Wb韦伯)


// 定时器中断参数设计
#define TIM_T 5e-5                          // 最大频率的控制周期


#endif // USERDATA_CONFIG_H

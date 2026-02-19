#ifndef __USERDATA_CALCULATE_H
#define __USERDATA_CALCULATE_H
/* 电机控制User用户设置·数据计算 */

/**
 * @description: 宏定义0~3决定“电机有/无感运行模式”的开启与否(默认有感FOC控制)
 * @reminder: 0->有传感器FOC控制(全速域) | 1->无感HFI高频注入控制(低速域)
 * @reminder: 2->无感SMO滑膜观测器控制(高速域) | 3->无感HFI-SMO控制(全速域)
 * @return {*}
 */
#define MOTOR_CONTROL 0

/**
 * @description: 宏定义0或1决定“PID运算参数”自适应与否(默认开启)
 * @reminder: 0->关闭PID参数自适应 | 1->开启PID参数自适应
 * @return {*}
 */
#define Open_PID_Calculate 0

/**
 * @description: 宏定义0或1决定“PID运算参数”自适应与否(默认开启)
 * @reminder: 0->关闭PID参数自适应 | 1->开启PID参数自适应
 * @return {*}
 */
#define Open_Velocity_OPEN      1
#define Open_Current_SINGLE     1
#define Open_Velocity_SINGLE    1
#define Open_Position_SINGLE    1
#define Open_VelCur_DOUBLE      1
#define Open_PosVel_DOUBLE      1
#define Open_PosVelCur_THREE    1

/**
 * @description: 宏定义0或者1决定“电机实体参数”的测量方式(默认关闭)
 * @reminder: 0->不主动进行测量，使用UserData_Motor.h中的离线值
 * @reminder: 1->(主动离线测量)之后一直使用这个值
 * @return {*}
 */
#define Open_Quantize_Method 0

/**
 * @description: 宏定义0或1决定“Q31定点化运算”的开启与否(默认关闭)
 * @reminder: 0->浮点运算 | 1->定点运算
 * @return {*}
 */
#define Open_Q31_Calculate 0

/**
 * @description: 定点化运算的数据标幺(基值设计)
 * @return {*}
 */
#define Qmax_Voltage 16             // 电压数据设定(单位为V伏特)
#define Qmax_Current 8              // 电流数据设定(单位为A安培)
#define Qmax_Inductor 0.001953125f  // 电感数据设定(单位为H亨利)
#define Qmax_Resistor 0.5f          // 电阻数据设定(单位为Ω欧姆)
#define Qmax_Flux 0.00390625f       // 磁链大小(单位为Wb韦伯)
#define Qmax_Speed 256              // 速度大小(单位为rad/s弧度每秒)
#define Qmax_Rad 128                // 角度大小(单位为rad弧度)
#define Qmax_Time 0.00006103515625f // 常量标幺化(无数学单位)

/**
 * @description: 宏定义0或1决定“VBUS母线电压测量”的开启与否(默认关闭)
 * @reminder: 0->关闭测量 | 1->开启测量
 * @return {*}
 */
#define Open_VBUS_Calculate 0

/**
 * @description: 宏定义0或1决定“Temp驱动器物理温度测量”的开启与否(默认关闭)
 * @reminder: 0->关闭测量 | 1->开启测量
 * @return {*}
 */
#define Open_Temp_Calculate 0

/**
 * @description: 宏定义决定UART或者CAN发送数据的模式
 * @reminder: (Printf_Send)0->发送正常数据
 * @reminder: 1->仅发送Debug数据，不发送正常数据
 * @return {*}
 */
#define Printf_Debug 0


#endif // USERDATA_CALCULATE_H

#ifndef __USERDATA_CALCULATE_H
#define __USERDATA_CALCULATE_H
/* 电机控制User用户设置·数据计算及printf重定向 */

/**
 * @description: 宏定义0或1决定“PID运算参数”自适应与否(默认开启)
 * @reminder: 0->关闭PID参数自适应 | 1->开启PID参数自适应
 * @return {*}
 */
#define PID_Calculate 1

/**
 * @description: 宏定义0或者1决定“电机实体参数”的测量方式(默认关闭)
 * @reminder: 0->不主动进行测量，使用UserData_Motor.h中的离线值
 * @reminder: 1->(主动离线测量)之后一直使用这个值
 * @return {*}
 */
#define Quantize_Method 0

/**
 * @description: 宏定义0或1决定“Q31定点化运算”的开启与否(默认关闭)
 * @reminder: 0->浮点运算 | 1->定点运算
 * @return {*}
 */
#define Q31_Calculate 0

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

// 放置自己的通信驱动代码,用于信息发送
static inline void User_PrintfSet(uint8_t *ch){
    /* Your code for UART or CAN Signal Transmit Driver */
}


#endif // USERDATA_CALCULATE_H

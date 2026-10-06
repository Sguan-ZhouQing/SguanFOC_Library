#ifndef __USERDATA_CONFIG_H
#define __USERDATA_CONFIG_H
/* 电机控制User用户设置·数据计算 */

/**
 * @description: 宏定义0-5决定“电机的控制模式”(默认使用“NLFO零速双闭环”模式)
 * @reminder: 0->MODE_VF_Only           VF控制              (无感电压幅频控制开环)yes
 * @reminder: 1->MODE_IF_Only           IF控制              (无感电流幅频控制开环)yes
 * @reminder: 2->MODE_NLFO_Voltag       NLFO单电压开环       (单电压开环：直接启动)yes
 * @reminder: 3->MODE_NLFO_Vel          NLFO零速双闭环       (零速双环：直接启动)yes
 * @reminder: 4->MODE_NLFO_Speed0       磁链速度-电流闭环    (速度-电流闭环“VF切磁链”)
 * @reminder: 5->MODE_NLFO_Speed1       磁链速度-电流闭环    (速度-电流闭环“IF切磁链”)
 * @return {*}
 */
#define Define_Run_Mode 0

/**
 * @description: 宏定义0-1决定“电机是否开启电流偏置读取”(默认开启)
 * @reminder: 0->不开启读取电流偏置任务
 * @reminder: 1->开启电流偏置的Tick读取
 * @return {*}
 */
#define Current_Offset_Open 0

/**
 * @description: 宏定义0-1决定“电机是否实时更新Q31/Float之间数据的转换”(默认开启)
 * @reminder: 0->不开启数据转换
 * @reminder: 1->开启Q31/Float数据之间的转换
 * @return {*}
 */
#define Q31_Float_Open 0

/**
 * @description: 宏定义0-1决定“电机采用单电阻采样”(默认关闭)
 * @reminder: 0->不开启单电阻采样程序
 * @reminder: 1->开启电机单电阻采样功能
 * @return {*}
 */
#define Read_SingleRs_Open 0

/**
 * @description: Q31定点化运算的数据标幺(基值设计)
 * @reminder:BASE需要->电压，电流，电阻，磁链，电感，弧度，时间，频率
 * @return {*}
 */
#define Q_Time 0.001953125f                 // 常量标幺化(单位为s)
#define Q_Rad 8.0f                          // 角度大小(单位为rad弧度)
#define Q_Current 64.0f                     // 电流数据设定(单位为A安培)
#define Q_Voltage 1024.0f                   // 电压数据设定(单位为V伏特)
#define Q_Speed (Q_Rad/Q_Time)              // 角速度大小(单位rad/s)
#define Q_Hz (1.0f/Q_Time)                  // 频率Hz的大小(单位1/s)
#define Q_Inductor ((Q_Voltage*Q_Time)/Q_Current) // 电感数据设定(单位为H亨利)
#define Q_Flux (Q_Voltage*Q_Time)           // 磁链大小(单位为Wb韦伯)
#define Q_Resistor (Q_Voltage/Q_Current)    // 电阻数据设定(单位为Ω欧姆)

// 电机一些宏定义设置
// #define Current_Gain 0.0101725264f          // (电机参数)电流最后增益

// 定时器中断参数设计
// (5K)
// #define TIM_T 2e-4f                         // 最大频率的控制周期
// (4.5K)
// #define TIM_T 2.222222222e-4f                   // 最大频率的控制周期
// (4K)
// #define TIM_T 2.5e-4f                       // 最大频率的控制周期
// (2K)
#define TIM_T 5e-4f                           // 最大频率的控制周期（FOC 2kHz，PWM 8kHz）
// 单电阻采样考量
// #define Dead_Time 4.2e-6f                // 死区及电流恢复时间
// #define ADC_Time 5e-7f                   // ADC采样及转换时间
// (这里的数值已经放置在了Sguan_Value.h中做Q31数值了)


#endif // USERDATA_CONFIG_H

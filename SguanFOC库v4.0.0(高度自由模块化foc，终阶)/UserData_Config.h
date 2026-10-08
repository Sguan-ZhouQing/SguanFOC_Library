#ifndef __USERDATA_CONFIG_H
#define __USERDATA_CONFIG_H



#define USERDATA_CONFIG_SAVE 0

// 定时器中断参数设计
#define TIM_T 5e-5                          // 最大频率的控制周期

// 电机实例数量（板卡上的电机个数，必须和 SguanFOC.c 里 MOTOR_LIST 的行数一致）
// Max 10

// #define CONFIG_MOTOR DTAT


#endif // USERDATA_CONFIG_H

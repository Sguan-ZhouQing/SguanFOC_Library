#ifndef __USERDATA_CONFIG_H
#define __USERDATA_CONFIG_H

// 0->float 1->Q31 2->Q15
#define DATA_DEFINE_IQMATH 0

#define MODE 0

// // 0->null 1->64 2->128 3->256 4->512
// #define DATA_DEFINE_SinTab 4

// // 0->null 1->91
// #define DATA_DEFINE_AtanTab 4

// 定时器中断参数设计
#define TIM_T 5e-5                          // 最大频率的控制周期

// 电机实例数量（板卡上的电机个数，必须和 SguanFOC.c 里 MOTOR_LIST 的行数一致）
// Max 10
#define CONFIG_MOTOR 6
// #define CONFIG_MOTOR DTAT

#endif // USERDATA_CONFIG_H

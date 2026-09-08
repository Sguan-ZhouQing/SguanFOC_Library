#ifndef __USERDATA_CONFIG_H
#define __USERDATA_CONFIG_H

// 0->float 1->Q31 2->Q15
#define DATA_DEFINE_IQmath 0

#define MODE 0

// // 0->null 1->64 2->128 3->256 4->512
// #define DATA_DEFINE_SinTab 4

// // 0->null 1->91
// #define DATA_DEFINE_AtanTab 4

// 定时器中断参数设计
#define TIM_T 5e-5                          // 最大频率的控制周期

#endif // USERDATA_CONFIG_H

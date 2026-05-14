#ifndef __SGUAN_FILTER_H
#define __SGUAN_FILTER_H

/* SguanFOC配置文件声明 */
#include "Sguan_Config.h"

#if CONFIG_Filter==0x01
typedef struct{
    float i[2];             // (数据)历史输入值
    float o[2];             // (数据)历史输出值

    float Input;            // (数据)Input输入
    float Output;           // (数据)Output输出

    float num[3];           // (中间量)传递函数分子系数
    float den[2];           // (中间量)传递函数分母系数
}FILTER_STRUCT;

#elif CONFIG_Filter==0x02
typedef struct{
    float i[2];             // (数据)历史输入值
    float o[2];             // (数据)历史输出值

    float Input;            // (数据)Input输入
    float Output;           // (数据)Output输出

    float num[3];           // (中间量)传递函数分子系数
    float den[2];           // (中间量)传递函数分母系数
}FILTER_STRUCT;

#elif CONFIG_Filter==0x03
typedef struct{
    float i[2];             // (数据)历史输入值
    float o[2];             // (数据)历史输出值

    float Input;            // (数据)Input输入
    float Output;           // (数据)Output输出

    float num[3];           // (中间量)传递函数分子系数
    float den[2];           // (中间量)传递函数分母系数
}FILTER_STRUCT;

#elif CONFIG_Filter==0x04
typedef struct{
    float i[2];             // (数据)历史输入值
    float o[2];             // (数据)历史输出值

    float Input;            // (数据)Input输入
    float Output;           // (数据)Output输出

    float num[3];           // (中间量)传递函数分子系数
    float den[2];           // (中间量)传递函数分母系数
}FILTER_STRUCT;

#elif CONFIG_Filter==0x05
typedef struct{
    float i[2];             // (数据)历史输入值
    float o[2];             // (数据)历史输出值

    float Input;            // (数据)Input输入
    float Output;           // (数据)Output输出

    float num[3];           // (中间量)传递函数分子系数
    float den[2];           // (中间量)传递函数分母系数
}FILTER_STRUCT;

#else // CONFIG_Filter
typedef struct{
    float i[2];             // (数据)历史输入值
    float o[2];             // (数据)历史输出值

    float Input;            // (数据)Input输入
    float Output;           // (数据)Output输出

    float num[3];           // (中间量)传递函数分子系数
    float den[2];           // (中间量)传递函数分母系数
}FILTER_STRUCT;
#endif // CONFIG_Filter

typedef struct{
    FILTER_STRUCT filter;   // (结构体)滤波器运算数据

    float T;                // (系统时钟)T离散周期
    float Wc;               // (参数设计)Wc截止频率
}LPF_STRUCT;

void LPF_Init(LPF_STRUCT *bpf);
void LPF_Loop(LPF_STRUCT *bpf);


#endif // SGUAN_FILTER_H

#ifndef __SGUAN_FILTER_H
#define __SGUAN_FILTER_H

/* SguanFOC配置文件声明 */
#include "Sguan_Config.h"

typedef struct{
    float i[3];             // (数据)历史输入值
    float o[3];             // (数据)历史输出值

    float Input;            // (数据)Input输入
    float Output;           // (数据)Output输出

    float num[3];           // (中间量)传递函数分子系数
    float den[3];           // (中间量)传递函数分母系数
}FILTER_STRUCT;

typedef struct{
    FILTER_STRUCT filter;   // (结构体)二阶低通结构体

    double Wc;              // (参数设计)Wc巴特沃斯截止频率
    double T;               // (参数设计)T周期
}LPF_STRUCT;


// ============================ Q31 版本代码 ============================

typedef struct{
    Q31_t i[3];             // (数据)历史输入值
    Q31_t o[3];             // (数据)历史输出值

    Q31_t Input;            // (数据)Input输入
    Q31_t Output;           // (数据)Output输出

    Q31_t num[3];           // (中间量)传递函数分子系数
    Q31_t den[2];           // (中间量)传递函数分母系数
}FILTER_STRUCT_q31;

typedef struct{
    FILTER_STRUCT_q31 filter; // (结构体)二阶低通结构体

    double Wc;              // (参数设计)Wc巴特沃斯截止频率
    double T;               // (参数设计)T周期
}LPF_STRUCT_q31;

void LPF_Init(LPF_STRUCT *bpf);
void LPF_Loop(LPF_STRUCT *bpf);

uint8_t LPF_Init_q31(LPF_STRUCT_q31 *bpf);
void LPF_Loop_q31(LPF_STRUCT_q31 *bpf);


#endif // SGUAN_FILTER_H

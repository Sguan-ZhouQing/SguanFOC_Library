#ifndef __SGUAN_FILTER_H
#define __SGUAN_FILTER_H

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
}BPF_STRUCT;

void BPF_Init(BPF_STRUCT *bpf);
void BPF_Loop(BPF_STRUCT *bpf);


#endif // SGUAN_FILTER_H

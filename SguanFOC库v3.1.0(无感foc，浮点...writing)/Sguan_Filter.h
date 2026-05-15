#ifndef __SGUAN_FILTER_H
#define __SGUAN_FILTER_H

/* SguanFOC配置文件声明 */
#include "Sguan_Config.h"

// =========================== LPF低通滤波器 ========================
typedef struct{
    float i[2];             // (数据)历史输入值
    float o[2];             // (数据)历史输出值

    float Input;            // (数据)Input输入
    float Output;           // (数据)Output输出

    float num[3];           // (中间量)传递函数分子系数
    float den[2];           // (中间量)传递函数分母系数
}LPF_GO_STRUCT;

typedef struct{
    LPF_GO_STRUCT filter;   // (结构体)滤波器运算数据

    float T;                // (系统时钟)T离散周期
    float Wc;               // (参数设计)Wc截止频率
}LPF_STRUCT;

void LPF_Init(LPF_STRUCT *lpf);
void LPF_Loop(LPF_STRUCT *lpf);


// =========================== BSF陷波滤波器 ========================
typedef struct{
    float Input;            // (数据)Input输入
    float Output;           // (数据)Output输出
}BSF_GO_STRUCT;

typedef struct{
    BSF_GO_STRUCT filter;   // ()

    float T;                // (系统时钟)T离散周期
    float Wo;               // (参数设计)Wo陷波频率
}BSF_STRUCT;

void BSF_Init(BSF_STRUCT *bsf);
void BSF_Loop(BSF_STRUCT *bsf);


// =========================== BPF带通滤波器 ========================
typedef struct{
    float Input;            // (数据)Input输入
    float Output;           // (数据)Output输出
}BPF_GO_STRUCT;

typedef struct{
    BPF_GO_STRUCT filter;   // ()

    float T;                // (系统时钟)T离散周期
    float Wo;               // (参数设计)Wo带通频率
}BPF_STRUCT;

void BPF_Init(BPF_STRUCT *bpf);
void BPF_Loop(BPF_STRUCT *bpf);


// =========================== APF全通滤波器 ========================
typedef struct{
    float Input;            // (数据)Input输入
    float Output;           // (数据)Output输出
}APF_GO_STRUCT;

typedef struct{
    APF_GO_STRUCT filter;   // ()

    float T;                // (系统时钟)T离散周期
    float Wn;               // (参数设计)Wn特征频率
}APF_STRUCT;

void APF_Init(APF_STRUCT *apf);
void APF_Loop(APF_STRUCT *apf);


#endif // SGUAN_FILTER_H

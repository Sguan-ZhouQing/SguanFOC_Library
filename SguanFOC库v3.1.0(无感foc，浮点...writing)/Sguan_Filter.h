#ifndef __SGUAN_FILTER_H
#define __SGUAN_FILTER_H

/* SguanFOC配置文件声明 */
#include "Sguan_Config.h"

// =========================== LPF低通滤波器 ========================
typedef struct{
    float i[2];             // (数据)历史输入值
    float o[2];             // (数据)历史输出值

    float Input;            // (输入数据)Input输入
    float Output;           // (输出数据)Output输出

    float num[2];           // (中间量)传递函数分子系数
    float den[2];           // (中间量)传递函数分母系数
}LPF_GO_STRUCT;

typedef struct{
    LPF_GO_STRUCT filter;   // (结构体)滤波器运算数据

    float T;                // (系统时钟)T离散周期
    float Wc;               // (参数设计)Wc截止频率
}LPF_STRUCT;

void LPF_Init(LPF_STRUCT *lpf);
void LPF_Loop(LPF_STRUCT *lpf);


// ======================== TPNF三参数陷波滤波器 =======================
typedef struct{
    float i[2];             // (数据)历史输入值
    float o[2];             // (数据)历史输出值

    float Input;            // (输入数据)Input输入
    float Output;           // (输出数据)Output输出

    float Gain0;            // (中间量)增益系数
    float Gain1;            // (中间量)增益系数
    float Gain2;            // (中间量)增益系数
}TPNF_GO_STRUCT;

typedef struct{
    TPNF_GO_STRUCT filter;  // (结构体)滤波器运算数据

    float T;                // (系统时钟)T离散周期

    float Wo;               // (参数设计)Wo陷波频率
    float K1;               // (参数设计)K1分母阻尼比
    float K2;               // (参数设计)K2分子阻尼比
}TPNF_STRUCT;

void TPNF_Init(TPNF_STRUCT *tpnf);
void TPNF_Loop(TPNF_STRUCT *tpnf);


#endif // SGUAN_FILTER_H

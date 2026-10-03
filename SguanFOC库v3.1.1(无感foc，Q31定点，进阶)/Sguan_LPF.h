#ifndef __SGUAN_LPF_H
#define __SGUAN_LPF_H

/* SguanFOC配置文件声明 */
#include "Sguan_Config.h"

typedef struct{
    Q31_t Input;            // (输入数据)Input输入
    Q31_t Output;           // (输出数据)Output输出

    Q31_t num;              // (中间量)传递函数分子系数
    Q31_t den;              // (中间量)传递函数分母系数
}__LPF_GO_STRUCT;

typedef struct{
    __LPF_GO_STRUCT go;     // (结构体)滤波器运算数据

    float T;                // (参数设计)离散时间
    float Wc;               // (参数设计)截止频率
}LPF_STRUCT;

void LPF_Init(LPF_STRUCT *lpf);
void LPF_Loop(LPF_STRUCT *lpf);


#endif // SGUAN_LPF_H

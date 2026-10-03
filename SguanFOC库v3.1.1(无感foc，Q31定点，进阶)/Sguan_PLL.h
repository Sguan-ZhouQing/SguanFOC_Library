#ifndef __SGUAN_PLL_H
#define __SGUAN_PLL_H

/* SguanFOC配置文件声明 */
#include "Sguan_Config.h"

typedef struct{
    Q31_t Io;               // PI控制器积分项

    Q31_t Error;            // (输入数据)Error真实反馈数据
    Q31_t OutWe;            // (输出数据)OutWe电角速度输出
    Q31_t OutRe;            // (输出数据)OutRe电角度输出

    Q31_t T;                // (Q31)T运算离散周期

    Q31_t Kp;               // (Q31)Kp比例项增益
    Q31_t Ki;               // (Q31)Ki积分项增益
}__PLL_GO_STRUCT;

typedef struct{
    __PLL_GO_STRUCT go;     // (结构体)PID运算结构体

    float T;                // (系统时钟)T运算离散周期
    
    float Kp;               // (参数设计)Kp比例项增益
    float Ki;               // (参数设计)Ki积分项增益
}PLL_STRUCT;

void PLL_Init(PLL_STRUCT *pll);
void PLL_Loop(PLL_STRUCT *pll);


#endif // SGUAN_PLL_H

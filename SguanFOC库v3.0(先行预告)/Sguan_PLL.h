#ifndef __SGUAN_PLL_H
#define __SGUAN_PLL_H

/* 外部函数声明 */
#include "Sguan_Calculate.h"

typedef struct{
    float i;        // (数据)数据历史输入值
    float Xo;       // (数据)PI历史输出值
    float Yo;       // (数据)积分历史输出值
    float X_num[2]; // (中间量)PI传递函数分子系数

    float Error;    // (输入数据)Error真实反馈数据
    float OutWe;    // (输出数据)OutWe电角速度输出
    float OutRe;    // (输出数据)OutRe电角度输出
}GO_STRUCT;

typedef struct{
    GO_STRUCT go;  // (结构体)PID运算结构体

    double T;      // (参数设计)T运算离散周期
    double Kp;     // (参数设计)Kp比例项增益
    double Ki;     // (参数设计)Ki积分项增益

    uint8_t is_position_mode; // (参数设计)位置环模式标志位
}PLL_STRUCT;

void PLL_Init(PLL_STRUCT *pll);
void PLL_Loop(PLL_STRUCT *pll);


#endif // SGUAN_PLL_H

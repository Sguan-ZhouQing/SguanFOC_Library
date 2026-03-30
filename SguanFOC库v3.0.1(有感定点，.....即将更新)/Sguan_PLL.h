#ifndef __SGUAN_PLL_H
#define __SGUAN_PLL_H

/* SguanFOC配置文件声明 */
#include "Sguan_Config.h"

typedef struct{
    float i;                // (数据)数据历史输入值
    float Xo;               // (数据)PI历史输出值
    float Yo;               // (数据)积分历史输出值
    float X_num[2];         // (中间量)PI传递函数分子系数
    float Y_num;            // (中间量)积分项分母系数

    float Error;            // (输入数据)Error真实反馈数据
    float OutWe;            // (输出数据)OutWe电角速度输出
    float OutRe;            // (输出数据)OutRe电角度输出
}GO_STRUCT;

typedef struct{
    GO_STRUCT go;           // (结构体)PID运算结构体

    double T;               // (参数设计)T运算离散周期
    double Kp;              // (参数设计)Kp比例项增益
    double Ki;              // (参数设计)Ki积分项增益

    uint8_t is_position_mode; // (参数设计)位置环模式标志位
}PLL_STRUCT;


// ============================ Q31 版本代码 ============================

typedef struct{
    Q31_t i;                // (数据)数据历史输入值
    Q31_t Xo;               // (数据)PI历史输出值
    Q31_t Yo;               // (数据)积分历史输出值
    Q31_t X_num[2];         // (中间量)PI传递函数分子系数
    Q31_t Y_num;            // (中间量)积分项分母系数

    Q31_t Error;            // (输入数据)Error真实反馈数据
    Q31_t OutWe;            // (输出数据)OutWe电角速度输出
    Q31_t OutRe;            // (输出数据)OutRe电角度输出
}GO_STRUCT_q31;

typedef struct{
    GO_STRUCT_q31 go;       // (结构体)PID运算结构体

    double T;               // (参数设计)T运算离散周期
    double Kp;              // (参数设计)Kp比例项增益
    double Ki;              // (参数设计)Ki积分项增益

    uint8_t is_position_mode; // (参数设计)位置环模式标志位
}PLL_STRUCT_q31;

void PLL_Init(PLL_STRUCT *pll);
void PLL_Loop(PLL_STRUCT *pll);

void PLL_Init_q31(PLL_STRUCT_q31 *pll);
void PLL_Loop_q31(PLL_STRUCT_q31 *pll);


#endif // SGUAN_PLL_H

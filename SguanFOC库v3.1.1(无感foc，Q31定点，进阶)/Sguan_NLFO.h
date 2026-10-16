#ifndef __SGUAN_NLFO_H
#define __SGUAN_NLFO_H

/* SguanFOC配置文件声明 */
#include "Sguan_Config.h"

// typedef struct{
//     float alpha_o;          // (数据)alpha轴积分历史输出值
//     float beta_o;           // (数据)beta轴积分历史输出值

//     float Flux_pow;         // (中间量)电机磁链平方
//     float Flux_inv;         // (中间量)电机磁链倒数

//     Q31_t Input_Ialpha;     // (输入数据)alpha轴电流
//     Q31_t Input_Ibeta;      // (输入数据)beta轴电流
//     Q31_t Input_Ualpha;     // (输入数据)alpha轴电压
//     Q31_t Input_Ubeta;      // (输入数据)beta轴电压
//     Q31_t Output_Sine;      // (输出数据)Sine角度信息
//     Q31_t Output_Cosine;    // (输出数据)Cosine角度信息

//     float Input_Ialpha_f;     // (输入数据)alpha轴电流
//     float Input_Ibeta_f;      // (输入数据)beta轴电流
//     float Input_Ualpha_f;     // (输入数据)alpha轴电压
//     float Input_Ubeta_f;      // (输入数据)beta轴电压
//     float Output_Sine_f;      // (输出数据)Sine角度信息
//     float Output_Cosine_f;    // (输出数据)Cosine角度信息  

//     float T;                // (Q31)离散周期
//     float Rs;               // (Q31)电机相电阻
//     float Ls;               // (Q31)电机平均电感

//     float Gain;             // (Q31)磁链观测解调增益
// }__NLFO_GO_STRUCT;

// typedef struct{
//     __NLFO_GO_STRUCT go;      // (结构体)非线性磁链运算数据

//     float T;                // (系统时钟)离散周期
    
//     float Flux;             // (电机参数)磁链
//     float Rs;               // (电机参数)电机相电阻
//     float Ls;               // (中间量)电机平均电感

//     float Gain;             // (参数设计)磁链观测解调增益
// }NLFO_STRUCT;

// .....................................................................
typedef struct{
    Q31_t alpha_o;          // (数据)alpha轴积分历史输出值
    Q31_t beta_o;           // (数据)beta轴积分历史输出值

    Q31_t Flux_pow;         // (中间量)电机磁链平方
    Q31_t Flux_inv;         // (中间量)电机磁链倒数

    Q31_t Input_Ialpha;     // (输入数据)alpha轴电流
    Q31_t Input_Ibeta;      // (输入数据)beta轴电流
    Q31_t Input_Ualpha;     // (输入数据)alpha轴电压
    Q31_t Input_Ubeta;      // (输入数据)beta轴电压
    Q31_t Output_Sine;      // (输出数据)Sine角度信息
    Q31_t Output_Cosine;    // (输出数据)Cosine角度信息

    Q31_t T;                // (Q31)离散周期
    Q31_t Rs;               // (Q31)电机相电阻
    Q31_t Ls;               // (Q31)电机平均电感

    Q31_t Gain;             // (Q31)磁链观测解调增益
}__NLFO_GO_STRUCT;

typedef struct{
    __NLFO_GO_STRUCT go;      // (结构体)非线性磁链运算数据

    float T;                // (系统时钟)离散周期
    
    float Flux;             // (电机参数)磁链
    float Rs;               // (电机参数)电机相电阻
    float Ls;               // (中间量)电机平均电感

    float Gain;             // (参数设计)磁链观测解调增益
}NLFO_STRUCT;

void NLFO_Init(NLFO_STRUCT *nlfo);
void NLFO_Loop(NLFO_STRUCT *nlfo);


#endif // SGUAN_NLFO_H

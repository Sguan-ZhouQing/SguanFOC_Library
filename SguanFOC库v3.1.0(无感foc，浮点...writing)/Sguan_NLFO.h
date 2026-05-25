#ifndef __SGUAN_NLFO_H
#define __SGUAN_NLFO_H

/* SguanFOC配置文件声明 */
#include "Sguan_Config.h"

typedef struct{
    float I_num;            // (中间量)积分项传递函数
    float alpha_i;          // (数据)alpha轴积分历史输入值
    float beta_i;           // (数据)beta轴积分历史输入值
    float alpha_o;          // (数据)alpha轴积分历史输出值
    float beta_o;           // (数据)beta轴积分历史输出值

    float Flux_pow;         // (中间量)电机磁链平方
    float Flux_inv;         // (中间量)电机磁链倒数
    float Ls;               // (中间量)电机平均电感

    float Input_Ialpha;     // (输入数据)alpha轴电流
    float Input_Ibeta;      // (输入数据)beta轴电流
    float Input_Ualpha;     // (输入数据)alpha轴电压
    float Input_Ubeta;      // (输入数据)beta轴电压
    float Output_Sine;      // (输出数据)Sine角度信息
    float Output_Cosine;    // (输出数据)Cosine角度信息
}NLFO_GO_STRUCT;

typedef struct{
    NLFO_GO_STRUCT go;      // (结构体)非线性磁链运算数据

    float T;                // (系统时钟)离散周期
    
    float Flux;             // (电机参数)磁链
    float Rs;               // (电机参数)电机相电阻
    float Ld;               // (电机参数)电机D轴电感
    float Lq;               // (电机参数)电机Q轴电感

    float Gain;             // (参数设计)观测器增益
}NLFO_STRUCT;

void NLFO_Init(NLFO_STRUCT *nlfo);
void NLFO_Loop(NLFO_STRUCT *nlfo);


#endif // SGUAN_NLFO_H

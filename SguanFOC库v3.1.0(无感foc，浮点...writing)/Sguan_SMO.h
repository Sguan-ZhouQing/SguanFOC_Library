#ifndef __SGUAN_SMO_H
#define __SGUAN_SMO_H

/* SguanFOC配置文件声明 */
#include "Sguan_Config.h"

typedef struct{
    float I_i;              // (数据)积分项的历史输入
    float F_i;              // (数据)滤波项的历史输入
    float F_o;              // (数据)滤波项的历史输出

    float Input_Ix;         // (输入数据)x轴的电流值_实际
    float Input_Ux;         // (输入数据)x轴的电压值_实际
    float Input_Iy;         // (输入数据)y轴的电流值_预测
    float Output_Ex;        // (输出数据)输出x轴的反电动势_预测
    float Output_Ix;        // (输出数据)输出x轴的轴向电流_预测

    uint8_t IntegralFrozen_flag; // (中间量)积分抗饱和
}SMO_GO_STRUCT;

typedef struct{
    float I_num;            // (中间量)积分项的传递函数分子
    float F_num;            // (中间量)滤波项的传递函数分子
    float F_den;            // (中间量)滤波项的传递函数分母

    float Gain0;            // (中间量)滑模增益1/Ld
    float Gain1;            // (中间量)滑模增益Rs/Ld
    float Gain2;            // (中间量)滑模增益(Ld-Lq)/Ld

    float Input_We;         // (输入数据)电机电角速度_预测
}SMO_DATA_STRUCT;

typedef struct{
    SMO_GO_STRUCT alpha;    // (结构体)轴的反电动势运算数据
    SMO_GO_STRUCT beta;     // (结构体)轴的反电动势运算数据
    SMO_DATA_STRUCT data;   // (结构体)控制器自动计算的参数

    float T;                // (系统时钟)离散周期

    float Rs;               // (电机参数)电机相电阻
    float Ld;               // (电机参数)电机D轴电感
    float Lq;               // (电机参数)电机Q轴电感
    
    float Wc;               // (参数设计)低通滤波截止频率
    float h;                // (参数设计)滑模观测器增益

    float IntMax;           // (参数设计)共用的积分项限幅
    float IntMin;           // (参数设计)共用的积分项限幅
}SMO_STRUCT;

void SMO_Init(SMO_STRUCT *smo);
void SMO_Loop(SMO_STRUCT *smo);


#endif // SGUAN_SMO_H

#ifndef __SGUAN_SMC_H
#define __SGUAN_SMC_H

/* SguanFOC配置文件声明 */
#include "stdint.h"

typedef struct{
    float I_i;              // (数据)积分项历史输入
    float D_i;              // (数据)微分项历史输入

    float I_num;            // (中间量)积分的传递函数分子系数
    float D_num;            // (中间量)微分的传递函数分子系数
    float D_den;            // (中间量)微分的传递函数分母系数

    float Ref;              // (输入数据)Target期望数值
    float Fbk;              // (输出数据)Real真实反馈数据
    float Output;           // (输出数据)Output输出

    uint8_t IntegralFrozen_flag; // (中间量)积分抗饱和
}TRADITION_STRUCT;

typedef struct{
    TRADITION_STRUCT run;   // (结构体)传统滑模控制
    
    float T;                // (参数设计)离散周期
    
    float miu;              // (参数设计)不连续控制增益
    float q;                // (参数设计)切换项增益
    float C;                // (参数设计)动态响应增益
    float Gain;             // (参数设计)电机输入增益

    float IntMax;           // (参数设计)积分项上限,即输出限幅
    float IntMin;           // (参数设计)积分项下限,即输出限幅
}SMC_STRUCT;

void SMC_Init(SMC_STRUCT *smc);
void SMC_Loop(SMC_STRUCT *smc);


#endif // SGUAN_SMC_H

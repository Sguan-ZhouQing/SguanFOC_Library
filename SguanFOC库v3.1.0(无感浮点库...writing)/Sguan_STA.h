#ifndef __SGUAN_STA_H
#define __SGUAN_STA_H

/* SguanFOC配置文件声明 */
#include "Sguan_Config.h"

typedef struct{
    float s[2];             // (数据)滑模面当前值
    float Io;            // (数据)历史积分值
    float I_num;            // (数据)历史积分值
    
    float Ref;              // (输入数据)期望输入值
    float Fbk;              // (输入数据)真实反馈值
    float Output;           // (输出数据)控制器输出

    uint8_t IntegralFrozen_flag; // (中间量)积分抗饱和
}STA_RUN_STRUCT;

typedef struct{
    STA_RUN_STRUCT run;     // (结构体)STA运算结构体
    
    float k1;               // (参数设计)非线性增益
    double k2;              // (参数设计)积分增益
    double T;               // (参数设计)积分步长_采样周期
    
    float boundary;         // (参数设计)边界层厚度(用于饱和函数)
    
    float OutMax;           // (参数设计)输出上限限幅
    float OutMin;           // (参数设计)输出下限限幅
    
    float IntMax;           // (参数设计)积分项上限限幅
    float IntMin;           // (参数设计)积分项下限限幅
}STA_STRUCT;

void STA_Init(STA_STRUCT *sta);
void STA_Loop(STA_STRUCT *sta);


#endif // SGUAN_STA_H

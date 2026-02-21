#ifndef __SGUAN_INTERNALMODEL_H
#define __SGUAN_INTERNALMODEL_H

typedef struct{
    float i;                // (数据)历史输入值
    float o;                // (数据)历史输出值
    float den[2];           // (中间量)传递函数分母系数

    float Input;            // (数据)Input输入
    float Output;           // (数据)Output输出
}IMC_STRUCT;

typedef struct{
    IMC_STRUCT imc;         // (结构体)内模控制器结构体
    double T;               // (参数设计)T周期
}INTERNALMODEL_STRUCT;

void InternalModel_Init(INTERNALMODEL_STRUCT *im);
void InternalModel_Loop(INTERNALMODEL_STRUCT *im);


#endif // SGUAN_INTERNALMODEL_H

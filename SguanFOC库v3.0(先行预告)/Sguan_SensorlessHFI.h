#ifndef __SGUAN_SENSORLESSHFI_H
#define __SGUAN_SENSORLESSHFI_H

/* 外部函数声明 */
#include "Sguan_Calculate.h"

typedef struct{
    float alpha[2];         // (数据)电机alpha轴历史输入
    float beta[2];          // (数据)电机beta轴历史输入

    float Input_a;          // (输入数据)当前alpha轴电流
    float Input_b;          // (输入数据)当前beta轴电流
    
    float alpha_h;          // (输出数据)高频注入alpha轴高频分量
    float beta_h;           // (输出数据)高频注入beta轴高频分量
    float alpha_f;          // (输出数据)电机此时alpha轴基频分量
    float beta_f;           // (输出数据)电机此时beta轴基频分量

    float D_h;              // (输出数据)D轴高频分量
    float Q_h;              // (输出数据)Q轴高频分量
    float D_f;              // (输出数据)D轴基频分量
    float Q_f;              // (输出数据)Q轴基频分量
}HFI_STRUCT;

float HFI_ToggleCurrent(float Ud_Bias, float Ud_Limit);
void HFI_ReadCurrent(HFI_STRUCT *hfi);



#endif // SGUAN_SENSORLESSHFI_H

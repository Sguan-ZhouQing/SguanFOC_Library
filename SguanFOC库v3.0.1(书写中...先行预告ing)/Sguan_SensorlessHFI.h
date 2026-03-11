#ifndef __SGUAN_SENSORLESSHFI_H
#define __SGUAN_SENSORLESSHFI_H

/* 外部函数声明 */
#include "Sguan_Calculate.h"

typedef struct{
    float i[2];                 // (数据)电机x轴历史输入

    float Input;                // (输入数据)当前轴线的电流值
    float Output;               // (输出数据)电流高/基频分量
}HFI_STRUCT;

float HFI_ToggleVBUS(float Ud_Bias, float Ud_Limit);
void HFI_ReadHighCurrent(HFI_STRUCT *hfi);
void HFI_ReadFundamentalCurrent(HFI_STRUCT *hfi);


#endif // SGUAN_SENSORLESSHFI_H

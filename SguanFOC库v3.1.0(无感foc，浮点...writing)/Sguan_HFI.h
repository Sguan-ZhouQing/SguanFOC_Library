#ifndef __SGUAN_HFI_H
#define __SGUAN_HFI_H

/* SguanFOC配置文件声明 */
#include "Sguan_Config.h"

typedef struct{
    float i[2];             // (数据)电机x轴历史输入数值

    float Input;            // (输入数据)当前轴的电流数值
    float Output;           // (输出数据)电流高/基频分量
}HFI_GO_STRUCT;

typedef struct{
    float Last_Ialpha;      // (中间量)历史输出参数->角度解耦
    float Last_Ibeta;       // (中间量)历史输出参数->角度解耦
    float Uin;              // (中间量)电机高频方波信号
    
    float Input_VBUS;       // (输入数据)电机母线电压数值
    float Input_Ud;         // (输入数据)电机D轴电压基频值
    float Output_Iah;       // (输出数据)电机实际采样的电流值
    float Output_Ibh;       // (输出数据)电机实际采样的电流值
    float Output_Ud;        // (输出数据)电机D轴电压高频值
}HFI_DATA_STRUCT;

typedef struct{
    HFI_GO_STRUCT Id_f;     // (结构体)基频分量->D轴电流环反馈值
    HFI_GO_STRUCT Iq_f;     // (结构体)基频分量->Q轴电流环反馈值
    HFI_GO_STRUCT Ialpha_h; // (结构体)高频分量->角度解耦
    HFI_GO_STRUCT Ibeta_h;  // (结构体)高频分量->角度解耦
    HFI_DATA_STRUCT data;   // (结构体)高频数据->角度解耦

    float Percentage_hfi;   // (参数设计)注入电压“母线电压”的占比
}HFI_STRUCT;

void HFI_Init(HFI_STRUCT *hfi);
void HFI_ReadRad_Loop(HFI_STRUCT *hfi);
void HFI_Current_Loop(HFI_STRUCT *hfi);


#endif // SGUAN_HFI_H

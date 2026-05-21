#ifndef __SGUAN_HFI_H
#define __SGUAN_HFI_H

/* SguanFOC配置文件声明 */
#include "Sguan_Config.h"

// typedef struct{
//     float i[2];             // (数据)电机x轴历史输入数值

//     float Input;            // (输入数据)当前轴的电流数值
//     float Output;           // (输出数据)电流高/基频分量
// }HFI_GO_STRUCT;

// typedef struct{
//     float Last_Ialpha;      // (中间量)历史输出参数->角度解耦
//     float Last_Ibeta;       // (中间量)历史输出参数->角度解耦
//     float Uin;              // (中间量)电机高频方波信号
    
//     float Input_VBUS;       // (输入数据)电机母线电压数值
//     float Input_Ud;         // (输入数据)电机D轴电压基频值
//     float Output_Iah;       // (输出数据)电机实际采样的电流值
//     float Output_Ibh;       // (输出数据)电机实际采样的电流值
//     float Output_Ud;        // (输出数据)电机D轴电压高频值
// }HFI_DATA_STRUCT;


typedef struct{
    float Angle;            // (中间量)注入的角度数值变化
    float Sine;             // (中间量)注入的原始正弦信号

    float Input_Id;         // (输入数据)原始D轴电流信号
    float Input_Iq;         // (输入数据)原始Q轴电流信号
    float Output_Id;        // (输出数据)基频D轴电流
    float Output_Iq;        // (输出数据)基频Q轴电流

    float Input_Ialpha;     // (输入数据)原始alpha轴电流信号
    float Input_Ibeta;      // (输入数据)原始beta轴的电流信号
    float Output_Ialpha;    // (输出数据)高频alpha轴电流
    float Output_Ibeta;     // (输出数据)高频beta轴的电流

    float Output_Uin;       // (输出数据)高频正弦电压D轴叠加量

    float s_num[2];           // (中间量)陷波滤波器->传递函数分子系数
    float s_den[2];           // (中间量)陷波滤波器->传递函数分母系数

    float p_num;              // (中间量)带通滤波器->传递函数分子系数
    float p_den[2];           // (中间量)带通滤波器->传递函数分母系数
}HFI_GO_STRUCT;

typedef struct{
    float s_i[2];             // (数据)陷波滤波器->历史输入值
    float s_o;                // (数据)陷波滤波器->历史输出值

    float p_i[2];             // (数据)带通滤波器->历史输入值
    float p_o[2];             // (数据)带通滤波器->历史输出值
}HFI_DATA_STRUCT;

typedef struct{
    HFI_GO_STRUCT go;       // (结构体)高频正弦波注入运算
    HFI_DATA_STRUCT data0;  // (结构体)运算中间量
    HFI_DATA_STRUCT data1;  // (结构体)运算中将量

    float T;                // (系统时钟)T离散周期

    float Wo;               // (参赛设计)注入正弦波频率
    float h;                // (参赛设计)高频正弦波注入增益
    float Uh;               // (参数设计)注入电压幅值

    float zeta0;            // (参数设计)陷波滤波器->zeta阻尼比
    float zeta1;            // (参数设计)带通滤波器->zeta阻尼比
}HFI_STRUCT;

void HFI_Init(HFI_STRUCT *hfi);
void HFI_ReadRad_Loop(HFI_STRUCT *hfi);
void HFI_Current_Loop(HFI_STRUCT *hfi);


#endif // SGUAN_HFI_H

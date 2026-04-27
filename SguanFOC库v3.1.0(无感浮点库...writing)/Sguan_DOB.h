#ifndef __SGUAN_DOB_H
#define __SGUAN_DOB_H

/* SguanFOC配置文件声明 */
#include "Sguan_Config.h"

typedef struct{
    float I_num;            // (中间量)积分项传递函数分子系数
    float O_num;            // (中间量)输出项传递函数分子系数
    float O_den;            // (中间量)输出项传递函数分母系数

    float Fd_i;             // (数值)历史的Fd输入值
    float Wm_i;             // (数据)历史的Wm输入值

    float Gain0;            // (中间量)数据增益
    float Gain1;            // (中间量)数据增益

    float Input_Iq;         // (输入数据)输真实的Q轴电流
    float Input_Wm;         // (输入数据)真实的机械角速度
    float Output_Fd;        // (输出数据)预估扰动转矩
    float Output_Wm;        // (输出数据)预估的机械角速度
}SMDO_STRUCT;

typedef struct{
    SMDO_STRUCT smdo;       // (结构体)超螺旋滑模DOB
    
    uint8_t Pn;             // (电机数据)极对数
    float Flux;             // (电机数据)磁链
    float B;                // (电机数据)粘性阻尼
    float J;                // (电机数据)转动惯量
    
    float T;                // (参数设计)离散周期
    float K1;               // (参数设计)比例项增益
    float K2;               // (参数设计)积分项增益

    float OutMax_Fd;        // (参数设计)输出上限限幅
    float OutMin_Fd;        // (参数设计)输出下限限幅
    float OutMax_Wm;        // (参数设计)输出下限限幅
    float OutMin_Wm;        // (参数设计)输出下限限幅
}DOB_STRUCT;

void DOB_Init(DOB_STRUCT *dob);
void DOB_Loop(DOB_STRUCT *dob);


#endif // SGUAN_DOB_H

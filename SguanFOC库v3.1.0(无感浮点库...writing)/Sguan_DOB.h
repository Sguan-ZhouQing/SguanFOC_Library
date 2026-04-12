#ifndef __SGUAN_DOB_H
#define __SGUAN_DOB_H

/* SguanFOC配置文件声明 */
#include "Sguan_Config.h"

typedef struct{
    float Fd_i;             // (数据)Fd的积分数值输入项
    float Wm_i;             // (数据)Wm的积分数值输入项
    float Fd_o;             // (数据)Last_Time的Fd数值
    float Wm_o;             // (数据)Last_Time的Wm数值
    float num;              // (中间量)积分传递函数分子系数

    float Gain0;            // (中间量)输入增益
    float Gain1;            // (中间量)输入增益

    float Input_Iq;         // (输入数据)输真实的Q轴电流
    float Input_Wm;         // (输入数据)真实的机械角速度
    float Output_Fd;        // (输出数据)预估扰动转矩
    float Output_Wm;        // (输出数据)预估的机械角速度
}SMDO_STRUCT;

typedef struct{
    SMDO_STRUCT smdo;       // (结构体)超螺旋滑模DOB
    
    uint8_t Pn;             // (电机数据)极对数
    double Flux;            // (电机数据)磁链
    double B;               // (电机数据)粘性阻尼
    double J;               // (电机数据)转动惯量
    
    double T;               // (参数设计)离散周期
    float K1;               // (参数设计)比例项增益
    float K2;               // (参数设计)积分项增益

    float OutMax;           // (参数设计)输出上限限幅
    float OutMin;           // (参数设计)输出下限限幅
}DOB_STRUCT;

void DOB_Init(DOB_STRUCT *dob);
void DOB_Loop(DOB_STRUCT *dob);


#endif // SGUAN_DOB_H

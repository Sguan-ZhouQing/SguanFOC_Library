#ifndef __SGUAN_IDENTIFY_H
#define __SGUAN_IDENTIFY_H

/* SguanFOC配置文件声明 */
#include "Sguan_Config.h"

typedef struct{
    float Input_Id;         // (输入数据)D轴电流数值
    float Input_Iq;         // (输入数据)Q轴电流数值
    float Input_We;         // (输入数据)电机电子角速度
    float Output_Ud;        // (输出数据)D轴电压给定
    float Output_Uq;        // (输出数据)Q轴电压给定

    float Set_Uh;           // (参数设计)轴向电压幅值->电阻电感测量
    float Set_Us;           // (参数设计)Q轴电压的幅值->仅磁链测量
    float Set_Delay;        // (参数设计)延时函数的时间->单位ms毫秒

    float Current_h;        // (数据)单位幅值下记录的电流
    uint32_t Time;          // (中间量)电流上升时间
    uint8_t Step;           // (数据)电机参数辨识的标志位
    // (Step为01则开始Rs、Ls(Ld,Lq),2以后结束辨识)
}IDENTIFY_GO_STRUCT;

typedef struct{
    IDENTIFY_GO_STRUCT go;  // (结构体)永磁同步电机运算数据

    float Rs;               // (电机实体参数)相线电阻
    float Ld;               // (电机实体参数)D轴电感
    float Lq;               // (电机实体参数)Q轴电感

    float Flux;             // (电机实体参数)电机磁链
    float B;                // (电机实体参数)粘性阻尼
    float J;                // (电机实体参数)转动惯量
}IDENTIFY_STRUCT;

void Identify_ReadRs_Init(IDENTIFY_STRUCT *identify);
void Identify_ReadFlux_Init(IDENTIFY_STRUCT *identify);
void Identify_ReadLs_Loop(IDENTIFY_STRUCT *identify);


#endif // SGUAN_IDENTIFY_H

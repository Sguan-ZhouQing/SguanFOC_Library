#ifndef __SGUAN_IDENTIFY_H
#define __SGUAN_IDENTIFY_H

/* SguanFOC配置文件声明 */
#include "Sguan_Config.h"

#define MOTOR_IDENTIFY_STANDBY          0x00    // 电机参数待辨识(初始化会进入辨识)
#define MOTOR_IDENTIFY_ENCODER_READING  0x01    // 读取编码器方向(定位0和90度实现)
#define MOTOR_IDENTIFY_RS_READING       0x02    // 读取电机相电阻(Rs = ΔU / ΔI)
#define MOTOR_IDENTIFY_LS_READING       0x03    // 读取电机相电感(L = R * t / ln(20))
#define MOTOR_IDENTIFY_FLUX_READING     0x04    // 读取电机磁链(空载测E0/We,E0=Uq-Iq*Rs)
#define MOTOR_IDENTIFY_B_READING        0x05    // 读取粘性阻尼(空载“电流-转速”斜率法)
#define MOTOR_IDENTIFY_J_READING        0x06    // 读取转动惯量(自由减速法,求J/B,忽略库伦摩擦)
#define MOTOR_IDENTIFY_FINISHED         0x07    // 参数辨识完毕(辨识结束，电机正常运行)

typedef struct{
    uint8_t Status;         // (状态机)电机参数辨识 

    float temp;             // (中间量)
}PMSM_STRUCT;

typedef struct{
    PMSM_STRUCT pmsm;       // (结构体)永磁同步电机运算数据

    int8_t Encoder_Dir;     // (有感实体参数)编码器方向
    float Rs;               // (电机实体参数)相线电阻
    float Ld;               // (电机实体参数)D轴电感
    float Lq;               // (电机实体参数)Q轴电感

    float Flux;             // (电机实体参数)电机磁链
    float B;                // (电机实体参数)粘性阻尼
    float J;                // (电机实体参数)转动惯量
}IDENTIFY_STRUCT;

void Identify_Init(IDENTIFY_STRUCT *identify);
void Identify_Loop(IDENTIFY_STRUCT *identify);


#endif // SGUAN_IDENTIFY_H

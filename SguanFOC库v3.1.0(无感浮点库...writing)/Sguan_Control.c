/*
 * @Author: 星必尘Sguan
 * @GitHub: https://github.com/Sguan-ZhouQing
 * @Date: 2026-04-27 15:31:50
 * @LastEditors: 星必尘Sguan|3464647102@qq.com
 * @LastEditTime: 2026-04-28 01:06:39
 * @FilePath: \SguanFOC_Debug\SguanFOC\Sguan_DOB.c
 * @Description: SguanFOC库的“控制函数”实现
 * 
 * Copyright (c) 2026 by $星必尘Sguan, All Rights Reserved. 
 */
#include "SguanFOC.h"

static void Control_Start(void){
    Sguan.status = MOTOR_STATUS_UNINITIALIZED;
}

static void Control_Stop(void){
    Sguan.status = MOTOR_STATUS_STANDBY;
}

static void Control_SetMode(uint8_t mode){
    Sguan.mode = mode;
}

static void Control_SetUq(float uq){
    Sguan.foc.Uq_in = uq;
}

static void Control_SetIq(float iq){
    Sguan.foc.Target_Iq = iq;
}

static void Control_SetVelocity(float speed){
    Sguan.foc.Target_Speed = speed;
}

static void Control_SetPosition(float pos){
    Sguan.foc.Target_Pos = pos;
}

static void Control_SetTXdata(uint8_t ch,float data){
    Sguan.txdata.fdata[ch] = data;
}

void Control_Init(void *ctrl){
    SguanFOC_System_STRUCT *p = (SguanFOC_System_STRUCT*)ctrl;
    p->Func_Start = Control_Start;
    p->Func_Stop = Control_Stop;
    p->Func_Set_Mode = Control_SetMode;
    p->Func_Set_Uq = Control_SetUq;
    p->Func_Set_Iq = Control_SetIq;
    p->Func_Set_Velocity = Control_SetVelocity;
    p->Func_Set_Position = Control_SetPosition;
    p->Func_Set_TXdata = Control_SetTXdata;
}


/*
 * @Author: 星必尘Sguan
 * @GitHub: https://github.com/Sguan-ZhouQing
 * @Date: 2026-04-30 01:37:34
 * @LastEditors: 星必尘Sguan|3464647102@qq.com
 * @LastEditTime: 2026-04-30 01:56:37
 * @FilePath: \SguanFOC_Debug\SguanFOC\Sguan_Cogging.c
 * @Description: SguanFOC库的“抗齿槽算法(离线标定int16_t的1800点位)”实现
 * 
 * Copyright (c) 2026 by $星必尘Sguan, All Rights Reserved. 
 */
#include "Sguan_Cogging.h"

// 离线标定一圈耗时90秒，总计标定10圈
// 设计1800点位，每点位置给50ms响应时间
#define Cogging_Count   1800
#define Cogging_Cycle   900
static int16_t iq_tab[Cogging_Count] = {0};


void Cogging_Loop(float *Target_Pos,float Uq){
    static uint32_t count = 0;
    static uint32_t time_count = 0;
    static float Add_Pos = 0.0f;
    if (!time_count){
        time_count = (uint32_t)(Cogging_Cycle/(PMSM_RUN_T*Cogging_Count*10.0f));
    }
    if (!Add_Pos){
        Add_Pos = Value_2PI/((float)Cogging_Count);
    }

    count++;
    if (count % time_count == 0){
        float num = Uq;
        *Target_Pos += Add_Pos;
    }
}

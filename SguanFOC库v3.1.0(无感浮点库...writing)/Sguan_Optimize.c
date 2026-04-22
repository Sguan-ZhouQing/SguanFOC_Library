/*
 * @Author: 星必尘Sguan
 * @GitHub: https://github.com/Sguan-ZhouQing
 * @Date: 2026-02-26 22:37:25
 * @LastEditors: 星必尘Sguan|3464647102@qq.com
 * @LastEditTime: 2026-04-19 01:00:46
 * @FilePath: \SguanFOC_Debug\SguanFOC\Sguan_Optimize.c
 * @Description: SguanFOC库的“MTPA和FW弱磁控制的电机优化算法”实现
 * 
 * Copyright (c) 2026 by $星必尘Sguan, All Rights Reserved. 
 */
#include "Sguan_Optimize.h"

// 经典最大转矩电流id计算公式

/**
 * @description: 
 * @param {float} *Target_id
 * @param {float} flux
 * @param {float} Ld
 * @param {float} Lq
 * @param {float} iq
 * @return {*}
 */
void MTPA_Loop(float *Target_id,float flux,float Ld,float Lq,float iq){
    *Target_id = - (flux + Value_sqrtf(flux*flux + 
                8*(Ld - Lq)*(Ld - Lq)*iq*iq))/
                ((Ld - Lq)*4);
}


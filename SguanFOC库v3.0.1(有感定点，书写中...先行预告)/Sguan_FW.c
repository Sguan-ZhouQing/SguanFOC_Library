/*
 * @Author: 星必尘Sguan
 * @GitHub: https://github.com/Sguan-ZhouQing
 * @Date: 2026-02-26 22:37:25
 * @LastEditors: 星必尘Sguan|3464647102@qq.com
 * @LastEditTime: 2026-03-03 00:47:53
 * @FilePath: \stm_SguanFOCtest\SguanFOC\Sguan_Filter.c
 * @Description: SguanFOC库的“FW弱磁控制”实现，转为SPMWM设计
 * 
 * Copyright (c) 2026 by $星必尘Sguan, All Rights Reserved. 
 */
#include "Sguan_FW.h"

void FW_MTPA_Loop(float *Target_id,float flux,float Ld,float Lq,float iq){
    // 经典最大转矩电流id计算公式
    *Target_id = - (flux + Value_sqrtf(flux*flux + 8*(Ld - Lq)*(Ld - Lq)*iq*iq))/((Ld - Lq)*4);
}


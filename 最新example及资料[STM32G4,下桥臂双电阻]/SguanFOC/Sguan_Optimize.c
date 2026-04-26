/*
 * @Author: 星必尘Sguan
 * @GitHub: https://github.com/Sguan-ZhouQing
 * @Date: 2026-02-26 22:37:25
 * @LastEditors: 星必尘Sguan|3464647102@qq.com
 * @LastEditTime: 2026-04-01 03:45:27
 * @FilePath: \SguanFOC_Debug\SguanFOC\Sguan_Optimize.c
 * @Description: SguanFOC库的“MTPA和FW弱磁控制，三次谐波注入”实现
 * 
 * Copyright (c) 2026 by $星必尘Sguan, All Rights Reserved. 
 */
#include "Sguan_Optimize.h"

// 经典最大转矩电流id计算公式
void MTPA_Loop(float *Target_id,float flux,float Ld,float Lq,float iq){
    *Target_id = - (flux + Value_sqrtf(flux*flux + 
                8*(Ld - Lq)*(Ld - Lq)*iq*iq))/
                ((Ld - Lq)*4);
}

void MTPA_Loop_q31(Q31_t *Target_id, Q31_t flux, Q31_t Ld, Q31_t Lq, Q31_t iq){
    Q31_t Ld_minus_Lq;
    Q31_t temp;
    Q31_t sqrt_input;
    
    Ld_minus_Lq = IQmath_Q31_sub(Ld, Lq);
    
    temp = IQmath_Q31_mul(Ld_minus_Lq, Ld_minus_Lq);
    temp = IQmath_Q31_mul(temp, iq);
    temp = IQmath_Q31_mul(temp, iq);
    temp = IQmath_Q31_mul(temp, 8);
    
    sqrt_input = IQmath_Q31_add(IQmath_Q31_mul(flux, flux), temp);
    temp = Value_sqrt_q31(sqrt_input);
    
    temp = IQmath_Q31_add(flux, temp);
    temp = -temp;
    Ld_minus_Lq = IQmath_Q31_mul(Ld_minus_Lq, 4);

    *Target_id = IQmath_Q31_div(temp, Ld_minus_Lq);
}

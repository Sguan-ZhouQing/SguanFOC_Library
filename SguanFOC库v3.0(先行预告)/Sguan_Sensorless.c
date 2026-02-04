/*
 * @Author: 星必尘Sguan
 * @GitHub: https://github.com/Sguan-ZhouQing
 * @Date: 2026-01-26 22:50:59
 * @LastEditors: 星必尘Sguan|3464647102@qq.com
 * @LastEditTime: 2026-02-04 10:56:06
 * @FilePath: \demo_SguanFOCCode\SguanFOC库\Sguan_Sensorless.c
 * @Description: SguanFOC库的“无感控制算法HFI和SMO”实现
 * 
 * Copyright (c) 2026 by $星必尘Sguan, All Rights Reserved. 
 */
#include "Sguan_Sensorless.h"


void HFI_Init(HFI_STRUCT *hfi){

}


void SMO_Init(SMO_STRUCT *smo){
    
}



void HFI_Loop(HFI_STRUCT *hfi){
    float input;
    hfi->z[2] = hfi->z[1];
    hfi->z[1] = hfi->z[0];
    hfi->z[0] = input;
    float output = (hfi->z[0] - 2*hfi->z[1] + hfi->z[2])/4;
}


void NSD_Loop(void){
    
}

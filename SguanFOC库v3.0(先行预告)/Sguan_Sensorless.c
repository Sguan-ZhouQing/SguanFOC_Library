/*
 * @Author: 星必尘Sguan
 * @GitHub: https://github.com/Sguan-ZhouQing
 * @Date: 2026-01-26 22:50:59
 * @LastEditors: 星必尘Sguan|3464647102@qq.com
 * @LastEditTime: 2026-02-19 23:51:04
 * @FilePath: \stm_SguanFOCtest\SguanFOC\Sguan_Sensorless.c
 * @Description: SguanFOC库的“无感控制算法HFI和SMO”实现
 * 
 * Copyright (c) 2026 by $星必尘Sguan, All Rights Reserved. 
 */
#include "Sguan_Sensorless.h"
// 外部函数文件声明
#include "SguanFOC.h"


/* ================== 滑膜观测器的代码实现(SMO) ================== */
void SMO_Init(SMO_STRUCT *smo){
    
}





/* ================= 方波高频注入的代码实现(HFI) ================= */
void HFI_Init(HFI_STRUCT *hfi){

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

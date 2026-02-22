/*
 * @Author: 星必尘Sguan
 * @GitHub: https://github.com/Sguan-ZhouQing
 * @Date: 2026-02-20 23:23:07
 * @LastEditors: 星必尘Sguan|3464647102@qq.com
 * @LastEditTime: 2026-02-20 23:24:21
 * @FilePath: \stm_SguanFOCtest\SguanFOC\Sguan_Current.c
 * @Description: SguanFOC库的“电流计算”实现
 * 
 * Copyright (c) 2026 by $星必尘Sguan, All Rights Reserved. 
 */
#include "Sguan_Current.h"

// 外部函数文件声明
#include "UserData_Function.h"
// 内部函数文件声明
#define Current_Count 24

// 读取PMSM相电流偏置
void Current_OffsetRead(int32_t *offset0,int32_t *offset1){
    for (uint8_t i = 0; i < Current_Count; i++){
        *offset0 += User_ReadADC_Raw(0);
        *offset1 += User_ReadADC_Raw(1);
        User_Delay(2);
    }
    *offset0 = *offset0/Current_Count;
    *offset1 = *offset1/Current_Count;
}


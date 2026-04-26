/*
 * @Author: 星必尘Sguan
 * @GitHub: https://github.com/Sguan-ZhouQing
 * @Date: 2026-04-22 13:27:40
 * @LastEditors: 星必尘Sguan|3464647102@qq.com
 * @LastEditTime: 2026-04-22 15:23:44
 * @FilePath: \SguanFOC_Debug\SguanFOC\Sguan_Hall.c
 * @Description: SguanFOC库的“三霍尔信号处理的函数”实现
 * 
 * Copyright (c) 2026 by $星必尘Sguan, All Rights Reserved. 
 */
#include "Sguan_Hall.h"

/**
 * @description: 霍尔信号处理的运行函数
 * @param {uint8_t} Signal_A
 * @param {uint8_t} Signal_B
 * @param {uint8_t} Signal_C
 * @return {*}
 */
float Hall_Loop(uint8_t Signal_A,uint8_t Signal_B,uint8_t Signal_C){
    // static uint8_t v[6] = {5, 4, 6, 2, 3, 1};
    static uint8_t v[6] = {6, 4, 5, 2, 1, 3};
    uint8_t Sector = (Signal_A << 2) | (Signal_B << 1) | (Signal_C);

}



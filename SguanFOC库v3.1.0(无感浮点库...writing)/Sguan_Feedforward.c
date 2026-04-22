/*
 * @Author: 星必尘Sguan
 * @GitHub: https://github.com/Sguan-ZhouQing
 * @Date: 2026-04-09 16:26:33
 * @LastEditors: 星必尘Sguan|3464647102@qq.com
 * @LastEditTime: 2026-04-19 00:59:53
 * @FilePath: \SguanFOC_Debug\SguanFOC\Sguan_Feedforward.c
 * @Description: SguanFOC库的“前馈环节(提高系统稳定性)”实现
 * 
 * Copyright (c) 2026 by $星必尘Sguan, All Rights Reserved. 
 */
#include "Sguan_Feedforward.h"

// D轴电流前馈

/**
 * @description: 
 * @param {float} Espeed
 * @param {float} Lq
 * @param {float} Iq
 * @return {*}
 */
float Feedforward_CurrentD(float Espeed,float Lq,float Iq){
    return -(Espeed*Lq*Iq);
}

// Q轴电流前馈

/**
 * @description: 
 * @param {float} Espeed
 * @param {float} Ld
 * @param {float} Id
 * @param {float} Flux
 * @return {*}
 */
float Feedforward_CurrentQ(float Espeed,float Ld,float Id,float Flux){
    return Espeed*(Ld*Id + Flux);
}

// 转速环速度前馈

/**
 * @description: 
 * @param {float} Speed
 * @param {float} Ba
 * @return {*}
 */
float Feedforward_Velocity(float Speed,float Ba){
    return -Speed*Ba;
}


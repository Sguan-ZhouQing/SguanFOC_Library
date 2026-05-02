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

/**
 * @description: D轴电流前馈量
 * @param {float} Espeed
 * @param {float} Lq
 * @param {float} Iq
 * @return {*}
 */
float Feedforward_CurrentD(float We,float Lq,float Iq){
    return -(We*Lq*Iq);
}

/**
 * @description: Q轴电流前馈量
 * @param {float} Espeed
 * @param {float} Ld
 * @param {float} Id
 * @param {float} Flux
 * @return {*}
 */
float Feedforward_CurrentQ(float We,float Ld,float Id,float Flux){
    return We*(Ld*Id + Flux);
}

/**
 * @description: 转速环速度前馈量
 * @param {float} Speed
 * @param {float} Ba
 * @return {*}
 */
float Feedforward_Velocity(float Speed,float Ba){
    return -Speed*Ba;
}

/**
 * @description: 扰动观测器前馈量
 * @param {float} Fd
 * @param {float} Pn
 * @param {float} Flux
 * @return {*}
 */
float Feedforward_DOB(float Fd,float Pn,float Flux){
    return Fd/(1.5f*Pn*Flux);
}


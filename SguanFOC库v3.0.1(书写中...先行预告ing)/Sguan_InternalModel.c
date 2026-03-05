/*
 * @Author: 星必尘Sguan
 * @GitHub: https://github.com/Sguan-ZhouQing
 * @Date: 2026-02-20 23:09:47
 * @LastEditors: 星必尘Sguan|3464647102@qq.com
 * @LastEditTime: 2026-02-20 23:13:36
 * @FilePath: \stm_SguanFOCtest\SguanFOC\Sguan_InternalModel.c
 * @Description: SguanFOC库的“内模控制”实现，有利于电流环调优
 * 
 * Copyright (c) 2026 by $星必尘Sguan, All Rights Reserved. 
 */
#include "Sguan_InternalModel.h"

// 内模控制初始化函数
void InternalModel_Init(INTERNALMODEL_STRUCT *im){
    double temp0 = im->Rs*im->T;
    im->imc.den[0] = (float)(2*im->Ls+temp0);
    im->imc.den[1] = (float)(-2*im->Ls+temp0);
    // 初始化为零
    im->imc.i = 0;
    im->imc.o = 0;
    im->imc.Input = 0;
    im->imc.Output = 0;
}

// 定时调用的内模控制实际运行函数
void InternalModel_Loop(INTERNALMODEL_STRUCT *im){
    // 更新当前输入
    im->imc.Output = (im->T*im->imc.Input + im->T*im->imc.i - 
                im->imc.o*im->imc.den[1])/im->imc.den[0];
    // 更新历史输入和输出数值
    im->imc.i = im->imc.Input;
    im->imc.o = im->imc.Output;
}

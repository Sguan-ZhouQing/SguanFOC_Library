/*
 * @Author: 星必尘Sguan
 * @GitHub: https://github.com/Sguan-ZhouQing
 * @Date: 2026-04-29 19:58:03
 * @LastEditors: 星必尘Sguan|3464647102@qq.com
 * @LastEditTime: 2026-06-05 03:51:09
 * @FilePath: \SguanFOC_Debug\SguanFOC\Sguan_SMO.c
 * @Description: SguanFOC库的“SMO无感滑模观测器”实现
 * 
 * Copyright (c) 2026 by $星必尘Sguan, All Rights Reserved. 
 */
#include "Sguan_SMO.h"

// 局部静态函数的声明
static void SMO_RUN(SMO_STRUCT *smo,SMO_GO_STRUCT *run);

// 静止坐标系下的x轴观测器框图（x可以是alpha或者beta轴）
static void SMO_RUN(SMO_STRUCT *smo,SMO_GO_STRUCT *run){
    float I_input,part0,part1,part2,part3;
    part0 = run->Input_Ux*smo->data.Gain0;
    part1 = run->Output_Ix*smo->data.Gain1;
    part2 = smo->data.Input_We*smo->data.Gain2*run->Input_Iy;
    part3 = run->Output_Ex*smo->data.Gain0;
    I_input = part0 - part1 - part2 - part3;

    // 积分x轴，判断是否需要冻结积分
    if (run->IntegralFrozen_flag){
        // 如果积分已冻结，保持上次的积分值
        
        // 检查是否可以解除冻结
        // 情况1：误差反向（误差符号与积分输出符号相反）
        // 情况2：积分值回到限幅范围内
        if ((I_input*run->Output_Ix < 0) ||  // 误差反向
            ((run->Output_Ix < smo->IntMax) && 
            (run->Output_Ix > smo->IntMin))){
            run->IntegralFrozen_flag = 0;
        }
    } else{
        // 正常计算积分
        run->Output_Ix += smo->data.I_num*(I_input + run->I_i);
        
        // 检查是否达到限幅，达到则冻结积分
        if (run->Output_Ix > smo->IntMax){
            run->Output_Ix = smo->IntMax;
            run->IntegralFrozen_flag = 1;
        }
        else if (run->Output_Ix < smo->IntMin){
            run->Output_Ix = smo->IntMin;
            run->IntegralFrozen_flag = 1;
        }
    }

    // 输出x轴的预测反电动势，并滤波
    float error = run->Output_Ix - run->Input_Ix;
    Value_Limit(&error,1.0f,-1.0f);
    float Ex = error*smo->h;
    run->Output_Ex = smo->data.F_num*(Ex + run->F_i) - smo->data.F_den*run->F_o;

    // 更新历史输入输出值
    run->I_i = I_input;
    run->F_i = Ex;
    run->F_o = run->Output_Ex;
}

/**
 * @description: 静止坐标系下的SMO滑模的初始化函数
 * @reminder: (初始化相关系数float->double->float)
 * @reminder: (单浮点转double运算，提高系数精度)
 * @param {SMO_STRUCT} *smo
 * @return {*}
 */
void SMO_Init(SMO_STRUCT *smo){
    double temp0 = ((double)(smo->T))*((double)(smo->Wc));
    double den = 2.0+temp0;

    smo->data.I_num = (float)(((double)(smo->T))/2.0);
    smo->data.F_num = (float)(temp0/den);
    smo->data.F_den = (float)((-2.0+temp0)/den);

    smo->data.Gain0 = (float)(1.0/((double)(smo->Ld)));
    smo->data.Gain1 = (float)(((double)(smo->Rs))/((double)(smo->Ld)));
    smo->data.Gain2 = (float)((((double)(smo->Ld)) - 
                        ((double)(smo->Lq)))/
                        ((double)(smo->Ld)));

    // 初始化为零，alpha轴
    smo->alpha.I_i = 0.0f;
    smo->alpha.F_i = 0.0f;
    smo->alpha.F_o = 0.0f;

    smo->alpha.Input_Ix = 0.0f;
    smo->alpha.Input_Ux = 0.0f;
    smo->alpha.Output_Ex = 0.0f;
    smo->alpha.Output_Ix = 0.0f;

    // 初始化为零，beta轴
    smo->beta.I_i = 0.0f;
    smo->beta.F_i = 0.0f;
    smo->beta.F_o = 0.0f;

    smo->beta.Input_Ix = 0.0f;
    smo->beta.Input_Ux = 0.0f;
    smo->beta.Output_Ex = 0.0f;
    smo->beta.Output_Ix = 0.0f;

    // 初始化为零，data结构体
    smo->data.Input_We = 0.0f;
}

/**
 * @description: 静止坐标系下的SMO滑模的离散运行函数
 * @reminder: https://github.com/Sguan-ZhouQing/SguanFOC_Library/blob/main/%E9%85%8D%E5%A5%97Simulink%E6%A8%A1%E5%9E%8B%E5%BC%80%E6%BA%90%E2%91%A1%5B%E7%AE%97%E6%B3%95%E5%8E%9F%E7%90%86%E5%9B%BE%5D/Sguan_SMO.png
 * @reminder: (上方链接是此Sguan_SMO模块Simulink原理仿真图)
 * @param {SMO_STRUCT} *smo
 * @return {*}
 */
void SMO_Loop(SMO_STRUCT *smo){
    SMO_RUN(smo,&smo->alpha);
    SMO_RUN(smo,&smo->beta);
    smo->alpha.Input_Iy = smo->beta.Output_Ix;
    smo->beta.Input_Iy = smo->alpha.Output_Ix;
}

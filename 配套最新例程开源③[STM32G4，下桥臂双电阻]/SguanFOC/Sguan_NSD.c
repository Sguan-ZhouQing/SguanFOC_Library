/*
 * @Author: 星必尘Sguan
 * @GitHub: https://github.com/Sguan-ZhouQing
 * @Date: 2026-06-03 01:05:02
 * @LastEditors: 星必尘Sguan|3464647102@qq.com
 * @LastEditTime: 2026-06-05 03:48:14
 * @FilePath: \SguanFOC_Debug\SguanFOC\Sguan_NSD.c
 * @Description: SguanFOC库的“NSD电机极性辨识(需结合高频注入使用)”实现
 * 
 * Copyright (c) 2026 by $星必尘Sguan, All Rights Reserved. 
 */
#include "Sguan_NSD.h"

/* UserData配置文件声明 */
#include "UserData_Function.h"

/**
 * @description: 电机极性辨识NSD函数初始化
 * @reminder: (初始化相关系数float->double->float)
 * @reminder: (单浮点转double运算，提高系数精度)
 * @param {NSD_STRUCT} *nsd
 * @return {*}
 */
void NSD_Init(NSD_STRUCT *nsd){
    double p_temp1 = ((double)nsd->T)*((double)nsd->Wo)*(((double)nsd->zeta)*4.0);
    double p_temp2 = ((double)nsd->T)*((double)nsd->Wo)*
                    ((double)nsd->T)*((double)nsd->Wo);
    double den = p_temp2+p_temp1+4.0;

    nsd->go.p_num = (float)(p_temp1/den);
    nsd->go.p_den[0] = (float)((-8.0+2.0*p_temp2)/den);
    nsd->go.p_den[1] = (float)((p_temp2-p_temp1+4.0)/den);

    // 初始化为零
    nsd->go.p_i[0] = 0.0f;
    nsd->go.p_i[1] = 0.0f;

    nsd->go.Input_D = 0.0f;
    nsd->go.Output_Ud = 0.0f;
    nsd->go.Output_Flag = 0;
    
    nsd->go.Step = 0;
}

/**
 * @description: 电机极性辨识NSD函数初始化
 * @reminder: (纯设定输入信号的正负偏置)
 * @param {NSD_STRUCT} *nsd
 * @return {*}
 */
void NSD_SetUd_Init(NSD_STRUCT *nsd){
    // 1.百分之70的时间用于等待初始状态稳定
    // (这个时候HFI高频注入开始收敛)
    User_Delay((uint32_t)(nsd->Cycle*700.0f));

    // 2.开始施加正向D轴电压偏置
    nsd->go.Output_Ud = nsd->Ud;
    nsd->go.Step = 1;
    User_Delay((uint32_t)(nsd->Cycle*100.0f));

    // 3.偏置清零
    nsd->go.Output_Ud = 0.0f;
    nsd->go.Step = 2;
    User_Delay((uint32_t)(nsd->Cycle*100.0f));

    // 4.开始施加负向D轴电压偏置
    nsd->go.Output_Ud = -nsd->Ud;
    nsd->go.Step = 3;
    User_Delay((uint32_t)(nsd->Cycle*100.0f));

    // 5.NSD电机极性辨识结束
    nsd->go.Output_Ud = 0.0f;
    nsd->go.Step = 4;
    User_Delay((uint32_t)(nsd->Cycle*100.0f));
}

/**
 * @description: NSD电机极性辨识的离散运行函数
 * @reminder: https://github.com/Sguan-ZhouQing/SguanFOC_Library/blob/main/%E9%85%8D%E5%A5%97Simulink%E6%A8%A1%E5%9E%8B%E5%BC%80%E6%BA%90%E2%91%A1%5B%E7%AE%97%E6%B3%95%E5%8E%9F%E7%90%86%E5%9B%BE%5D/Sguan_NSD.png
 * @reminder: (上方链接是此Sguan_NSD模块Simulink原理仿真图)
 * @param {NSD_STRUCT} *nsd
 * @return {*}
 */
uint8_t NSD_ReadOffset_Loop(NSD_STRUCT *nsd){
    // 1.如果电机极性辨识结束则直接不参与下面计算
    if (nsd->go.Step >= 4){
        return 1;
    }

    // 2.带通滤波器->带入差分方程，计算输出
    float High = nsd->go.p_num*(nsd->go.Input_D - nsd->go.p_i[1]) - 
                nsd->go.p_den[0]*nsd->go.p_o[0] - 
                nsd->go.p_den[1]*nsd->go.p_o[1];

    // 3.带通滤波器->更新历史输入和输出数值
    nsd->go.p_i[1] = nsd->go.p_i[0];
    nsd->go.p_i[0] = nsd->go.Input_D;
    nsd->go.p_o[1] = nsd->go.p_o[0];
    nsd->go.p_o[0] = High;

    // 4.高频信号数值积分值并判断输出
    switch (nsd->go.Step){
    case 1:
        nsd->go.Integration0 += nsd->T*High;
        break;
    case 2:
        if (nsd->go.Integration0 < 0){
            nsd->go.Integration0 = -nsd->go.Integration0;
        }
        break;
    case 3:
        nsd->go.Integration1 += nsd->T*High;
        break;
    case 4:
        if (nsd->go.Integration1 < 0){
            nsd->go.Integration1 = -nsd->go.Integration1;
        }
        if ((nsd->go.Integration0 - nsd->go.Integration1) < 0.0f){
            nsd->go.Output_Flag = 1;
        }
        return 1;
    default:
        break;
    }

    // 5.返回极性辨识还在进行中的标志位
    return 0;
}

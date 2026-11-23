/* ===================================================================
 * 文件:     Sguan_DOB.c
 * 项目:     SguanFOC_v3.1.0 —— 开源 FOC 电机控制框架（无感FOC · 浮点运算 · 进阶）
 * 链接:     https://github.com/Sguan-ZhouQing/SguanFOC_Library
 * 版权:     Copyright 2026 星必尘Sguan <3464647102@qq.com>
 * 许可证:   MIT License
 * SPDX:     SPDX-License-Identifier: MIT
 *
 * 许可说明:
 *   [EN] Permission is hereby granted, free of charge, to use, copy,
 *        modify, merge, publish, distribute, sublicense, and/or sell
 *        copies of this software, provided this notice is retained.
 *   [中文] 本软件依据 MIT 许可证授权：可免费使用、复制、修改、合并、
 *        出版、分发、再授权及销售，仅须保留本版权与许可声明。
 *   [DE] Hiermit wird jeder Person kostenlos erlaubt, diese Software zu
 *        nutzen, zu kopieren, zu veraendern, zu veroeffentlichen, zu
 *        vertreiben, zu unterlizenzieren und/oder zu verkaufen, sofern
 *        dieser Hinweis erhalten bleibt.
 *   [JA] 本ソフトウェアは MIT ライセンスに基づき、無償で使用・複製・
 *        改変・公開・配布・再許諾・販売が許可されます。ただし本表示を
 *        保持する必要があります。
 * 完整许可文本: 参见仓库根目录下的 LICENSE-MIT 文件。
 * =================================================================== */
#include "Sguan_DOB.h"

/**
 * @description: 超螺旋滑模DOB的初始化函数
 * @reminder: (初始化相关系数float->double->float)
 * @reminder: (单浮点转double运算，提高系数精度)
 * @param {DOB_STRUCT} *dob
 * @return {*}
 */
void DOB_Init(DOB_STRUCT *dob){
    double den = 2.0*((double)dob->J) + ((double)dob->B)*((double)dob->T);

    dob->smdo.I_num = (float)(((double)dob->T)/2.0);
    dob->smdo.O_num = (float)((((double)dob->J)*((double)dob->T))/den);
    dob->smdo.O_den = (float)((2.0*((double)dob->J) - 
                            ((double)dob->B)*((double)dob->T))/den);
    dob->smdo.Gain0 = (float)((1.5*((double)dob->Pn)*
                            ((double)dob->Flux))/
                            ((double)dob->J));
    dob->smdo.Gain1 = (float)(1.0/((double)dob->J));

    // 初始化为零
    dob->smdo.Output_Wm = 0.0f;
    dob->smdo.Input_Iq = 0.0f;
    dob->smdo.Input_Wm = 0.0f;
    dob->smdo.Output_Fd = 0.0f;
    dob->smdo.Output_Wm = 0.0f;
}

/**
 * @description: 超螺旋滑模DOB的离散运行函数
 * @reminder: https://github.com/Sguan-ZhouQing/SguanFOC_Library/blob/main/%E9%85%8D%E5%A5%97Simulink%E6%A8%A1%E5%9E%8B%E5%BC%80%E6%BA%90%E2%91%A1%5B%E7%AE%97%E6%B3%95%E5%8E%9F%E7%90%86%E5%9B%BE%5D/Sguan_DOB.png
 * @reminder: (上方链接是此Sguan_DOB模块Simulink原理仿真图)
 * @param {DOB_STRUCT} *dob
 * @return {*}
 */
void DOB_Loop(DOB_STRUCT *dob){
    // 1.创建局部变量，并运算(part0)
    float part_main,part0,part1,part2,error_wm,sign,temp_fd;
    part0 = dob->smdo.Input_Iq*dob->smdo.Gain0;

    // 2.计算扰动力矩(part1)
    error_wm = dob->smdo.Output_Wm - dob->smdo.Input_Wm;
    sign = Value_Sign(error_wm);
    temp_fd = sign*dob->K2;
    dob->smdo.Output_Fd += dob->smdo.I_num*(temp_fd + dob->smdo.Fd_i);
    Value_Limit(&dob->smdo.Output_Fd,
                dob->OutMax_Fd,
                dob->OutMin_Fd);
    part1 = dob->smdo.Output_Fd*dob->smdo.Gain1;
    
    // 3.计算不连续量(part3)
    part2 = Value_sqrtf(Value_fabsf(error_wm))*sign*dob->K1;

    // 4.总积分项运算
    part_main = part0 - part1 - part2;
    dob->smdo.Output_Wm = dob->smdo.O_num*(part_main + dob->smdo.Wm_i) + 
                        dob->smdo.O_den*dob->smdo.Output_Wm;
    Value_Limit(&dob->smdo.Output_Wm,
                dob->OutMax_Wm,
                dob->OutMin_Wm);

    // 5.更新历史输入输出值
    dob->smdo.Fd_i = temp_fd;
    dob->smdo.Wm_i = part_main;
}


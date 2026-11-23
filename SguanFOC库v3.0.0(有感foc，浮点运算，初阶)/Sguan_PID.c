/* ===================================================================
 * 文件:     Sguan_PID.c
 * 项目:     SguanFOC_v3.0.0 —— 开源 FOC 电机控制框架（有感FOC · 浮点运算 · 初阶）
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
#include "Sguan_PID.h"

// 闭环系统PID核心参数初始化，主函数调用
void PID_Init(PID_STRUCT *pid){
    double temp0 = pid->T*pid->Ki;
    double temp1 = pid->T*pid->Wc;
    double temp2 = pid->Kd*pid->Wc;
    pid->run.I_num[0] = (float)temp0;
    pid->run.I_num[1] = (float)temp0;
    pid->run.D_num[0] = (float)(2*temp2);
    pid->run.D_num[1] = (float)(-2*temp2);
    pid->run.D_den[0] = (float)(2+temp1);
    pid->run.D_den[1] = (float)(-2+temp1);
    // 初始化为零
    for (int n = 0; n < 2; n++){
        pid->run.i[n] = 0;
        pid->run.Io[n] = 0;
        pid->run.Do[n] = 0;
    }
    pid->run.Ref = 0;
    pid->run.Fbk = 0;
    pid->run.Output = 0;
}

// 闭环控制运算的定时器中断服务函数
void PID_Loop(PID_STRUCT *pid){
    // 积分项冻结限幅的标志位
    static uint8_t IntegralFrozen_flag = 0;
    // 更新历史输入和输出数值
    pid->run.i[1] = pid->run.i[0];
    pid->run.Io[1] = pid->run.Io[0];
    pid->run.Do[1] = pid->run.Do[0];
    // 更新当前输入
    pid->run.i[0] = pid->run.Ref - pid->run.Fbk;
    if (pid->Ki){
        // 判断是否需要冻结积分
        if (IntegralFrozen_flag) {
            // 如果积分已冻结，保持上次的积分值
            pid->run.Io[0] = pid->run.Io[1];
            
            // 检查是否可以解除冻结
            // 情况1：误差反向（误差符号与积分输出符号相反）
            // 情况2：积分值回到限幅范围内
            if ((pid->run.i[0] * pid->run.Io[0] < 0) ||  // 误差反向
                (pid->run.Io[0] < pid->IntMax && pid->run.Io[0] > pid->IntMin)) {  // 回到范围内
                IntegralFrozen_flag = 0;
            }
        } else {
            // 正常计算积分
            pid->run.Io[0] = (pid->run.I_num[0]*pid->run.i[0] + pid->run.I_num[1]*pid->run.i[1] 
                        + 2.0f*pid->run.Io[1]) / 2.0f;
            
            // 检查是否达到限幅，达到则冻结积分
            if (pid->run.Io[0] > pid->IntMax) {
                pid->run.Io[0] = pid->IntMax;
                IntegralFrozen_flag = 1;
            }
            else if (pid->run.Io[0] < pid->IntMin) {
                pid->run.Io[0] = pid->IntMin;
                IntegralFrozen_flag = 1;
            }
        }
    }
    if (pid->Kd){
        pid->run.Do[0] = (pid->run.D_num[0]*pid->run.i[0] + pid->run.D_num[1]*pid->run.i[1] 
                    - pid->run.D_den[1]*pid->run.Do[1]) / pid->run.D_den[0];
    }
    pid->run.Output = pid->run.i[0]*pid->Kp + pid->run.Io[0] + pid->run.Do[0];
    pid->run.Output = Value_Limit(pid->run.Output,pid->OutMax,pid->OutMin);
}


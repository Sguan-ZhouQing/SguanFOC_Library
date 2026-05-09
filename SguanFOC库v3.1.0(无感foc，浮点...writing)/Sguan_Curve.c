/*
 * @Author: 星必尘Sguan
 * @GitHub: https://github.com/Sguan-ZhouQing
 * @Date: 2026-04-29 16:26:00
 * @LastEditors: 星必尘Sguan|3464647102@qq.com
 * @LastEditTime: 2026-04-29 16:34:59
 * @FilePath: \SguanFOC_Debug\SguanFOC\Sguan_Curve.c
 * @Description: SguanFOC库的“半周期sin(先增后减的加减速算法)”实现
 * 
 * Copyright (c) 2026 by $星必尘Sguan, All Rights Reserved. 
 */
#include "Sguan_Curve.h"

void Curve_Loop(CURVE_STRUCT *curve){
    // 检测输入是否发生变化
    if (Value_fabsf(curve->go.Input - curve->go.TargetSpeed) > 0.001f){
        // 输入变化，重新设置曲线参数
        curve->go.StartSpeed = curve->go.Output;  // 从当前输出开始
        curve->go.TargetSpeed = curve->go.Input;  // 新目标为当前输入
        curve->go.StepCount = 0;
        
        // 计算速度变化范围，如果速度变化很小，直接设置为目标值
        curve->go.SpeedRange = curve->go.TargetSpeed - curve->go.StartSpeed;
        if (Value_fabsf(curve->go.SpeedRange) < 0.001f){
            curve->go.Output = curve->go.TargetSpeed;
            curve->go.Active = 0;
            return;
        }
        
        // 计算中间转折速度和总步数
        curve->go.MidSpeed = curve->go.StartSpeed + curve->go.SpeedRange / 2.0f;
        curve->go.TotalSteps = (uint32_t)(curve->K_factor*Value_fabsf(curve->go.SpeedRange));
        if (curve->go.TotalSteps < 2){
            curve->go.TotalSteps = 2; // 至少2步
        }
        curve->go.Active = 1;
    }
    
    // 如果不活跃，直接返回当前输出
    if (!curve->go.Active){
        return;
    }
    
    // 如果已经完成所有步数
    if (curve->go.StepCount >= curve->go.TotalSteps){
        curve->go.Output = curve->go.TargetSpeed;
        curve->go.Active = 0;
        return;
    }
    
    // 计算当前进度
    float progress = (float)curve->go.StepCount/(float)(curve->go.TotalSteps - 1);
    float sin_value;
    
    if (curve->go.SpeedRange > 0){
        // 加速：使用 -π/2 到 π/2 的正弦部分
        float angle = -Value_2PI + progress*Value_PI;
        sin_value = fast_sin(angle);
        curve->go.Output = curve->go.MidSpeed + (curve->go.SpeedRange/2.0f)*sin_value;
    } else{
        // 减速：使用 π/2 到 3π/2 的正弦部分
        float angle = Value_2PI + progress*Value_PI;
        sin_value = fast_sin(angle);
        curve->go.Output = curve->go.MidSpeed + (curve->go.SpeedRange/2.0f)*sin_value;
    }
    curve->go.StepCount++;
}


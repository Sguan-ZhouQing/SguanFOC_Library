/*
 * @Author: 星必尘Sguan
 * @GitHub: https://github.com/Sguan-ZhouQing
 * @Date: 2026-04-29 16:26:00
 * @LastEditors: 星必尘Sguan|3464647102@qq.com
 * @LastEditTime: 2026-04-29 16:34:59
 * @FilePath: \SguanFOC_Debug\SguanFOC\Sguan_Curve.c
 * @Description: SguanFOC库的“双曲线(先增后减的加减速算法)”实现
 * 
 * Copyright (c) 2026 by $星必尘Sguan, All Rights Reserved. 
 */
#include "Sguan_Curve.h"

static float Curve_Gain_Tick(CURVE_STRUCT *curve){
    if (curve->go.Output <= (curve->go.Input/2.0f)){
        curve->go.Gain += curve->go.K*curve->T;
    }
    else{
        curve->go.Gain -= curve->go.K*curve->T;   
    }
    return curve->go.Gain;
}


void Curve_Init(CURVE_STRUCT *curve){
    curve->go.Gain = 0.0f;
    curve->go.K = curve->K_max*2.0f/curve->Cycle;
}

/**
 * @description: 斜切双曲线生成函数
 * @param {CURVE_STRUCT} *curve
 * @return {*}
 */
void Curve_Loop(CURVE_STRUCT *curve){
    if (curve->go.Input == curve->go.Last_in){
        curve->go.Output = curve->go.Input;
        return;
    }
    

    if ((curve->go.Input >= curve->go.Last_in) && (curve->go.Output < curve->go.Input)){        
        if (curve->go.Output <= (curve->go.Input/2.0f)){
            curve->go.Gain += curve->go.K*curve->T;
        }
        else{
            curve->go.Gain -= curve->go.K*curve->T;
            
        }
        curve->go.Output += curve->go.Gain*curve->T;
    }
    else if ((curve->go.Input <= curve->go.Last_in) && (curve->go.Output > curve->go.Input)){
        if (curve->go.Output <= (curve->go.Input/2.0f)){
            curve->go.Gain += curve->go.K*curve->T;
        }
        else{
            curve->go.Gain -= curve->go.K*curve->T;
            
        }
        curve->go.Output -= curve->go.Gain*curve->T;
    }

    // curve->go.Output += Gain*curve->T;
    
}

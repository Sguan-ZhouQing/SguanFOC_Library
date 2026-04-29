/*
 * @Author: 星必尘Sguan
 * @GitHub: https://github.com/Sguan-ZhouQing
 * @Date: 2026-04-29 16:26:00
 * @LastEditors: 星必尘Sguan|3464647102@qq.com
 * @LastEditTime: 2026-04-29 16:34:59
 * @FilePath: \SguanFOC_Debug\SguanFOC\Sguan_Curve.c
 * @Description: 
 * 
 * Copyright (c) 2026 by $星必尘Sguan, All Rights Reserved. 
 */
#include "Sguan_Curve.h"

static float Curve_Gain_Tick(CURVE_STRUCT *curve){
    if (curve->set.Output <= (curve->set.Input/2.0f)){
        curve->set.Gain += curve->set.Multiple;
    }
    else{
        curve->set.Gain -= curve->set.Multiple;   
    }
    return curve->set.Gain*curve->T;
}


void Curve_Init(CURVE_STRUCT *curve){
    curve->set.Gain = curve->K_min;
    curve->set.Multiple = (curve->K_max - curve->K_min)*2.0f*curve->T;
}

/**
 * @description: 斜切双曲线生成函数
 * @param {CURVE_STRUCT} *curve
 * @return {*}
 */
void Curve_Loop(CURVE_STRUCT *curve){
    if ((curve->set.Input >= curve->set.Last_in) && (curve->set.Output < curve->set.Input)){        
        if (curve->set.Output <= (curve->set.Input/2.0f)){
            curve->set.Gain += curve->set.Multiple*curve->T;
        }
        else{
            curve->set.Gain -= curve->set.Multiple*curve->T;
            
        }
        curve->set.Output += curve->set.Gain*curve->T;
    }
    else if ((curve->set.Input <= curve->set.Last_in) && (curve->set.Output > curve->set.Input)){
        if (curve->set.Output <= (curve->set.Input/2.0f)){
            curve->set.Gain += curve->set.Multiple*curve->T;
        }
        else{
            curve->set.Gain -= curve->set.Multiple*curve->T;
            
        }
        curve->set.Output -= curve->set.Gain*curve->T;
    }

    // curve->set.Output += Gain*curve->T;
    
}

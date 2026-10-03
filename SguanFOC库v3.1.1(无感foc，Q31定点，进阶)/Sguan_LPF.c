#include "Sguan_LPF.h"


// LPF初始化函数
void LPF_Init(LPF_STRUCT *lpf){
    // 1.计算其余传递函数系数
    float num = (float)(((double)lpf->T)*((double)lpf->Wc)/(1.0 + ((double)lpf->T)*((double)lpf->Wc)));
    float den = (float)(1.0/(1.0 + ((double)lpf->T)*((double)lpf->Wc)));

    lpf->go.num = iqmath_from_float(num, 1.0f);
    lpf->go.den = iqmath_from_float(den, 1.0f);

    // 3.初始化为零
    lpf->go.Input = 0;
    lpf->go.Output = 0;
}


// LPF运行函数
void LPF_Loop(LPF_STRUCT *lpf){
    // 1.带入差分方程，计算输出
    // (Output和Input是同一单位，所以num和den是单位为1的比例系数)
    lpf->go.Output = iqmath_add(
        iqmath_mul(lpf->go.Input, lpf->go.num),
        iqmath_mul(lpf->go.Output, lpf->go.den));
}


#include "Sguan_FIR.h"

void FIR_Loop(FIR_STRUCT *fir){
    // 1.如果滑动通道设计不对，跳出运算
    if (FIR_N == 2){
        return;
    }

    // 2.输入数据并计算，输出数据
    fir->data[fir->count] = fir->Input;
    float temp = 0.0f;
    for (int i = 0; i < FIR_N; i++){
        temp +=fir->data[i];
    }
    fir->Output = temp/(float)FIR_N;

    // 3.滑动数据count更新
    fir->count++;
    if (fir->count > FIR_N){
        fir->count = 0;
    }
}


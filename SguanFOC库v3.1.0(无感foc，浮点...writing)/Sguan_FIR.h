#ifndef __SGUAN_FIR
#define __SGUAN_FIR

#define FIR_WC 100          // (rad/s)滑动滤波器的截止频率
#define FIR_T  1e-5         // (s)离散运行的时间周期
#define FIR_N ((int)(0.443f/((float)FIR_WC*(float)FIR_T)) >= 2) ? ((int)(0.443f/((float)FIR_WC*(float)FIR_T)) >= 2) : 2 // (int)滑动滤波的滑动数量

typedef struct{
    float data[FIR_N];      // (数据)滑动滤波数据
    int count;              // (中间量)当前数值记录

    float Input;            // (输入数据)滤波输入
    float Output;           // (输出数据)滤波输出
}FIR_STRUCT;

void FIR_Loop(FIR_STRUCT *fir);


#endif // SGUAN_FIR

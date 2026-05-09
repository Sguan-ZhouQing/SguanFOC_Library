#ifndef __SGUAN_CURVE_H
#define __SGUAN_CURVE_H

/* SguanFOC配置文件声明 */
#include "Sguan_Config.h"

typedef struct {
    float StartSpeed;      // (中间量)起始速度数值
    float TargetSpeed;     // (中间量)目标速度数值
    float MidSpeed;        // (中间量)中间转折速度
    float SpeedRange;      // (中间量)速度变化范围
    uint32_t TotalSteps;   // (中间量)总步数
    uint32_t StepCount;    // (中间量)当前步数计数
    uint8_t Active;        // (中间量)是否激活曲线

    float Input;           // (输入数据)输入目标速度
    float Output;          // (输出数据)输出平滑速度
} CURVE_GO_STRUCT;

typedef struct {
    CURVE_GO_STRUCT go;    // (结构体)曲线加减速
    
    float K_factor;        // (参数设计)加减速时间系数
} CURVE_STRUCT;

void Curve_Loop(CURVE_STRUCT *curve);


#endif // SGUAN_CURVE_H

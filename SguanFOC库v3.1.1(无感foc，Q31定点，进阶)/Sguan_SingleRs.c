#include "Sguan_SingleRs.h"

/* SguanFOC配置文件声明 */
#include "SguanFOC.h"
#include "Sguan_ADC.h"

#define MOTOR_mid 0

// ................................................................
static uint8_t SingleRs_Duty_Loop(Q31_t Du, Q31_t Dv, Q31_t Dw, 
                   Q31_t *D_min, Q31_t *D_mid, Q31_t *D_max,
                   Q31_t *D_n, Q31_t *D_m);
// ................................................................
// Q31_t MOS_Duty = Dead_T_q31*2.0f/PMSM_RUN_T_q31
// Q31_t ADC_Duty = ADC_T_q31*2.0f/PMSM_RUN_T_q31
// Q31_t Min_Duty = (MOS_Duty + ADC_Duty)

// (0x0226809D · 实际表示 ≈ 0.0167999998666 · 归一化 0.0167999998666)
Q31_t MOS_Duty = 36077725;
// (0x00418937 · 实际表示 ≈ 0.00199999986216 · 归一化 0.00199999986216)
Q31_t ADC_Duty = 4294967;
// (0x026809D5 · 实际表示 ≈ 0.0188000001945 · 归一化 0.0188000001945)
Q31_t Min_Duty = 40372693;

int16_t MOTOR_Ix[3];
Q31_t Duty_min = 0;
Q31_t Duty_mid = 0;
Q31_t Duty_max = 0;
Q31_t Duty_n = 0;
Q31_t Duty_m = 0;
uint8_t ADC_Yes = 0;

// 对三相占空比计算
static uint8_t SingleRs_Duty_Loop(Q31_t Du, Q31_t Dv, Q31_t Dw, 
                   Q31_t *D_min, Q31_t *D_mid, Q31_t *D_max,
                   Q31_t *D_n, Q31_t *D_m){
    Q31_t a = Du, b = Dv, c = Dw;   // 高侧的PWM占空比
    Q31_t tmp;
    
    // 排序网络：3元素最优需要3次比较交换
    if (a > b) { tmp = a; a = b; b = tmp; }  // a <= b
    if (a > c) { tmp = a; a = c; c = tmp; }  // a 是最小的
    if (b > c) { tmp = b; b = c; c = tmp; }  // b <= c
    
    // 输出排序后的三相占空比
    *D_min = a;
    *D_mid = b;
    *D_max = c;
    
    // 输出差值
    *D_n = b - a;  // D_mid - D_min
    *D_m = c - b;  // D_max - D_mid

    if ((Duty_m <= Min_Duty) || (Duty_n <= Min_Duty)){
        return 0;
    }
    return 1;
}

// 对读取到的电流进行重构
void SingleRs_ReadCurrent(uint16_t Raw_CH0, uint16_t Raw_CH1){
    switch (sector_SVPWM){
    case 1:
        MOTOR_Ix[2] = -((int16_t)Raw_CH0-(int16_t)MOTOR_mid); // V110 -IC
        MOTOR_Ix[0] = ((int16_t)Raw_CH1-(int16_t)MOTOR_mid);  // V100  IA
        MOTOR_Ix[1] = -MOTOR_Ix[2]-MOTOR_Ix[0];
        break;
    case 2:
        MOTOR_Ix[2] = -((int16_t)Raw_CH0-(int16_t)MOTOR_mid); // V110 -IC
        MOTOR_Ix[1] = ((int16_t)Raw_CH1-(int16_t)MOTOR_mid);  // V010  IB
        MOTOR_Ix[0] = -MOTOR_Ix[2]-MOTOR_Ix[1];
        break;
    case 3:
        MOTOR_Ix[0] = -((int16_t)Raw_CH0-(int16_t)MOTOR_mid); // V011 -IA
        MOTOR_Ix[1] = ((int16_t)Raw_CH1-(int16_t)MOTOR_mid);  // V010  IB
        MOTOR_Ix[2] = -MOTOR_Ix[0]-MOTOR_Ix[1];
        break;
    case 4:
        MOTOR_Ix[0] = -((int16_t)Raw_CH0-(int16_t)MOTOR_mid); // V011 -IA
        MOTOR_Ix[2] = ((int16_t)Raw_CH1-(int16_t)MOTOR_mid);  // V001  IC
        MOTOR_Ix[1] = -MOTOR_Ix[0]-MOTOR_Ix[2];
        break;
    case 5:
        MOTOR_Ix[1] = -((int16_t)Raw_CH0-(int16_t)MOTOR_mid); // V101 -IB
        MOTOR_Ix[2] = ((int16_t)Raw_CH1-(int16_t)MOTOR_mid);  // V001  IC
        MOTOR_Ix[0] = -MOTOR_Ix[1]-MOTOR_Ix[2];
        break;
    case 6:
        MOTOR_Ix[1] = -((int16_t)Raw_CH0-(int16_t)MOTOR_mid); // V101 -IB
        MOTOR_Ix[0] = ((int16_t)Raw_CH1-(int16_t)MOTOR_mid);  // V100  IA
        MOTOR_Ix[2] = -MOTOR_Ix[1]-MOTOR_Ix[0];
        break;
    
    default:
        MOTOR_Ix[0] = 0;// 原始数据A相电流
        MOTOR_Ix[1] = 0;// 原始数据B相电流
        MOTOR_Ix[2] = 0;// 原始数据C相电流
        break;
    }
}


// (SingleRs)设置下一时刻的采样时机
void SingleRs_END_Loop(void){
    // Sguan.foc.Du
    // Sguan.foc.Dv
    // Sguan.foc.Dw

    ADC_Yes = SingleRs_Duty_Loop(Sguan.Foc.Du, 
                                Sguan.Foc.Dv, 
                                Sguan.Foc.Dw, 
                                &Duty_min, 
                                &Duty_mid, 
                                &Duty_max, 
                                &Duty_n, 
                                &Duty_m);

    // 判断矢量是否到达观测区
    // (0.2->429496730)
    // (0.8->1717986918)
    if (ADC_Yes){
        Sguan_ADC_SetDuty((uint16_t)(iqmath_mul((Duty_max - MOS_Duty - iqmath_mul((Duty_m-MOS_Duty),429496730)),Sguan.Motor.Duty)),
            (uint16_t)(iqmath_mul((Duty_mid - MOS_Duty - iqmath_mul((Duty_n-MOS_Duty),429496730)),Sguan.Motor.Duty)));
    }
    else{
        uint16_t high,low;
        if (Duty_m <= Min_Duty){
            high = (uint16_t)(iqmath_mul((Duty_mid + iqmath_mul((Min_Duty-MOS_Duty),1717986918)),Sguan.Motor.Duty));
        }
        else{
            high = (uint16_t)(iqmath_mul((Duty_max - MOS_Duty - iqmath_mul((Duty_m-MOS_Duty),429496730)),Sguan.Motor.Duty));
        }
        if (Duty_n <= Min_Duty){
            low = (uint16_t)(iqmath_mul((Duty_mid - MOS_Duty - iqmath_mul((Min_Duty-MOS_Duty),429496730)),Sguan.Motor.Duty));
        }
        else{
            low = (uint16_t)(iqmath_mul((Duty_mid - MOS_Duty - iqmath_mul((Duty_n-MOS_Duty),429496730)),Sguan.Motor.Duty));
        }
        Sguan_ADC_SetDuty(high, low);
    }
}


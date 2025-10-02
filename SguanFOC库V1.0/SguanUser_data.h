#ifndef __SGUANUSER_DATA_H
#define __SGUANUSER_DATA_H

// 【1】电机参数设定（宏定义）
#define SguanFOC_ARR 2000               // 磁定向控制之中PWM份额值
#define Pole_Pairs 7                    // 电机的极对极数（通常为7）
#define Motor_Dir 1                     // 电机方向辨识（正负区分）
#define Dead_Time 0.01f                 // 死区时间限幅（Low-High）
#define Motor_Vbus 12.0f                // 设置电机驱动电源的大小
#define MOTOR_RESISTANCE 11.1f          // 电机相电阻（Ω）
// 【2】角度位置传感器延迟误差补偿
float OPTIMAL_DELAY_TIME = 0.017f;
// 【3】Q轴电压计算值（中间变量）
#define Intermediate_Raw 1972           // 电流采样基准Raw数据
#define Operational_Num 50              // User自己的功放倍数
#define Shunt_Resistor 20               // 电流采样电阻（单位毫欧）
float current_Iq = 0.0f;                // Iq电流滤波后的数据
// 【4】卡尔曼滤波宏定义(Iq电流滤波)
#define M_NOISE     10.0f               // R值,传感器噪声大则设大
#define P_NOISE     0.01f               // Q值,系统变化快则设大

// 【5】用户接口函数书写（必写项）
void Sguan_TimerDriverInit(void){
    /* Your code here */
}
void Sguan_PosSensorInit(void){
    /* Your code here */
}
void Sguan_CurSamplingInit(void){
    /* Your code here */
}
void Sguan_UartInit(void){
    /* Your code here */
}
void Sguan_PwmSet(uint8_t Channel,uint16_t duty){
    switch (Channel){
    case 1:
        /* Your code for PWM Channel1 */
        break;
    case 2:
        /* Your code for PWM Channel2 */
        break;
    case 3:
        /* Your code for PWM Channel3 */
        break;
    default:
        break;
    }
}
void Sguan_UserDelay(uint32_t ms){
    /* Your code here */
}
float Sguan_CurAcquisition(uint8_t Channel){
    if (Channel){
        /* code for Channel1 Current */
    }
    else {
        /* code for Channel2 Current */
    }// Return your data
}
uint32_t Sguan_UserGetTick(void){
    /* Your code here */
}
void Sguan_ReadMultiTurnAngle(float *rads){
    /* Your code here */
}
bool Sguan_ReadAngle(float *rad){
    /* Your code here */
}
void Sguan_FilteredAngularVelocity(float *Speed){
    /* Your code here */
}

/* 【6】函数使用调用
extern void FOC_Init(void);
extern void FOC_LoopHandler(void);
extern void FOC_SetPIDParams(const char *loop, float kp, float ki, float kd, float limit);
*/

#endif // SGUANUSER_DATA_H

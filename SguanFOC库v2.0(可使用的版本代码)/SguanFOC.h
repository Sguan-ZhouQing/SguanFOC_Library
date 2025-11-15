#ifndef __SGUANFOC_H
#define __SGUANFOC_H

#include "Sguan_math.h"

typedef enum{
    Current_SINGLE_MODE = 0,// 电流单闭环(力矩控制)
    Velocity_SINGLE_MODE,   // 速度单闭环(速度控制)
    Position_SINGLE_MODE,   // 位置单闭环(角度控制)
    VelCur_DOUBLE_MODE,     // 速度-电流串级PID
    PosVel_DOUBLE_MODE,     // 位置-速度串级PID
    PosVelCur_THREE_MODE,   // 位置-速度-电流多环PID
}MOTOR_MODE_ENUM;

typedef struct{
    uint8_t Dir_n;          // 与编码器Dir匹配，电机输入角度增加，编码器数值增加
    float Polepairs;        // 电机极对极数
    uint16_t du;            // U相Period数值
    uint16_t dv;            // V相Period数值
    uint16_t dw;            // W相Period数值
}MOTOR_STRUCT;

typedef struct{
    uint8_t Dir_m;          // PWM输出的正反匹配(占空比为mode1或者mode2)
    uint16_t Period;        // 满占空比Period数值
    float Id_Out;           // 电机输入Id
    float Iq_Out;           // 电机输入Iq
    float Target_Id;        // 期望的D轴电流(Target)
    float Target_Iq;        // 期望的Q轴电流(Target)
    float Target_Speed;     // 期望的Q轴电流(Target)
    float Target_Pos;       // 期望的Q轴电流(Target)
    float Input_du;         // 输入的du大小
    float Input_dv;         // 输入的dv大小
    float Input_dw;         // 输入的dw大小
    uint8_t Response_Count; // 记录内外环响应次数
    uint8_t Response_Num;   // PID内外环响应倍数
}FOC_STRUCT;

typedef struct{
	float Kp;	   		    // (参数)比例环路增益
	float Ki;	   		    // (参数)积分增益	
	float Kd;           	// (参数)微分系数
	float Ref;	  		  	// (输入)参考设定点
	float Fbk;	 		    // (输入)反馈值
	float Out;	   	    	// (输出)控制器输出
	float Err;  		    // (数据)误差
	float ErrLast;      	// (数据)上次误差
  	float Ui;	          	// (数据)积分项
	float OutMax; 		  	// (参数)上限饱和限制
	float OutMin;		    // (参数)下限饱和限制
	float ErrLimltFlag;		// (参数)误差限幅标志
	float ErrMax;     		// (参数)误差上限
	float ErrMin;     		// (参数)误差下限	
	float D_Filter;		    // (参数)微分项滤波系数
	float LastDerivative;   // (数据)上次微分项值
}PID_STRUCT;

typedef struct{
    uint8_t Dir_a;          // A相方向设定
    uint8_t Dir_b;          // B偏方向设定
    uint8_t Dir_c;          // C相方向设定
    int32_t Offset_a;       // 电流偏置
    int32_t Offset_b;       // 电流偏置
    int32_t Offset_c;       // 电流偏置
    float Real_Iu;          // 实际的U相电流(Real)
    float Real_Iv;          // 实际的V相电流(Real)
    float Real_Iw;          // 实际的W相电流(Real)
    float Real_Ialpha;      // 实际的α轴电流(Real)
    float Real_Ibeta;       // 实际的β轴电流(Real)
    float Real_Id;          // 实际的D轴电流(Real)
    float Real_Iq;          // 实际的Q轴电流(Real)
    float Raw_Id;           // 当前D轴电流
    float Raw_Iq;           // 当前Q轴电流
    float Id_Filter;        // D轴的LPF系数
    float Iq_Filter;        // Q轴的LPF系数
    float VCC_Voltage;      // 微控制器VCC实际电压值
    float Sampling_resistor;// 采样电阻阻值
    uint8_t Amplifier_Gain; // 运放增益(倍数)
    uint32_t ADC_Accuracy;  // ADC采样精度
    float Final_Gain;       // 最终采样增益
}CURRENT_STRUCT;

typedef struct{
    uint8_t Dir_n;          // 与电机实际旋转方向匹配，电机正向旋转，编码器数值增加
    int32_t PosFlag;        // 记录多圈圈数
    float Offset;           // 编码器偏置
    float Real_Speed;       // 实际的速度(Real)
    float Real_Pos;         // 实际的多圈角度(Real)
    float Real_El_Rad;      // 实际的单圈电角度(Real)
    float Real_Sine;        // 当前的正弦值
    float Real_Cosine;      // 当前的余弦值
    float LastRad;          // 记录的角度
    uint32_t LastTick;      // 记录的上次时间
    float Kalman_R_Filter;  // 噪声越大，参数越大
    float Kalman_Q_Filter;  // 系统反应越快，参数越大
}ENCODER_STRUCT;

typedef struct{
    MOTOR_MODE_ENUM Control_mode; // 控制模式
    MOTOR_STRUCT Motor;     // 电机结构体变量
    FOC_STRUCT Foc;         // FOC结构体变量
    PID_STRUCT Position;    // 位置环PID
    PID_STRUCT Speed;       // 速度环PID
    PID_STRUCT Id;          // 电流环D轴PID
    PID_STRUCT Iq;          // 电流环Q轴PID
    CURRENT_STRUCT Current; // 电流采样结构体变量
    ENCODER_STRUCT Encoder; // 编码器结构体变量
}Motor_System_STRUCT;


// 函数声明
void Sguan_FocInit(void);             // 主函数初始化调用
void SguanFOC_Run_Tick(void);         // 定时1ms中断调用
void SguanFOC_GeneratePWM_Loop(void); // ADC采样完成中断调用(高于or等于1ms中断频率)

#endif // SGUANFOC_H

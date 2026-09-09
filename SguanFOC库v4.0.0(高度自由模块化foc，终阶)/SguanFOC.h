#ifndef __SGUANFOC_H
#define __SGUANFOC_H

#include "Sguan_Function.h"
#include "Sguan_MotorStatus.h"
#include "Sguan_Printf.h"
#include "Sguan_Transfer.h"



typedef struct{
    void (*Func_Start)(void);

}__MOTOR_FUNCTION_STRUCT;

typedef struct{
    #if CONFIG_TRANSFER1 // 典型一阶传递函数
    Transfer1 *transfer1_ch[CONFIG_TRANSFER1];
    #endif // CONFIG_TRANSFER1

    #if CONFIG_TRANSFER2 // 典型二阶传递函数
    Transfer2 *transfer2_ch[CONFIG_TRANSFER2];
    #endif // CONFIG_TRANSFER2

    #if CONFIG_TRANSFER3 // 典型三阶传递函数
    Transfer3 *transfer3_ch[CONFIG_TRANSFER3];
    #endif // CONFIG_TRANSFER3

    #if CONFIG_TRANSFER4 // 典型四阶传递函数
    Transfer4 *transfer4_ch[CONFIG_TRANSFER4];
    #endif // CONFIG_TRANSFER4

    #if CONFIG_TRANSFER5 // 典型五阶传递函数
    Transfer5 *transfer5_ch[CONFIG_TRANSFER5];
    #endif // CONFIG_TRANSFER5

    #if CONFIG_INTEGRATOR // 积分器
    Integrator *integrator_ch[CONFIG_INTEGRATOR];
    #endif // CONFIG_INTEGRATOR

    #if CONFIG_DERIVATIVE // 微分器
    Derivative *derivative_ch[CONFIG_DERIVATIVE];
    #endif // CONFIG_DERIVATIVE

    #if CONFIG_DFT // 快速傅里叶变换
    Dft *dft_ch[CONFIG_DFT];
    #endif // CONFIG_DFT

    #if CONFIG_HALL // 霍尔编码器
    Hall *hall_ch[CONFIG_HALL];
    #endif // CONFIG_HALL

    #if CONFIG_LADRC1 // 一阶线性自适应抗干扰控制
    Ladrc1 *ladrc1_ch[CONFIG_LADRC1];
    #endif // CONFIG_LADRC1

    #if CONFIG_LADRC2 // 二阶线性自适应抗干扰控制
    Ladrc2 *ladrc2_ch[CONFIG_LADRC2];
    #endif // CONFIG_LADRC2

    #if CONFIG_SMC // 传统指数型趋近率的滑模控制
    Smc *smc_ch[CONFIG_SMC];
    #endif // CONFIG_SMC

    #if CONFIG_DPCC // 增量式电流预测控制
    Dpcc *dpcc_ch[CONFIG_DPCC];
    #endif // CONFIG_DPCC

    #if CONFIG_PIR // 比例积分谐振调节器
    Pir *pir_ch[CONFIG_PIR];
    #endif // CONFIG_PIR

    #if CONFIG_PID // 传统闭环控制器
    Pid *pid_ch[CONFIG_PID];
    #endif // CONFIG_PID

    #if CONFIG_PLL // 开环锁相环
    Pll *pll_ch[CONFIG_PLL];
    #endif // CONFIG_PLL

    #if CONFIG_LPF1 // 一阶低通滤波器
    Lpf1 *lpf1_ch[CONFIG_LPF1];
    #endif // CONFIG_LPF1

    #if CONFIG_LPF2 // 二阶低通滤波器
    Lpf2 *lpf2_ch[CONFIG_LPF2];
    #endif // CONFIG_LPF2

    #if CONFIG_HPF1 // 一阶高通滤波器
    Hpf1 *hpf1_ch[CONFIG_HPF1];
    #endif // CONFIG_HPF1

    #if CONFIG_HPF2 // 二阶高通滤波器
    Hpf2 *hpf2_ch[CONFIG_HPF2];
    #endif // CONFIG_HPF2

    #if CONFIG_BPF1 // 带通滤波器(一阶低通和高通串联)
    Bpf1 *bpf1_ch[CONFIG_BPF1];
    #endif // CONFIG_BPF1

    #if CONFIG_BPF2 // 带通滤波器(典型二阶系统改型)
    Bpf2 *bpf2_ch[CONFIG_BPF2];
    #endif // CONFIG_BPF2

    #if CONFIG_NF // 陷波滤波器(典型二阶系统改型)
    Nf *nf_ch[CONFIG_NF];
    #endif // CONFIG_NF

    #if CONFIG_TPNF // 陷波滤波器(三参数陷波滤波器)
    Tpnf *tpnf_ch[CONFIG_TPNF];
    #endif // CONFIG_TPNF

    #if CONFIG_DOB // 超螺旋滑模扰动观测器
    Dob *dob_ch[CONFIG_DOB];
    #endif // CONFIG_DOB

    #if CONFIG_RLS // 电机参数在线辨识观测器
    Rls *rls_ch[CONFIG_RLS];
    #endif // CONFIG_RLS

    #if CONFIG_SMO // (无感)滑模观测器
    Smo *smo_ch[CONFIG_SMO];
    #endif // CONFIG_SMO

    #if CONFIG_NLFO // (无感)非线性磁链观测器
    Nlfo *nlfo_ch[CONFIG_NLFO];
    #endif // CONFIG_NLFO

    #if CONFIG_VCFO // (无感)电压电流互补磁链观测器
    Vcfo *vcfo_ch[CONFIG_VCFO];
    #endif // CONFIG_VCFO

    #if CONFIG_HFI // (无感)高频正弦波注入
    Hfi *hfi_ch[CONFIG_HFI];
    #endif // CONFIG_HFI

    #if CONFIG_ROLO // (无感)降阶龙伯格观测器
    Rolo *rolo_ch[CONFIG_ROLO];
    #endif // CONFIG_ROLO

    #if CONFIG_MARS // (无感)模型参考自适应观测器
    Mars *mars_ch[CONFIG_MARS];
    #endif // CONFIG_MARS

    #if CONFIG_EKF // (无感)扩展卡尔曼滤波
    Ekf *ekf_ch[CONFIG_EKF];
    #endif // CONFIG_EKF

    #if CONFIG_DELAY1 // 延时函数(延时一拍)
    Delay1 *delay1_ch[CONFIG_DELAY1];
    #endif // CONFIG_DELAY1

    #if CONFIG_DELAY2 // 延时函数(延时两拍)
    Delay2 *delay2_ch[CONFIG_DELAY2];
    #endif // CONFIG_DELAY2

    #if CONFIG_DELAY3 // 延时函数(延时三拍)
    Delay3 *delay3_ch[CONFIG_DELAY3];
    #endif // CONFIG_DELAY3

    Sine *sine;                     // 正弦发生器
    Cosine *cosine;                 // 余弦发生器
    SinCos *sincos;                 // 正余弦发生器
    Tan *tan;                       // 正切求解器
    Atan *atan;                     // 反正切求解器
    Limit *limit;                   // 限幅函数
    Sign *sign;                     // 符号函数
    Clarke *clarke;                 // 克拉克变换
    Park *park;                     // 帕克变换
    Ipark *ipark;                   // 帕克逆变换
    Spwm0 *spwm0;                   // 零序注入的SPWM模块
    Spwm *spwm;                     // 普通SPWM模块
    Svpwm *svpwm;                   // 七段式SVPWM模块
    Swpwm *swpwm;                   // 电调PWM无感方波
    SingleRs *singlers;             // 单电阻采样函数
}__MOTOR_TRANSFER_STRUCT;

typedef struct{
    #if CONFIG_Float // 浮点数
    SguanQ             float_ch[CONFIG_Float];
    #endif // CONFIG_Float

    #if CONFIG_Int8 // 8位数据
    int8_t              int8_ch[CONFIG_Int8];
    #endif // CONFIG_Int8

    #if CONFIG_Uint8 // 8位数据
    uint8_t             uint8_ch[CONFIG_Uint8];
    #endif // CONFIG_Uint8

    #if CONFIG_Int16 // 16位数据
    int16_t             int16_ch[CONFIG_Int16];
    #endif // CONFIG_Int16

    #if CONFIG_Uint16 // 16位数据
    uint16_t            uint16_ch[CONFIG_Uint16];
    #endif // CONFIG_Uint16

    #if CONFIG_Int32 // 32位数据
    int32_t             int32_ch[CONFIG_Int32];
    #endif // CONFIG_Int32

    #if CONFIG_Uint32 // 32位数据
    uint32_t            uint32_ch[CONFIG_Uint32];
    #endif // CONFIG_Uint32

    uint8_t response;                           // (环路倍率)内外环控制倍率
}__MOTOR_VALUE_STRUCT;

typedef struct{
    SguanQ target_Speed;                        // (期望速度)Target期望机械角速度
    SguanQ target_Pos;                          // (期望角度)Target期望机械角度
    SguanQ target_Id;                           // (期望电流)期望D轴电流
    SguanQ target_Iq;                           // (期望电流)期望Q轴电流
    SguanQ target_Ud;                           // (期望电压)期望D轴电压
    SguanQ target_Uq;                           // (期望电压)期望Q轴电压

    SguanQ speed_in;                            // (输入量end)速度环输入值
    SguanQ ud_in;                               // (输入量end)D轴电压输入
    SguanQ uq_in;                               // (输入值end)Q轴电压输入

    // ================= 修改线(上面可修改，下面为自动计算量) =================
    SguanQ ualpha;                              // (中间量)alpha轴电压
    SguanQ ubeta;                               // (中间量)beta轴电压

    SguanQ du;                                  // (数据)U相占空比输入0~1
    SguanQ dv;                                  // (数据)V相占空比输入0~1
    SguanQ dw;                                  // (数据)W相占空比输入0~1

    SguanQ sine;                                // (数据)sine临时保存的正弦值
    SguanQ cosine;                              // (数据)cosine临时保存的余弦值

    SguanQ real_vbus;                           // (数据)Real实际的电机母线电压
    SguanQ real_current;                        // (数据)Current实际的电流数值
    SguanQ real_temp;                           // (数据)Temp实际的驱动器物理温度
}__MOTOR_FOC_STRUCT;

typedef struct{
    SguanQ real_id;                             // (Current电流)Real实际D轴电流
    SguanQ real_iq;                             // (Current电流)Real实际Q轴电流

    SguanQ real_ia;                             // (Current相电流)A相电流
    SguanQ real_ib;                             // (Current相电流)B相电流
    SguanQ real_ic;                             // (Current相电流)C相电流

    SguanQ real_ialpha;                         // (Current中间量电流)alpha轴电流
    SguanQ real_ibeta;                          // (Current中间量电流)beta轴电流

    SguanQ final_gain;                          // (ADC增益)最终的ADC电流采样增益
    int32_t current_offset0;                    // (Current电流偏置)offset偏置位
    int32_t current_offset1;                    // (Current电流偏置)offset偏置位
}__MOTOR_CURRENT_STRUCT;

typedef struct{
    SguanQ rs;                                  // (电机实体参数)Rs相电阻参数
    SguanQ ld;                                  // (电机实体参数)Ld电感参数
    SguanQ lq;                                  // (电机实体参数)Lq电感参数
    SguanQ flux;                                // (电机实体参数)Flux磁链参数

    uint8_t poles;                              // (电机实体参数)电机极对数
    SguanQ vbus;                                // (电机实体参数)母线电压

    int8_t motor_dir;                           // (参数设计)电机的运行方向设计
    int8_t encoder_dir;                         // (有感实体参数)编码器方向
    int8_t pwm_dir;                             // (参数设计)PWM占空比高低对应
    uint32_t duty;                              // (参数设计)PWM满占空比

    int8_t current_dir0;                        // (参数设计)电流采样方向0
    int8_t current_dir1;                        // (参数设计)电流采样方向1
    uint8_t current_num;                        // (参数设计)电流通道0->AB相，1->AC相，2->BC相
    uint32_t adc_precision;                     // (参数设计)ADC采样精度,如12位精度为4096
    SguanQ amplifier;                           // (参数设计)运放的放大倍数
    SguanQ mcu_voltage;                         // (参数设计)DSP/单片机的ADC基准电压
    SguanQ sampling_rs;                         // (参数设计)采样电阻的阻值大小
}__MOTOR_MASTER_STRUCT;

typedef struct{
    uint8_t Run;

    MotorStatus motor_status;
    uint8_t Mode;                   
    uint8_t Status;
    uint8_t Flag;

    __MOTOR_FOC_STRUCT Foc;
    
    uint8_t id;
    
    __MOTOR_TRANSFER_STRUCT Transfer;
    __MOTOR_VALUE_STRUCT Value;

    void (*func_high_loop)(void);
    void (*func_low_loop)(void);
    void (*func_printf_loop)(uint8_t *,uint16_t);
    void (*func_main_loop)(void);

}SguanFoc;


#endif // SGUANFOC_H

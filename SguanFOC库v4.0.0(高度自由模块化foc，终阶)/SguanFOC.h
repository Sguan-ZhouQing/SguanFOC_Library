#ifndef __SGUANFOC_H
#define __SGUANFOC_H

/* SguanFOC配置文件声明 */
#include "Sguan_Method.h"
#include "Sguan_MotorStatus.h"
#include "Sguan_Printf.h"
#include "Sguan_Transfer.h"

#define RUN_READY           0x01
#define RUN_INITIALIZING    0x02
#define RUN_STANDBY         0x03
#define RUN_FAULT           0x04
#define RUN_SUCCESS         0x05

#define MOTOR_ONE           0X00
#define MOTOR_TWO           0X01
#define MOTOR_THREE         0X02
#define MOTOR_FOUR          0X03
#define MOTOR_FIVE          0X04
#define MOTOR_SIX           0X05

#define CONTROL_TORUE       0x00
#define CONTROL_VELOCITY    0x01
#define CONTROL_POSITION    0x02

typedef struct{
    SguanQ target_speed;                        // (期望速度)Target期望机械角速度
    SguanQ target_pos;                          // (期望角度)Target期望机械角度
    SguanQ target_id;                           // (期望电流)期望D轴电流
    SguanQ target_iq;                           // (期望电流)期望Q轴电流
    SguanQ target_ud;                           // (期望电压)期望D轴电压
    SguanQ target_uq;                           // (期望电压)期望Q轴电压

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
}Foc;

typedef struct{
    SguanQ real_position;
    SguanQ real_speed;
    SguanQ real_We;
    SguanQ real_Re;

    SguanQ real_offset;
}Encoder;

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
}Current;

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
}Master;

typedef struct{
    uint8_t id_flag;
    uint8_t run_flag;
    uint8_t uart_flag;

    uint32_t tick_run;
    uint32_t tick_last;
    
    Method method;
    MotorStatus motorstatus;
    Printf printf;
    Transfer transfer;

    Foc foc;
    Encoder encoder;
    Current current;
    Master master;

    void (*func_high_loop)(void);
    void (*func_low_loop)(void);
    void (*func_printf_loop)(uint8_t *,uint16_t);
    void (*func_main_loop)(void);
}SguanFoc;

extern SguanFoc sguanfoc[CONFIG_MOTOR];


#endif // SGUANFOC_H

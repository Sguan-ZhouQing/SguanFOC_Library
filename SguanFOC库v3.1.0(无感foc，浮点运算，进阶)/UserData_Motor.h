#ifndef __USERDATA_MOTOR_H
#define __USERDATA_MOTOR_H
#include "SguanFOC.h"
/* 电机控制User用户设置·电机参数(SguanFOC) */

/**
 * @description: 实体参数填写函数的初始化代码
 * @reminder: (此方函数->填写电机实际物理参数)
 * @param {SguanFOC_System_STRUCT} *user
 * @return {*}
 */
static inline void User_Motor_Init(SguanFOC_System_STRUCT *user){
    // +---------------------------------------------------------+
    // |                  电机实体参数Motor填写                   |
    // +---------------------------------------------------------+

    static uint8_t foc_target_initialized = 0;
    if (!foc_target_initialized){
        user->foc.Target_Speed = 0.0f;              // (float)期望的机械角速度
        user->foc.Target_Pos = 0.0f;                // (float)期望的机械角度值
        user->foc.Target_Id = 0.0f;                 // (float)期望的D轴电流
        user->foc.Target_Iq = 0.0f;                 // (float)期望的Q轴电流

        user->foc.Target_VF_Uq = 0.0f;              // (float)设置的Uq大小
        user->foc.Target_IF_Iq = 0.0f;              // (float)设置的Iq大小

        foc_target_initialized = 1;
    }
    /* 说明：Target_* 在首次之后由调用方维护，本函数不再覆盖。
     * 未设值时它们就是上面的 0 —— 这符合"开机不自动转"的安全默认。 */

    user->foc.Ud_in = 0.0f;                         // (float)实际输入的D轴电压
    user->foc.Uq_in = 0.0f;                         // (float)实际输入的Q轴电压

    // 2.Sguan.motor参数设计(电机个性化配置)
    user->motor.identify.Rs = 0.13f;                // (float)相电阻_铭牌 R_t 0.26Ω(相间)÷2
    user->motor.identify.Ld = 0.0000235f;           // (float)D轴电感_铭牌 L 0.047mH(相间)÷2
    user->motor.identify.Lq = 0.0000235f;           // (float)Q轴电感_表贴式电机 Ld=Lq
    
    user->motor.identify.Flux = 0.00536667f;        // (float)电机磁链_由 Kt=16.1mNm/A 反推(Kt=1.5·Pn·Flux, Pn=2)
    user->motor.identify.B = 0.00001f;              // (float)粘性阻尼_铭牌未标注,占位值(仅SMC用)
    user->motor.identify.J = 0.0000016575f;         // (float)转动惯量_16.575g·cm²=1.6575e-6 kg·m²(仅SMC用)
    /* ====================================== 分割线 =================================== */
    user->motor.Poles = 2;                          // (uint8_t)电机的极对数_由实测 1/2 比值反推(见上)
    user->motor.VBUS = 24.0f;                       // (float)标定的母线电压
    
    user->motor.Motor_Dir = 1;                      // (int8_t)电机方向1->正向，负1->负向
    user->motor.Encoder_Dir = 1;                    // (int8_t)编码器方向(IF模式不使用)
    user->motor.PWM_Dir = -1;                       // (int8_t)PWM占空比电平,TIM1为PWM2模式故取-1(契约§9陷阱6)
    user->motor.Duty = 4200;                        // (uint16_t)PWM满占空比数值_TIM1 ARR=4200-1

    user->motor.Current_Dir0 = -1;                  // (int8_t)相线电流方向1->正向，负1->负向_用户实测(D9)
    user->motor.Current_Dir1 = -1;                  // (int8_t)相线电流方向1->正向，负1->负向_用户实测(D9)
    user->motor.Current_Num = 1;                    // (uint8_t)通道0->AB相，1->AC相，2->BC相(D8:CH0=Ia/PC3,CH1=Ic/PC5)
    user->motor.ADC_Precision = 4096;               // (uint32_t)ADC采样精度
    user->motor.Amplifier = 20.0f;                  // (float)DRV8334 CSA 增益 20V/V(D5)
    user->motor.MCU_Voltage = 3.3f;                 // (float)DSP/单片机的ADC电压基准
    user->motor.Sampling_Rs = 0.005f;               // (float)采样电阻的大小_5mΩ
    // (以上四项决定 Final_Gain = 3.3/(4096*20*0.005) = 0.008056640625 A/LSB)

    // 3.Sguan.safe参数设计(维护驱动器安全)
    user->safe.VBUS_MAX = 27.0f;                    // (float)母线电压值波动MAX阈值
    user->safe.VBUS_MIM = 18.0f;                    // (float)母线电压值波动MIN阈值
    user->safe.VBUS_watchdog_limit = 1000;          // (uint32_t)看门狗

    user->safe.Temp_MAX = 60.0f;                    // (float)驱动器允许最大温度
    user->safe.Temp_MIN = -20.0f;                   // (float)驱动器允许最小温度
    user->safe.Temp_watchdog_limit = 2e5;           // (uint32_t)看门狗_本板无温度采样,拉长避免误锁

    user->safe.Dcur_MAX = 20.0f;                    // (float)电机最大电流D轴限制
    user->safe.Qcur_MAX = 20.0f;                    // (float)电机最大电流Q轴限制
    user->safe.DQcur_watchdog_limit = 2e5;          // (uint32_t)看门狗

    user->safe.Current_limit = 0.5f;                // (float)电机->电流状态机判断的电流范围
    user->safe.Speed_limit = 5.0f;                  // (float)电机->速度状态机判断的速度范围
    user->safe.Position_limit = 1.0f;               // (float)电机->位置状态机判断的位置范围

    user->safe.DISABLED_watchdog_limit = 1e3;       // (uint32_t)看门狗

    // 4.Sguan.flag参数设计(非重要数据)
    user->flag.PWM_watchdog_limit = 10;             // (uint8_t)PWM错误次数限幅
}   


#endif // USERDATA_MOTOR_H

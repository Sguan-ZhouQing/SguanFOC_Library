#ifndef SGUAN_TRANSFER_H
#define SGUAN_TRANSFER_H

#include "Sguan_Config.h"

// ================================================================
// 半隐藏设计：
//   In/Out/Params 结构体公开 —— 用户要直接读写（喂信号、配参数）
//   Data 为不透明类型 —— 运行状态定义在 Sguan_Transfer.c，外部不可见
//   块实例从 transfer_xxx_get() 获取（静态池），data 指针自动绑定
// ================================================================

// --------------------------- TRANSFER1 典型一阶传递函数 ---------------------------
typedef struct{
    SguanQ input;
}Transfer1In;

typedef struct{
    SguanQ output;
}Transfer1Out;

typedef struct{
    SguanQ t;

    SguanQ num0;
    SguanQ num1;

    SguanQ den0;
    SguanQ den1;

    uint8_t recalculate_total_flag;
}Transfer1Params;

typedef struct Transfer1Data Transfer1Data;   // 不透明：运行状态

typedef struct Transfer1{
    Transfer1In in;
    Transfer1Out out;
    Transfer1Params params;
    Transfer1Data *data;                       // transfer_transfer1_get() 自动绑定
}Transfer1;
// --------------------------- TRANSFER2 典型二阶传递函数 ---------------------------
typedef struct{
    SguanQ input;
}Transfer2In;

typedef struct{
    SguanQ output;
}Transfer2Out;

typedef struct{
    SguanQ t;

    SguanQ num0;
    SguanQ num1;
    SguanQ num2;

    SguanQ den0;
    SguanQ den1;
    SguanQ den2;

    uint8_t recalculate_total_flag;
}Transfer2Params;

typedef struct Transfer2Data Transfer2Data;

typedef struct Transfer2{
    Transfer2In in;
    Transfer2Out out;
    Transfer2Params params;
    Transfer2Data *data;
}Transfer2;
// --------------------------- TRANSFER3 典型三阶传递函数 ---------------------------
typedef struct{
    SguanQ input;
}Transfer3In;

typedef struct{
    SguanQ output;
}Transfer3Out;

typedef struct{
    SguanQ t;

    SguanQ num0;
    SguanQ num1;
    SguanQ num2;
    SguanQ num3;

    SguanQ den0;
    SguanQ den1;
    SguanQ den2;
    SguanQ den3;

    uint8_t recalculate_total_flag;
}Transfer3Params;

typedef struct Transfer3Data Transfer3Data;

typedef struct Transfer3{
    Transfer3In in;
    Transfer3Out out;
    Transfer3Params params;
    Transfer3Data *data;
}Transfer3;
// --------------------------- TRANSFER4 典型四阶传递函数 ---------------------------
typedef struct{
    SguanQ input;
}Transfer4In;

typedef struct{
    SguanQ output;
}Transfer4Out;

typedef struct{
    SguanQ t;

    SguanQ num0;
    SguanQ num1;
    SguanQ num2;
    SguanQ num3;
    SguanQ num4;

    SguanQ den0;
    SguanQ den1;
    SguanQ den2;
    SguanQ den3;
    SguanQ den4;

    uint8_t recalculate_total_flag;
}Transfer4Params;

typedef struct Transfer4Data Transfer4Data;

typedef struct Transfer4{
    Transfer4In in;
    Transfer4Out out;
    Transfer4Params params;
    Transfer4Data *data;
}Transfer4;
// --------------------------- TRANSFER5 典型五阶传递函数 ---------------------------
typedef struct{
    SguanQ input;
}Transfer5In;

typedef struct{
    SguanQ output;
}Transfer5Out;

typedef struct{
    SguanQ t;

    SguanQ num0;
    SguanQ num1;
    SguanQ num2;
    SguanQ num3;
    SguanQ num4;
    SguanQ num5;

    SguanQ den0;
    SguanQ den1;
    SguanQ den2;
    SguanQ den3;
    SguanQ den4;
    SguanQ den5;

    uint8_t recalculate_total_flag;
}Transfer5Params;

typedef struct Transfer5Data Transfer5Data;

typedef struct Transfer5{
    Transfer5In in;
    Transfer5Out out;
    Transfer5Params params;
    Transfer5Data *data;
}Transfer5;
// --------------------------- INTEGRATOR 积分器 ---------------------------
typedef struct{
    SguanQ input;
}IntegratorIn;

typedef struct{
    SguanQ output;
}IntegratorOut;

typedef struct{
    SguanQ t;
}IntegratorParams;

typedef struct IntegratorData IntegratorData;

typedef struct Integrator{
    IntegratorIn in;
    IntegratorOut out;
    IntegratorParams params;
    IntegratorData *data;
}Integrator;
// --------------------------- DERIVATIVE 微分器 ---------------------------
typedef struct{
    SguanQ input;
}DerivativeIn;

typedef struct{
    SguanQ output;
}DerivativeOut;

typedef struct{
    SguanQ t;
    SguanQ wc;

    uint8_t recalculate_total_flag;
}DerivativeParams;

typedef struct DerivativeData DerivativeData;

typedef struct Derivative{
    DerivativeIn in;
    DerivativeOut out;
    DerivativeParams params;
    DerivativeData *data;
}Derivative;
// --------------------------- DFT 快速傅里叶变换 ---------------------------
typedef struct{
    SguanQ input;
}DftIn;

typedef struct{
    SguanQ output;
}DftOut;

typedef struct{
    SguanQ t;
    SguanQ wc;

    uint8_t recalculate_total_flag;
}DftParams;

typedef struct DftData DftData;

typedef struct Dft{
    DftIn in;
    DftOut out;
    DftParams params;
    DftData *data;
}Dft;
// --------------------------- HALL ---------------------------
typedef struct{
    SguanQ ref;
    SguanQ fbk;
}HallIn;

typedef struct{
    SguanQ output;
}HallOut;

typedef struct{
    SguanQ t;
    SguanQ wc;

    SguanQ kp;
    SguanQ ki;
    SguanQ kd;

    SguanQ out_max;
    SguanQ out_min;
    SguanQ int_max;
    SguanQ int_min;

    uint8_t recalculate_total_flag;
    uint8_t integral_frozen_flag;
}HallParams;

typedef struct HallData HallData;

typedef struct Hall{
    HallIn in;
    HallOut out;
    HallParams params;
    HallData *data;
}Hall;
// --------------------------- LADRC1 一阶线性自适应抗干扰控制 ---------------------------
typedef struct{
    SguanQ ref;
    SguanQ fbk;
}Ladrc1In;

typedef struct{
    SguanQ output;
}Ladrc1Out;

typedef struct{
    SguanQ t;
    SguanQ wc;

    SguanQ kp;
    SguanQ ki;
    SguanQ kd;

    SguanQ out_max;
    SguanQ out_min;
    SguanQ int_max;
    SguanQ int_min;

    uint8_t recalculate_total_flag;
    uint8_t integral_frozen_flag;
}Ladrc1Params;

typedef struct Ladrc1Data Ladrc1Data;

typedef struct Ladrc1{
    Ladrc1In in;
    Ladrc1Out out;
    Ladrc1Params params;
    Ladrc1Data *data;
}Ladrc1;
// --------------------------- LADRC2 二阶线性自适应抗干扰控制 ---------------------------
typedef struct{
    SguanQ ref;
    SguanQ fbk;
}Ladrc2In;

typedef struct{
    SguanQ output;
}Ladrc2Out;

typedef struct{
    SguanQ t;
    SguanQ wc;

    SguanQ kp;
    SguanQ ki;
    SguanQ kd;

    SguanQ out_max;
    SguanQ out_min;
    SguanQ int_max;
    SguanQ int_min;

    uint8_t recalculate_total_flag;
    uint8_t integral_frozen_flag;
}Ladrc2Params;

typedef struct Ladrc2Data Ladrc2Data;

typedef struct Ladrc2{
    Ladrc2In in;
    Ladrc2Out out;
    Ladrc2Params params;
    Ladrc2Data *data;
}Ladrc2;
// --------------------------- SMC 传统指数型趋近率的滑模控制 ---------------------------
typedef struct{
    SguanQ ref;
    SguanQ fbk;
}SmcIn;

typedef struct{
    SguanQ output;
}SmcOut;

typedef struct{
    SguanQ t;
    SguanQ wc;

    SguanQ kp;
    SguanQ ki;
    SguanQ kd;

    SguanQ out_max;
    SguanQ out_min;
    SguanQ int_max;
    SguanQ int_min;

    uint8_t recalculate_total_flag;
    uint8_t integral_frozen_flag;
}SmcParams;

typedef struct SmcData SmcData;

typedef struct Smc{
    SmcIn in;
    SmcOut out;
    SmcParams params;
    SmcData *data;
}Smc;
// --------------------------- DPCC 增量式电流预测控制 ---------------------------
typedef struct{
    SguanQ ref;
    SguanQ fbk;
}DpccIn;

typedef struct{
    SguanQ output;
}DpccOut;

typedef struct{
    SguanQ t;
    SguanQ wc;

    SguanQ kp;
    SguanQ ki;
    SguanQ kd;

    SguanQ out_max;
    SguanQ out_min;
    SguanQ int_max;
    SguanQ int_min;

    uint8_t recalculate_total_flag;
    uint8_t integral_frozen_flag;
}DpccParams;

typedef struct DpccData DpccData;

typedef struct Dpcc{
    DpccIn in;
    DpccOut out;
    DpccParams params;
    DpccData *data;
}Dpcc;
// --------------------------- PIR 比例积分谐振调节器 ---------------------------
typedef struct{
    SguanQ ref;
    SguanQ fbk;
}PirIn;

typedef struct{
    SguanQ output;
}PirOut;

typedef struct{
    SguanQ t;
    SguanQ wc;

    SguanQ kp;
    SguanQ ki;
    SguanQ kd;

    SguanQ out_max;
    SguanQ out_min;
    SguanQ int_max;
    SguanQ int_min;

    uint8_t recalculate_total_flag;
    uint8_t integral_frozen_flag;
}PirParams;

typedef struct PirData PirData;

typedef struct Pir{
    PirIn in;
    PirOut out;
    PirParams params;
    PirData *data;
}Pir;
// --------------------------- PID 传统闭环控制器 ---------------------------
typedef struct{
    SguanQ ref;
    SguanQ fbk;
}PidIn;

typedef struct{
    SguanQ output;
}PidOut;

typedef struct{
    SguanQ t;
    SguanQ wc;

    SguanQ kp;
    SguanQ ki;
    SguanQ kd;

    SguanQ out_max;
    SguanQ out_min;
    SguanQ int_max;
    SguanQ int_min;

    uint8_t recalculate_total_flag;
    uint8_t integral_frozen_flag;
}PidParams;

typedef struct PidData PidData;

typedef struct Pid{
    PidIn in;
    PidOut out;
    PidParams params;
    PidData *data;
}Pid;
// --------------------------- PLL 开环锁相环 ---------------------------
typedef struct{
    SguanQ error;
}PllIn;

typedef struct{
    SguanQ we;
    SguanQ re;
}PllOut;

typedef struct{
    SguanQ t;
    SguanQ wc;

    SguanQ kp;
    SguanQ ki;

    uint8_t recalculate_total_flag;
}PllParams;

typedef struct PllData PllData;

typedef struct Pll{
    PllIn in;
    PllOut out;
    PllParams params;
    PllData *data;
}Pll;
// --------------------------- LPF1 一阶低通滤波器 ---------------------------
typedef struct{
    SguanQ input;
}Lpf1In;

typedef struct{
    SguanQ output;
}Lpf1Out;

typedef struct{
    SguanQ t;
    SguanQ wc;

    uint8_t recalculate_total_flag;
}Lpf1Params;

typedef struct Lpf1Data Lpf1Data;

typedef struct Lpf1{
    Lpf1In in;
    Lpf1Out out;
    Lpf1Params params;
    Lpf1Data *data;
}Lpf1;
// --------------------------- LPF2 二阶低通滤波器 ---------------------------
typedef struct{
    SguanQ input;
}Lpf2In;

typedef struct{
    SguanQ output;
}Lpf2Out;

typedef struct{
    SguanQ t;
    SguanQ wc;

    uint8_t recalculate_total_flag;
}Lpf2Params;

typedef struct Lpf2Data Lpf2Data;

typedef struct Lpf2{
    Lpf2In in;
    Lpf2Out out;
    Lpf2Params params;
    Lpf2Data *data;
}Lpf2;
// --------------------------- HPF1 一阶高通滤波器 ---------------------------
typedef struct{
    SguanQ input;
}Hpf1In;

typedef struct{
    SguanQ output;
}Hpf1Out;

typedef struct{
    SguanQ t;
    SguanQ wc;

    uint8_t recalculate_total_flag;
}Hpf1Params;

typedef struct Hpf1Data Hpf1Data;

typedef struct Hpf1{
    Hpf1In in;
    Hpf1Out out;
    Hpf1Params params;
    Hpf1Data *data;
}Hpf1;
// --------------------------- HPF2 二阶高通滤波器 ---------------------------
typedef struct{
    SguanQ input;
}Hpf2In;

typedef struct{
    SguanQ output;
}Hpf2Out;

typedef struct{
    SguanQ t;
    SguanQ wc;

    uint8_t recalculate_total_flag;
}Hpf2Params;

typedef struct Hpf2Data Hpf2Data;

typedef struct Hpf2{
    Hpf2In in;
    Hpf2Out out;
    Hpf2Params params;
    Hpf2Data *data;
}Hpf2;
// --------------------------- BPF1 带通滤波器(一阶低通和高通串联) ---------------------------
typedef struct{
    SguanQ input;
}Bpf1In;

typedef struct{
    SguanQ output;
}Bpf1Out;

typedef struct{
    SguanQ t;
    SguanQ wc_low;
    SguanQ wc_high;

    uint8_t recalculate_total_flag;
}Bpf1Params;

typedef struct Bpf1Data Bpf1Data;

typedef struct Bpf1{
    Bpf1In in;
    Bpf1Out out;
    Bpf1Params params;
    Bpf1Data *data;
}Bpf1;
// --------------------------- BPF2 带通滤波器(典型二阶系统改型) ---------------------------
typedef struct{
    SguanQ input;
}Bpf2In;

typedef struct{
    SguanQ output;
}Bpf2Out;

typedef struct{
    SguanQ t;
    SguanQ wo;
    SguanQ zeta;

    uint8_t recalculate_total_flag;
}Bpf2Params;

typedef struct Bpf2Data Bpf2Data;

typedef struct Bpf2{
    Bpf2In in;
    Bpf2Out out;
    Bpf2Params params;
    Bpf2Data *data;
}Bpf2;
// --------------------------- NF 陷波滤波器(典型二阶系统改型) ---------------------------
typedef struct{
    SguanQ input;
}NfIn;

typedef struct{
    SguanQ output;
}NfOut;

typedef struct{
    SguanQ t;
    SguanQ wo;
    SguanQ zeta;

    uint8_t recalculate_total_flag;
}NfParams;

typedef struct NfData NfData;

typedef struct Nf{
    NfIn in;
    NfOut out;
    NfParams params;
    NfData *data;
}Nf;
// --------------------------- TPNF 陷波滤波器(三参数陷波滤波器) ---------------------------
typedef struct{
    SguanQ input;
}TpnfIn;

typedef struct{
    SguanQ output;
}TpnfOut;

typedef struct{
    SguanQ t;
    SguanQ wo;

    SguanQ k1;
    SguanQ k2;

    uint8_t recalculate_total_flag;
}TpnfParams;

typedef struct TpnfData TpnfData;

typedef struct Tpnf{
    TpnfIn in;
    TpnfOut out;
    TpnfParams params;
    TpnfData *data;
}Tpnf;
// --------------------------- DOB 超螺旋滑模扰动观测器 ---------------------------
typedef struct{
    SguanQ input;
}DobIn;

typedef struct{
    SguanQ output;
}DobOut;

typedef struct{
    SguanQ t;
    SguanQ wo;

    SguanQ k1;
    SguanQ k2;

    uint8_t recalculate_total_flag;
}DobParams;

typedef struct DobData DobData;

typedef struct Dob{
    DobIn in;
    DobOut out;
    DobParams params;
    DobData *data;
}Dob;
// --------------------------- RLS 电机参数在线辨识观测器 ---------------------------
typedef struct{
    SguanQ input;
}RlsIn;

typedef struct{
    SguanQ output;
}RlsOut;

typedef struct{
    SguanQ t;
    SguanQ wo;

    SguanQ k1;
    SguanQ k2;

    uint8_t recalculate_total_flag;
}RlsParams;

typedef struct RlsData RlsData;

typedef struct Rls{
    RlsIn in;
    RlsOut out;
    RlsParams params;
    RlsData *data;
}Rls;
// --------------------------- SMO (无感)滑模观测器 ---------------------------
typedef struct{
    SguanQ input;
}SmoIn;

typedef struct{
    SguanQ output;
}SmoOut;

typedef struct{
    SguanQ t;
    SguanQ wo;

    SguanQ k1;
    SguanQ k2;

    uint8_t recalculate_total_flag;
}SmoParams;

typedef struct SmoData SmoData;

typedef struct Smo{
    SmoIn in;
    SmoOut out;
    SmoParams params;
    SmoData *data;
}Smo;
// --------------------------- NLFO (无感)非线性磁链观测器 ---------------------------
typedef struct{
    SguanQ input;
}NlfoIn;

typedef struct{
    SguanQ output;
}NlfoOut;

typedef struct{
    SguanQ t;
    SguanQ wo;

    SguanQ k1;
    SguanQ k2;

    uint8_t recalculate_total_flag;
}NlfoParams;

typedef struct NlfoData NlfoData;

typedef struct Nlfo{
    NlfoIn in;
    NlfoOut out;
    NlfoParams params;
    NlfoData *data;
}Nlfo;
// --------------------------- NLFO (无感)非线性磁链观测器 ---------------------------
typedef struct{
    SguanQ input;
}VcfoIn;

typedef struct{
    SguanQ output;
}VcfoOut;

typedef struct{
    SguanQ t;
    SguanQ wo;

    SguanQ k1;
    SguanQ k2;

    uint8_t recalculate_total_flag;
}VcfoParams;

typedef struct VcfoData VcfoData;

typedef struct Vcfo{
    VcfoIn in;
    VcfoOut out;
    VcfoParams params;
    VcfoData *data;
}Vcfo;
// --------------------------- HFI (无感)高频正弦波注入 ---------------------------
typedef struct{
    SguanQ input;
}HfiIn;

typedef struct{
    SguanQ output;
}HfiOut;

typedef struct{
    SguanQ t;
    SguanQ wo;

    SguanQ k1;
    SguanQ k2;

    uint8_t recalculate_total_flag;
}HfiParams;

typedef struct HfiData HfiData;

typedef struct Hfi{
    HfiIn in;
    HfiOut out;
    HfiParams params;
    HfiData *data;
}Hfi;
// --------------------------- ROLO (无感)降阶龙伯格观测器 ---------------------------
typedef struct{
    SguanQ input;
}RoloIn;

typedef struct{
    SguanQ output;
}RoloOut;

typedef struct{
    SguanQ t;
    SguanQ wo;

    SguanQ k1;
    SguanQ k2;

    uint8_t recalculate_total_flag;
}RoloParams;

typedef struct RoloData RoloData;

typedef struct Rolo{
    RoloIn in;
    RoloOut out;
    RoloParams params;
    RoloData *data;
}Rolo;
// --------------------------- MARS (无感)模型参考自适应观测器 ---------------------------
typedef struct{
    SguanQ input;
}MarsIn;

typedef struct{
    SguanQ output;
}MarsOut;

typedef struct{
    SguanQ t;
    SguanQ wo;

    SguanQ k1;
    SguanQ k2;

    uint8_t recalculate_total_flag;
}MarsParams;

typedef struct MarsData MarsData;

typedef struct Mars{
    MarsIn in;
    MarsOut out;
    MarsParams params;
    MarsData *data;
}Mars;
// --------------------------- EKF (无感)扩展卡尔曼滤波 ---------------------------
typedef struct{
    SguanQ input;
}EkfIn;

typedef struct{
    SguanQ output;
}EkfOut;

typedef struct{
    SguanQ t;
    SguanQ wo;

    SguanQ k1;
    SguanQ k2;

    uint8_t recalculate_total_flag;
}EkfParams;

typedef struct EkfData EkfData;

typedef struct Ekf{
    EkfIn in;
    EkfOut out;
    EkfParams params;
    EkfData *data;
}Ekf;
// --------------------------- DELAY1 延时函数(延时一拍) ---------------------------
typedef struct{
    SguanQ input;
}Delay1In;

typedef struct{
    SguanQ output;
}Delay1Out;

typedef struct Delay1{
    Delay1In in;
    Delay1Out out;
}Delay1;
// --------------------------- DELAY2 延时函数(延时两拍) ---------------------------
typedef struct{
    SguanQ input;
}Delay2In;

typedef struct{
    SguanQ output;
}Delay2Out;

typedef struct Delay2{
    Delay2In in;
    Delay2Out out;
}Delay2;
// --------------------------- DELAY3 延时函数(延时三拍) ---------------------------
typedef struct{
    SguanQ input;
}Delay3In;

typedef struct{
    SguanQ output;
}Delay3Out;

typedef struct Delay3{
    Delay3In in;
    Delay3Out out;
}Delay3;

// --------------------------- 单一功能模块（无 Data：全可见） ---------------------------
typedef struct{
    SguanQ input;
}SineIn;

typedef struct{
    SguanQ output;
}SineOut;

typedef struct Sine{
    SineIn in;
    SineOut out;
}Sine;
// --------------------------- Cosine 余弦发生器 ---------------------------
typedef struct{
    SguanQ input;
}CosineIn;

typedef struct{
    SguanQ output;
}CosineOut;

typedef struct Cosine{
    CosineIn in;
    CosineOut out;
}Cosine;
// --------------------------- Sign 符号函数 ---------------------------
typedef struct{
    SguanQ we;
    SguanQ wh;
}SignIn;

typedef struct{
    SguanQ output;
}SignOut;

typedef struct Sign{
    SignIn in;
    SignOut out;
}Sign;
// --------------------------- Clarke 克拉克变换 ---------------------------
typedef struct{
    SguanQ we;
    SguanQ wh;
}ClarkeIn;

typedef struct{
    SguanQ output;
}ClarkeOut;

typedef struct Clarke{
    ClarkeIn in;
    ClarkeOut out;
}Clarke;
// --------------------------- Park 帕克变换 ---------------------------
typedef struct{
    SguanQ we;
    SguanQ wh;
}ParkIn;

typedef struct{
    SguanQ output;
}ParkOut;

typedef struct Park{
    ParkIn in;
    ParkOut out;
}Park;
// --------------------------- Ipark 帕克逆变换 ---------------------------
typedef struct{
    SguanQ we;
    SguanQ wh;
}IparkIn;

typedef struct{
    SguanQ output;
}IparkOut;

typedef struct Ipark{
    IparkIn in;
    IparkOut out;
}Ipark;
// --------------------------- Spwm0 零序注入的SPWM模块 ---------------------------
typedef struct{
    SguanQ we;
    SguanQ wh;
}Spwm0In;

typedef struct{
    SguanQ output;
}Spwm0Out;

typedef struct Spwm0{
    Spwm0In in;
    Spwm0Out out;
}Spwm0;
// --------------------------- Spwm 普通SPWM模块 ---------------------------
typedef struct{
    SguanQ we;
    SguanQ wh;
}SpwmIn;

typedef struct{
    SguanQ output;
}SpwmOut;

typedef struct Spwm{
    SpwmIn in;
    SpwmOut out;
}Spwm;
// --------------------------- Svpwm 七段式SVPWM模块 ---------------------------
typedef struct{
    SguanQ we;
    SguanQ wh;
}SvpwmIn;

typedef struct{
    SguanQ output;
}SvpwmOut;

typedef struct Svpwm{
    SvpwmIn in;
    SvpwmOut out;
}Svpwm;
// --------------------------- SingleRs 单电阻采样函数（有 Data：半隐藏） ---------------------------
typedef struct{
    SguanQ input;
}SingleRsIn;

typedef struct{
    SguanQ output;
}SingleRsOut;

typedef struct{
    SguanQ t;
    SguanQ wo;

    SguanQ k1;
    SguanQ k2;

    uint8_t recalculate_total_flag;
}SingleRsParams;

typedef struct SingleRs{
    SingleRsIn in;
    SingleRsOut out;
    SingleRsParams params;
}SingleRs;
// --------------------------- SinCos 正余弦发生器 ---------------------------
typedef struct{
    SguanQ input;
}SinCosIn;

typedef struct{
    SguanQ sine;
    SguanQ cosine;
}SinCosOut;

typedef struct SinCos{
    SinCosIn in;
    SinCosOut out;
}SinCos;
// --------------------------- Tan 正切求解器 ---------------------------
typedef struct{
    SguanQ input;
}TanIn;

typedef struct{
    SguanQ output;
}TanOut;

typedef struct Tan{
    TanIn in;
    TanOut out;
}Tan;
// --------------------------- Atan 反正切求解器 ---------------------------
typedef struct{
    SguanQ input;
}AtanIn;

typedef struct{
    SguanQ output;
}AtanOut;

typedef struct Atan{
    AtanIn in;
    AtanOut out;
}Atan;
// --------------------------- Limit 限幅函数 ---------------------------
typedef struct{
    SguanQ input;
}LimitIn;

typedef struct{
    SguanQ output;
}LimitOut;

typedef struct{
    SguanQ out_max;
    SguanQ out_min;
}LimitParams;

typedef struct Limit{
    LimitIn in;
    LimitOut out;
    LimitParams params;
}Limit;
// --------------------------- Swpwm 电调PWM无感方波 ---------------------------
typedef struct{
    SguanQ we;
    SguanQ wh;
}SwpwmIn;

typedef struct{
    SguanQ output;
}SwpwmOut;

typedef struct Swpwm{
    SwpwmIn in;
    SwpwmOut out;
}Swpwm;

void transfer_transfer1_init(Transfer1 *transfer);
void transfer_transfer2_init(Transfer2 *transfer);
void transfer_transfer3_init(Transfer3 *transfer);
void transfer_transfer4_init(Transfer4 *transfer);
void transfer_transfer5_init(Transfer5 *transfer);
void transfer_integrator_init(Integrator *integrator);
void transfer_derivative_init(Derivative *derivative);
void transfer_dft_init(Dft *dft);
void transfer_hall_init(Hall *hall);
void transfer_ladrc1_init(Ladrc1 *ladrc);
void transfer_ladrc2_init(Ladrc2 *ladrc);
void transfer_smc_init(Smc *smc);
void transfer_dpcc_init(Dpcc *dpcc);
void transfer_pir_init(Pir *pir);
void transfer_pid_init(Pid *pid);
void transfer_pll_init(Pll *pll);
void transfer_lpf1_init(Lpf1 *lpf);
void transfer_lpf2_init(Lpf2 *lpf);
void transfer_hpf1_init(Hpf1 *hpf);
void transfer_hpf2_init(Hpf2 *hpf);
void transfer_bpf1_init(Bpf1 *bpf);
void transfer_bpf2_init(Bpf2 *bpf);
void transfer_nf_init(Nf *nf);
void transfer_tpnf_init(Tpnf *tpnf);
void transfer_dob_init(Dob *dob);
void transfer_rls_init(Rls *rls);
void transfer_smo_init(Smo *smo);
void transfer_nlfo_init(Nlfo *nlfo);
void transfer_vcfo_init(Vcfo *vcfo);
void transfer_hfi_init(Hfi *hfi);
void transfer_rolo_init(Rolo *rolo);
void transfer_mars_init(Mars *mars);
void transfer_ekf_init(Ekf *ekf);
void transfer_delay1_init(Delay1 *delay);
void transfer_delay2_init(Delay2 *delay);
void transfer_delay3_init(Delay3 *delay);

void transfer_transfer1_loop(Transfer1 *transfer);
void transfer_transfer2_loop(Transfer2 *transfer);
void transfer_transfer3_loop(Transfer3 *transfer);
void transfer_transfer4_loop(Transfer4 *transfer);
void transfer_transfer5_loop(Transfer5 *transfer);
void transfer_integrator_loop(Integrator *integrator);
void transfer_derivative_loop(Derivative *derivative);
void transfer_dft_loop(Dft *dft);
void transfer_hall_loop(Hall *hall);
void transfer_ladrc1_loop(Ladrc1 *ladrc);
void transfer_ladrc2_loop(Ladrc2 *ladrc);
void transfer_smc_loop(Smc *smc);
void transfer_dpcc_loop(Dpcc *dpcc);
void transfer_pir_loop(Pir *pir);
void transfer_pid_loop(Pid *pid);
void transfer_pll_loop(Pll *pll);
void transfer_lpf1_loop(Lpf1 *lpf);
void transfer_lpf2_loop(Lpf2 *lpf);
void transfer_hpf1_loop(Hpf1 *hpf);
void transfer_hpf2_loop(Hpf2 *hpf);
void transfer_bpf1_loop(Bpf1 *bpf);
void transfer_bpf2_loop(Bpf2 *bpf);
void transfer_nf_loop(Nf *nf);
void transfer_tpnf_loop(Tpnf *tpnf);
void transfer_dob_loop(Dob *dob);
void transfer_rls_loop(Rls *rls);
void transfer_smo_loop(Smo *smo);
void transfer_nlfo_loop(Nlfo *nlfo);
void transfer_vcfo_loop(Vcfo *vcfo);
void transfer_hfi_loop(Hfi *hfi);
void transfer_rolo_loop(Rolo *rolo);
void transfer_mars_loop(Mars *mars);
void transfer_ekf_loop(Ekf *ekf);
void transfer_delay1_loop(Delay1 *delay);
void transfer_delay2_loop(Delay2 *delay);
void transfer_delay3_loop(Delay3 *delay);
void transfer_sine_loop(Sine *sine);
void transfer_cosine_loop(Cosine *cosine);
void transfer_sincos_loop(SinCos *sincos);
void transfer_tan_loop(Tan *tan);
void transfer_atan_loop(Atan *atan);
void transfer_limit_loop(Limit *limit);
void transfer_sign_loop(Sign *sign);
void transfer_clarke_loop(Clarke *clarke);
void transfer_park_loop(Park *park);
void transfer_ipark_loop(Ipark *ipark);
void transfer_spwm0_loop(Spwm0 *spwm);
void transfer_spwm_loop(Spwm *spwm);
void transfer_svpwm_loop(Svpwm *svpwm);
void transfer_swpwm_loop(Swpwm *swpwm);
void transfer_singlers_loop(SingleRs *singlers);

// ==================== 实例获取：块实例统一由 .c 的静态池提供，用户只拿指针 ====================
#if CONFIG_TRANSFER1
Transfer1 *transfer_transfer1_get(uint8_t motor, int ch);
#endif
#if CONFIG_TRANSFER2
Transfer2 *transfer_transfer2_get(uint8_t motor, int ch);
#endif
#if CONFIG_TRANSFER3
Transfer3 *transfer_transfer3_get(uint8_t motor, int ch);
#endif
#if CONFIG_TRANSFER4
Transfer4 *transfer_transfer4_get(uint8_t motor, int ch);
#endif
#if CONFIG_TRANSFER5
Transfer5 *transfer_transfer5_get(uint8_t motor, int ch);
#endif
#if CONFIG_INTEGRATOR
Integrator *transfer_integrator_get(uint8_t motor, int ch);
#endif
#if CONFIG_DERIVATIVE
Derivative *transfer_derivative_get(uint8_t motor, int ch);
#endif
#if CONFIG_DFT
Dft *transfer_dft_get(uint8_t motor, int ch);
#endif
#if CONFIG_HALL
Hall *transfer_hall_get(uint8_t motor, int ch);
#endif
#if CONFIG_LADRC1
Ladrc1 *transfer_ladrc1_get(uint8_t motor, int ch);
#endif
#if CONFIG_LADRC2
Ladrc2 *transfer_ladrc2_get(uint8_t motor, int ch);
#endif
#if CONFIG_SMC
Smc *transfer_smc_get(uint8_t motor, int ch);
#endif
#if CONFIG_DPCC
Dpcc *transfer_dpcc_get(uint8_t motor, int ch);
#endif
#if CONFIG_PIR
Pir *transfer_pir_get(uint8_t motor, int ch);
#endif
#if CONFIG_PID
Pid *transfer_pid_get(uint8_t motor, int ch);
#endif
#if CONFIG_PLL
Pll *transfer_pll_get(uint8_t motor, int ch);
#endif
#if CONFIG_LPF1
Lpf1 *transfer_lpf1_get(uint8_t motor, int ch);
#endif
#if CONFIG_LPF2
Lpf2 *transfer_lpf2_get(uint8_t motor, int ch);
#endif
#if CONFIG_HPF1
Hpf1 *transfer_hpf1_get(uint8_t motor, int ch);
#endif
#if CONFIG_HPF2
Hpf2 *transfer_hpf2_get(uint8_t motor, int ch);
#endif
#if CONFIG_BPF1
Bpf1 *transfer_bpf1_get(uint8_t motor, int ch);
#endif
#if CONFIG_BPF2
Bpf2 *transfer_bpf2_get(uint8_t motor, int ch);
#endif
#if CONFIG_NF
Nf *transfer_nf_get(uint8_t motor, int ch);
#endif
#if CONFIG_TPNF
Tpnf *transfer_tpnf_get(uint8_t motor, int ch);
#endif
#if CONFIG_DOB
Dob *transfer_dob_get(uint8_t motor, int ch);
#endif
#if CONFIG_RLS
Rls *transfer_rls_get(uint8_t motor, int ch);
#endif
#if CONFIG_SMO
Smo *transfer_smo_get(uint8_t motor, int ch);
#endif
#if CONFIG_NLFO
Nlfo *transfer_nlfo_get(uint8_t motor, int ch);
#endif
#if CONFIG_VCFO
Vcfo *transfer_vcfo_get(uint8_t motor, int ch);
#endif
#if CONFIG_HFI
Hfi *transfer_hfi_get(uint8_t motor, int ch);
#endif
#if CONFIG_ROLO
Rolo *transfer_rolo_get(uint8_t motor, int ch);
#endif
#if CONFIG_MARS
Mars *transfer_mars_get(uint8_t motor, int ch);
#endif
#if CONFIG_EKF
Ekf *transfer_ekf_get(uint8_t motor, int ch);
#endif
#if CONFIG_DELAY1
Delay1 *transfer_delay1_get(uint8_t motor, int ch);
#endif
#if CONFIG_DELAY2
Delay2 *transfer_delay2_get(uint8_t motor, int ch);
#endif
#if CONFIG_DELAY3
Delay3 *transfer_delay3_get(uint8_t motor, int ch);
#endif
// ==================== 单一功能模块实例（单实例，无通道参数，顺序同 README） ====================
Sine *transfer_sine_get(uint8_t motor);
Cosine *transfer_cosine_get(uint8_t motor);
SinCos *transfer_sincos_get(uint8_t motor);
Tan *transfer_tan_get(uint8_t motor);
Atan *transfer_atan_get(uint8_t motor);
Limit *transfer_limit_get(uint8_t motor);
Sign *transfer_sign_get(uint8_t motor);
Clarke *transfer_clarke_get(uint8_t motor);
Park *transfer_park_get(uint8_t motor);
Ipark *transfer_ipark_get(uint8_t motor);
Spwm0 *transfer_spwm0_get(uint8_t motor);
Spwm *transfer_spwm_get(uint8_t motor);
Svpwm *transfer_svpwm_get(uint8_t motor);
Swpwm *transfer_swpwm_get(uint8_t motor);
SingleRs *transfer_singlers_get(uint8_t motor);
// ---------------------------工程模块Transfer---------------------------
void transfer_reinit_integrator(void *p);


#endif // SGUAN_TRANSFER_H

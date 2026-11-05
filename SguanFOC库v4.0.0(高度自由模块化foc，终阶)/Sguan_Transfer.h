#ifndef __SGUAN_TRANSFER_H
#define __SGUAN_TRANSFER_H

/* SguanFOC配置文件声明 */
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
}__LocalTransfer1In;

typedef struct{
    SguanQ output;
}__LocalTransfer1Out;

typedef struct{
    SguanQ t;

    SguanQ num0;
    SguanQ num1;

    SguanQ den0;
    SguanQ den1;

    uint8_t recalculate_total_flag;
}__LocalTransfer1Params;

typedef struct __LocalTransfer1Data __LocalTransfer1Data;   // 不透明：运行状态

typedef struct __LocalTransfer1{
    __LocalTransfer1In in;
    __LocalTransfer1Out out;
    __LocalTransfer1Params params;
    __LocalTransfer1Data *data;                       // transfer_transfer1_get() 自动绑定
}__LocalTransfer1;
// --------------------------- TRANSFER2 典型二阶传递函数 ---------------------------
typedef struct{
    SguanQ input;
}__LocalTransfer2In;

typedef struct{
    SguanQ output;
}__LocalTransfer2Out;

typedef struct{
    SguanQ t;

    SguanQ num0;
    SguanQ num1;
    SguanQ num2;

    SguanQ den0;
    SguanQ den1;
    SguanQ den2;

    uint8_t recalculate_total_flag;
}__LocalTransfer2Params;

typedef struct __LocalTransfer2Data __LocalTransfer2Data;

typedef struct __LocalTransfer2{
    __LocalTransfer2In in;
    __LocalTransfer2Out out;
    __LocalTransfer2Params params;
    __LocalTransfer2Data *data;
}__LocalTransfer2;
// --------------------------- TRANSFER3 典型三阶传递函数 ---------------------------
typedef struct{
    SguanQ input;
}__LocalTransfer3In;

typedef struct{
    SguanQ output;
}__LocalTransfer3Out;

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
}__LocalTransfer3Params;

typedef struct __LocalTransfer3Data __LocalTransfer3Data;

typedef struct __LocalTransfer3{
    __LocalTransfer3In in;
    __LocalTransfer3Out out;
    __LocalTransfer3Params params;
    __LocalTransfer3Data *data;
}__LocalTransfer3;
// --------------------------- TRANSFER4 典型四阶传递函数 ---------------------------
typedef struct{
    SguanQ input;
}__LocalTransfer4In;

typedef struct{
    SguanQ output;
}__LocalTransfer4Out;

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
}__LocalTransfer4Params;

typedef struct __LocalTransfer4Data __LocalTransfer4Data;

typedef struct __LocalTransfer4{
    __LocalTransfer4In in;
    __LocalTransfer4Out out;
    __LocalTransfer4Params params;
    __LocalTransfer4Data *data;
}__LocalTransfer4;
// --------------------------- TRANSFER5 典型五阶传递函数 ---------------------------
typedef struct{
    SguanQ input;
}__LocalTransfer5In;

typedef struct{
    SguanQ output;
}__LocalTransfer5Out;

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
}__LocalTransfer5Params;

typedef struct __LocalTransfer5Data __LocalTransfer5Data;

typedef struct __LocalTransfer5{
    __LocalTransfer5In in;
    __LocalTransfer5Out out;
    __LocalTransfer5Params params;
    __LocalTransfer5Data *data;
}__LocalTransfer5;
// --------------------------- INTEGRATOR 积分器 ---------------------------
typedef struct{
    SguanQ input;
}__LocalIntegratorIn;

typedef struct{
    SguanQ output;
}__LocalIntegratorOut;

typedef struct{
    SguanQ t;
}__LocalIntegratorParams;

typedef struct __LocalIntegratorData __LocalIntegratorData;

typedef struct __LocalIntegrator{
    __LocalIntegratorIn in;
    __LocalIntegratorOut out;
    __LocalIntegratorParams params;
    __LocalIntegratorData *data;
}__LocalIntegrator;
// --------------------------- DERIVATIVE 微分器 ---------------------------
typedef struct{
    SguanQ input;
}__LocalDerivativeIn;

typedef struct{
    SguanQ output;
}__LocalDerivativeOut;

typedef struct{
    SguanQ t;
    SguanQ wc;

    uint8_t recalculate_total_flag;
}__LocalDerivativeParams;

typedef struct __LocalDerivativeData __LocalDerivativeData;

typedef struct __LocalDerivative{
    __LocalDerivativeIn in;
    __LocalDerivativeOut out;
    __LocalDerivativeParams params;
    __LocalDerivativeData *data;
}__LocalDerivative;
// --------------------------- Curve S型曲线加减速函数 ---------------------------
typedef struct{
    SguanQ input;
}__LocalCurveIn;

typedef struct{
    SguanQ output;
}__LocalCurveOut;

typedef struct{
    SguanQ t;
    SguanQ wo;

    SguanQ k1;
    SguanQ k2;

    uint8_t recalculate_total_flag;
}__LocalCurveParams;

typedef struct __LocalCurveData __LocalCurveData;

typedef struct __LocalCurve{
    __LocalCurveIn in;
    __LocalCurveOut out;
    __LocalCurveParams params;
    __LocalCurveData *data;
}__LocalCurve;
// --------------------------- DFT 快速傅里叶变换 ---------------------------
typedef struct{
    SguanQ input;
}__LocalDftIn;

typedef struct{
    SguanQ output;
}__LocalDftOut;

typedef struct{
    SguanQ t;
    SguanQ wc;

    uint8_t recalculate_total_flag;
}__LocalDftParams;

typedef struct __LocalDftData __LocalDftData;

typedef struct __LocalDft{
    __LocalDftIn in;
    __LocalDftOut out;
    __LocalDftParams params;
    __LocalDftData *data;
}__LocalDft;
// --------------------------- HALL ---------------------------
typedef struct{
    SguanQ ref;
    SguanQ fbk;
}__LocalHallIn;

typedef struct{
    SguanQ output;
}__LocalHallOut;

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
}__LocalHallParams;

typedef struct __LocalHallData __LocalHallData;

typedef struct __LocalHall{
    __LocalHallIn in;
    __LocalHallOut out;
    __LocalHallParams params;
    __LocalHallData *data;
}__LocalHall;
// --------------------------- LADRC1 一阶线性自适应抗干扰控制 ---------------------------
typedef struct{
    SguanQ ref;
    SguanQ fbk;
}__LocalLadrc1In;

typedef struct{
    SguanQ output;
}__LocalLadrc1Out;

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
}__LocalLadrc1Params;

typedef struct __LocalLadrc1Data __LocalLadrc1Data;

typedef struct __LocalLadrc1{
    __LocalLadrc1In in;
    __LocalLadrc1Out out;
    __LocalLadrc1Params params;
    __LocalLadrc1Data *data;
}__LocalLadrc1;
// --------------------------- LADRC2 二阶线性自适应抗干扰控制 ---------------------------
typedef struct{
    SguanQ ref;
    SguanQ fbk;
}__LocalLadrc2In;

typedef struct{
    SguanQ output;
}__LocalLadrc2Out;

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
}__LocalLadrc2Params;

typedef struct __LocalLadrc2Data __LocalLadrc2Data;

typedef struct __LocalLadrc2{
    __LocalLadrc2In in;
    __LocalLadrc2Out out;
    __LocalLadrc2Params params;
    __LocalLadrc2Data *data;
}__LocalLadrc2;
// --------------------------- SMC 传统指数型趋近率的滑模控制 ---------------------------
typedef struct{
    SguanQ ref;
    SguanQ fbk;
}__LocalSmcIn;

typedef struct{
    SguanQ output;
}__LocalSmcOut;

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
}__LocalSmcParams;

typedef struct __LocalSmcData __LocalSmcData;

typedef struct __LocalSmc{
    __LocalSmcIn in;
    __LocalSmcOut out;
    __LocalSmcParams params;
    __LocalSmcData *data;
}__LocalSmc;
// --------------------------- DPCC 增量式电流预测控制 ---------------------------
typedef struct{
    SguanQ ref;
    SguanQ fbk;
}__LocalDpccIn;

typedef struct{
    SguanQ output;
}__LocalDpccOut;

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
}__LocalDpccParams;

typedef struct __LocalDpccData __LocalDpccData;

typedef struct __LocalDpcc{
    __LocalDpccIn in;
    __LocalDpccOut out;
    __LocalDpccParams params;
    __LocalDpccData *data;
}__LocalDpcc;
// --------------------------- PIR 比例积分谐振调节器 ---------------------------
typedef struct{
    SguanQ ref;
    SguanQ fbk;
}__LocalPirIn;

typedef struct{
    SguanQ output;
}__LocalPirOut;

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
}__LocalPirParams;

typedef struct __LocalPirData __LocalPirData;

typedef struct __LocalPir{
    __LocalPirIn in;
    __LocalPirOut out;
    __LocalPirParams params;
    __LocalPirData *data;
}__LocalPir;
// --------------------------- PID 传统闭环控制器 ---------------------------
typedef struct{
    SguanQ ref;
    SguanQ fbk;
}__LocalPidIn;

typedef struct{
    SguanQ output;
}__LocalPidOut;

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
}__LocalPidParams;

typedef struct __LocalPidData __LocalPidData;

typedef struct __LocalPid{
    __LocalPidIn in;
    __LocalPidOut out;
    __LocalPidParams params;
    __LocalPidData *data;
}__LocalPid;
// --------------------------- PLL 开环锁相环 ---------------------------
typedef struct{
    SguanQ error;
}__LocalPllIn;

typedef struct{
    SguanQ we;
    SguanQ re;
}__LocalPllOut;

typedef struct{
    SguanQ t;
    SguanQ wc;

    SguanQ kp;
    SguanQ ki;

    uint8_t recalculate_total_flag;
}__LocalPllParams;

typedef struct __LocalPllData __LocalPllData;

typedef struct __LocalPll{
    __LocalPllIn in;
    __LocalPllOut out;
    __LocalPllParams params;
    __LocalPllData *data;
}__LocalPll;
// --------------------------- LPF1 一阶低通滤波器 ---------------------------
typedef struct{
    SguanQ input;
}__LocalLpf1In;

typedef struct{
    SguanQ output;
}__LocalLpf1Out;

typedef struct{
    SguanQ t;
    SguanQ wc;

    uint8_t recalculate_total_flag;
}__LocalLpf1Params;

typedef struct __LocalLpf1Data __LocalLpf1Data;

typedef struct __LocalLpf1{
    __LocalLpf1In in;
    __LocalLpf1Out out;
    __LocalLpf1Params params;
    __LocalLpf1Data *data;
}__LocalLpf1;
// --------------------------- LPF2 二阶低通滤波器 ---------------------------
typedef struct{
    SguanQ input;
}__LocalLpf2In;

typedef struct{
    SguanQ output;
}__LocalLpf2Out;

typedef struct{
    SguanQ t;
    SguanQ wc;

    uint8_t recalculate_total_flag;
}__LocalLpf2Params;

typedef struct __LocalLpf2Data __LocalLpf2Data;

typedef struct __LocalLpf2{
    __LocalLpf2In in;
    __LocalLpf2Out out;
    __LocalLpf2Params params;
    __LocalLpf2Data *data;
}__LocalLpf2;
// --------------------------- HPF1 一阶高通滤波器 ---------------------------
typedef struct{
    SguanQ input;
}__LocalHpf1In;

typedef struct{
    SguanQ output;
}__LocalHpf1Out;

typedef struct{
    SguanQ t;
    SguanQ wc;

    uint8_t recalculate_total_flag;
}__LocalHpf1Params;

typedef struct __LocalHpf1Data __LocalHpf1Data;

typedef struct __LocalHpf1{
    __LocalHpf1In in;
    __LocalHpf1Out out;
    __LocalHpf1Params params;
    __LocalHpf1Data *data;
}__LocalHpf1;
// --------------------------- HPF2 二阶高通滤波器 ---------------------------
typedef struct{
    SguanQ input;
}__LocalHpf2In;

typedef struct{
    SguanQ output;
}__LocalHpf2Out;

typedef struct{
    SguanQ t;
    SguanQ wc;

    uint8_t recalculate_total_flag;
}__LocalHpf2Params;

typedef struct __LocalHpf2Data __LocalHpf2Data;

typedef struct __LocalHpf2{
    __LocalHpf2In in;
    __LocalHpf2Out out;
    __LocalHpf2Params params;
    __LocalHpf2Data *data;
}__LocalHpf2;
// --------------------------- BPF1 带通滤波器(一阶低通和高通串联) ---------------------------
typedef struct{
    SguanQ input;
}__LocalBpf1In;

typedef struct{
    SguanQ output;
}__LocalBpf1Out;

typedef struct{
    SguanQ t;
    SguanQ wc_low;
    SguanQ wc_high;

    uint8_t recalculate_total_flag;
}__LocalBpf1Params;

typedef struct __LocalBpf1Data __LocalBpf1Data;

typedef struct __LocalBpf1{
    __LocalBpf1In in;
    __LocalBpf1Out out;
    __LocalBpf1Params params;
    __LocalBpf1Data *data;
}__LocalBpf1;
// --------------------------- BPF2 带通滤波器(典型二阶系统改型) ---------------------------
typedef struct{
    SguanQ input;
}__LocalBpf2In;

typedef struct{
    SguanQ output;
}__LocalBpf2Out;

typedef struct{
    SguanQ t;
    SguanQ wo;
    SguanQ zeta;

    uint8_t recalculate_total_flag;
}__LocalBpf2Params;

typedef struct __LocalBpf2Data __LocalBpf2Data;

typedef struct __LocalBpf2{
    __LocalBpf2In in;
    __LocalBpf2Out out;
    __LocalBpf2Params params;
    __LocalBpf2Data *data;
}__LocalBpf2;
// --------------------------- BPF2 带通滤波器(典型二阶系统改型) ---------------------------
typedef struct{
    SguanQ input;
}__LocalSogiIn;

typedef struct{
    SguanQ output;
}__LocalSogiOut;

typedef struct{
    SguanQ t;
    SguanQ wo;
    SguanQ zeta;

    uint8_t recalculate_total_flag;
}__LocalSogiParams;

typedef struct __LocalSogiData __LocalSogiData;

typedef struct __LocalSogi{
    __LocalSogiIn in;
    __LocalSogiOut out;
    __LocalSogiParams params;
    __LocalSogiData *data;
}__LocalSogi;
// --------------------------- NF 陷波滤波器(典型二阶系统改型) ---------------------------
typedef struct{
    SguanQ input;
}__LocalNfIn;

typedef struct{
    SguanQ output;
}__LocalNfOut;

typedef struct{
    SguanQ t;
    SguanQ wo;
    SguanQ zeta;

    uint8_t recalculate_total_flag;
}__LocalNfParams;

typedef struct __LocalNfData __LocalNfData;

typedef struct __LocalNf{
    __LocalNfIn in;
    __LocalNfOut out;
    __LocalNfParams params;
    __LocalNfData *data;
}__LocalNf;
// --------------------------- TPNF 陷波滤波器(三参数陷波滤波器) ---------------------------
typedef struct{
    SguanQ input;
}__LocalTpnfIn;

typedef struct{
    SguanQ output;
}__LocalTpnfOut;

typedef struct{
    SguanQ t;
    SguanQ wo;

    SguanQ k1;
    SguanQ k2;

    uint8_t recalculate_total_flag;
}__LocalTpnfParams;

typedef struct __LocalTpnfData __LocalTpnfData;

typedef struct __LocalTpnf{
    __LocalTpnfIn in;
    __LocalTpnfOut out;
    __LocalTpnfParams params;
    __LocalTpnfData *data;
}__LocalTpnf;
// --------------------------- DOB 超螺旋滑模扰动观测器 ---------------------------
typedef struct{
    SguanQ input;
}__LocalDobIn;

typedef struct{
    SguanQ output;
}__LocalDobOut;

typedef struct{
    SguanQ t;
    SguanQ wo;

    SguanQ k1;
    SguanQ k2;

    uint8_t recalculate_total_flag;
}__LocalDobParams;

typedef struct __LocalDobData __LocalDobData;

typedef struct __LocalDob{
    __LocalDobIn in;
    __LocalDobOut out;
    __LocalDobParams params;
    __LocalDobData *data;
}__LocalDob;
// --------------------------- RLS 电机参数在线辨识观测器 ---------------------------
typedef struct{
    SguanQ input;
}__LocalRlsIn;

typedef struct{
    SguanQ output;
}__LocalRlsOut;

typedef struct{
    SguanQ t;
    SguanQ wo;

    SguanQ k1;
    SguanQ k2;

    uint8_t recalculate_total_flag;
}__LocalRlsParams;

typedef struct __LocalRlsData __LocalRlsData;

typedef struct __LocalRls{
    __LocalRlsIn in;
    __LocalRlsOut out;
    __LocalRlsParams params;
    __LocalRlsData *data;
}__LocalRls;
// --------------------------- SMO (无感)滑模观测器 ---------------------------
typedef struct{
    SguanQ input;
}__LocalSmoIn;

typedef struct{
    SguanQ output;
}__LocalSmoOut;

typedef struct{
    SguanQ t;
    SguanQ wo;

    SguanQ k1;
    SguanQ k2;

    uint8_t recalculate_total_flag;
}__LocalSmoParams;

typedef struct __LocalSmoData __LocalSmoData;

typedef struct __LocalSmo{
    __LocalSmoIn in;
    __LocalSmoOut out;
    __LocalSmoParams params;
    __LocalSmoData *data;
}__LocalSmo;
// --------------------------- NLFO (无感)非线性磁链观测器 ---------------------------
typedef struct{
    SguanQ input;
}__LocalNlfoIn;

typedef struct{
    SguanQ output;
}__LocalNlfoOut;

typedef struct{
    SguanQ t;
    SguanQ wo;

    SguanQ k1;
    SguanQ k2;

    uint8_t recalculate_total_flag;
}__LocalNlfoParams;

typedef struct __LocalNlfoData __LocalNlfoData;

typedef struct __LocalNlfo{
    __LocalNlfoIn in;
    __LocalNlfoOut out;
    __LocalNlfoParams params;
    __LocalNlfoData *data;
}__LocalNlfo;
// --------------------------- NLFO (无感)非线性磁链观测器 ---------------------------
typedef struct{
    SguanQ input;
}__LocalVcfoIn;

typedef struct{
    SguanQ output;
}__LocalVcfoOut;

typedef struct{
    SguanQ t;
    SguanQ wo;

    SguanQ k1;
    SguanQ k2;

    uint8_t recalculate_total_flag;
}__LocalVcfoParams;

typedef struct __LocalVcfoData __LocalVcfoData;

typedef struct __LocalVcfo{
    __LocalVcfoIn in;
    __LocalVcfoOut out;
    __LocalVcfoParams params;
    __LocalVcfoData *data;
}__LocalVcfo;
// --------------------------- HFI (无感)高频正弦波注入 ---------------------------
typedef struct{
    SguanQ input;
}__LocalHfiIn;

typedef struct{
    SguanQ output;
}__LocalHfiOut;

typedef struct{
    SguanQ t;
    SguanQ wo;

    SguanQ k1;
    SguanQ k2;

    uint8_t recalculate_total_flag;
}__LocalHfiParams;

typedef struct __LocalHfiData __LocalHfiData;

typedef struct __LocalHfi{
    __LocalHfiIn in;
    __LocalHfiOut out;
    __LocalHfiParams params;
    __LocalHfiData *data;
}__LocalHfi;
// --------------------------- ROLO (无感)降阶龙伯格观测器 ---------------------------
typedef struct{
    SguanQ input;
}__LocalRoloIn;

typedef struct{
    SguanQ output;
}__LocalRoloOut;

typedef struct{
    SguanQ t;
    SguanQ wo;

    SguanQ k1;
    SguanQ k2;

    uint8_t recalculate_total_flag;
}__LocalRoloParams;

typedef struct __LocalRoloData __LocalRoloData;

typedef struct __LocalRolo{
    __LocalRoloIn in;
    __LocalRoloOut out;
    __LocalRoloParams params;
    __LocalRoloData *data;
}__LocalRolo;
// --------------------------- MARS (无感)模型参考自适应观测器 ---------------------------
typedef struct{
    SguanQ input;
}__LocalMarsIn;

typedef struct{
    SguanQ output;
}__LocalMarsOut;

typedef struct{
    SguanQ t;
    SguanQ wo;

    SguanQ k1;
    SguanQ k2;

    uint8_t recalculate_total_flag;
}__LocalMarsParams;

typedef struct __LocalMarsData __LocalMarsData;

typedef struct __LocalMars{
    __LocalMarsIn in;
    __LocalMarsOut out;
    __LocalMarsParams params;
    __LocalMarsData *data;
}__LocalMars;
// --------------------------- EKF (无感)扩展卡尔曼滤波 ---------------------------
typedef struct{
    SguanQ input;
}__LocalEkfIn;

typedef struct{
    SguanQ output;
}__LocalEkfOut;

typedef struct{
    SguanQ t;
    SguanQ wo;

    SguanQ k1;
    SguanQ k2;

    uint8_t recalculate_total_flag;
}__LocalEkfParams;

typedef struct __LocalEkfData __LocalEkfData;

typedef struct __LocalEkf{
    __LocalEkfIn in;
    __LocalEkfOut out;
    __LocalEkfParams params;
    __LocalEkfData *data;
}__LocalEkf;
// --------------------------- DELAY1 延时函数(延时一拍) ---------------------------
typedef struct{
    SguanQ input;
}__LocalDelay1In;

typedef struct{
    SguanQ output;
}__LocalDelay1Out;

typedef struct __LocalDelay1{
    __LocalDelay1In in;
    __LocalDelay1Out out;
}__LocalDelay1;
// --------------------------- DELAY2 延时函数(延时两拍) ---------------------------
typedef struct{
    SguanQ input;
}__LocalDelay2In;

typedef struct{
    SguanQ output;
}__LocalDelay2Out;

typedef struct __LocalDelay2{
    __LocalDelay2In in;
    __LocalDelay2Out out;
}__LocalDelay2;
// --------------------------- DELAY3 延时函数(延时三拍) ---------------------------
typedef struct{
    SguanQ input;
}__LocalDelay3In;

typedef struct{
    SguanQ output;
}__LocalDelay3Out;

typedef struct __LocalDelay3{
    __LocalDelay3In in;
    __LocalDelay3Out out;
}__LocalDelay3;

// --------------------------- Sine 正弦发生器 ---------------------------
typedef struct{
    SguanQ input;
}__LocalSineIn;

typedef struct{
    SguanQ output;
}__LocalSineOut;

typedef struct __LocalSine{
    __LocalSineIn in;
    __LocalSineOut out;
}__LocalSine;
// --------------------------- Cosine 余弦发生器 ---------------------------
typedef struct{
    SguanQ input;
}__LocalCosineIn;

typedef struct{
    SguanQ output;
}__LocalCosineOut;

typedef struct __LocalCosine{
    __LocalCosineIn in;
    __LocalCosineOut out;
}__LocalCosine;
// --------------------------- SinCos 正余弦发生器 ---------------------------
typedef struct{
    SguanQ input;
}__LocalSinCosIn;

typedef struct{
    SguanQ sine;
    SguanQ cosine;
}__LocalSinCosOut;

typedef struct __LocalSinCos{
    __LocalSinCosIn in;
    __LocalSinCosOut out;
}__LocalSinCos;
// --------------------------- Tan 正切求解器 ---------------------------
typedef struct{
    SguanQ input;
}__LocalTanIn;

typedef struct{
    SguanQ output;
}__LocalTanOut;

typedef struct __LocalTan{
    __LocalTanIn in;
    __LocalTanOut out;
}__LocalTan;
// --------------------------- Atan 反正切求解器 ---------------------------
typedef struct{
    SguanQ input;
}__LocalAtanIn;

typedef struct{
    SguanQ output;
}__LocalAtanOut;

typedef struct __LocalAtan{
    __LocalAtanIn in;
    __LocalAtanOut out;
}__LocalAtan;
// --------------------------- Limit 限幅函数 ---------------------------
typedef struct{
    SguanQ input;
}__LocalLimitIn;

typedef struct{
    SguanQ output;
}__LocalLimitOut;

typedef struct{
    SguanQ out_max;
    SguanQ out_min;
}__LocalLimitParams;

typedef struct __LocalLimit{
    __LocalLimitIn in;
    __LocalLimitOut out;
    __LocalLimitParams params;
}__LocalLimit;
// --------------------------- Sign 符号函数 ---------------------------
typedef struct{
    SguanQ we;
    SguanQ wh;
}__LocalSignIn;

typedef struct{
    SguanQ output;
}__LocalSignOut;

typedef struct __LocalSign{
    __LocalSignIn in;
    __LocalSignOut out;
}__LocalSign;
// --------------------------- Clarke 克拉克变换 ---------------------------
typedef struct{
    SguanQ we;
    SguanQ wh;
}__LocalClarkeIn;

typedef struct{
    SguanQ output;
}__LocalClarkeOut;

typedef struct __LocalClarke{
    __LocalClarkeIn in;
    __LocalClarkeOut out;
}__LocalClarke;
// --------------------------- Park 帕克变换 ---------------------------
typedef struct{
    SguanQ we;
    SguanQ wh;
}__LocalParkIn;

typedef struct{
    SguanQ output;
}__LocalParkOut;

typedef struct __LocalPark{
    __LocalParkIn in;
    __LocalParkOut out;
}__LocalPark;
// --------------------------- Ipark 帕克逆变换 ---------------------------
typedef struct{
    SguanQ we;
    SguanQ wh;
}__LocalIparkIn;

typedef struct{
    SguanQ output;
}__LocalIparkOut;

typedef struct __LocalIpark{
    __LocalIparkIn in;
    __LocalIparkOut out;
}__LocalIpark;
// --------------------------- Spwm0 零序注入的SPWM模块 ---------------------------
typedef struct{
    SguanQ we;
    SguanQ wh;
}__LocalSpwm0In;

typedef struct{
    SguanQ output;
}__LocalSpwm0Out;

typedef struct __LocalSpwm0{
    __LocalSpwm0In in;
    __LocalSpwm0Out out;
}__LocalSpwm0;
// --------------------------- Spwm 普通SPWM模块 ---------------------------
typedef struct{
    SguanQ we;
    SguanQ wh;
}__LocalSpwmIn;

typedef struct{
    SguanQ output;
}__LocalSpwmOut;

typedef struct __LocalSpwm{
    __LocalSpwmIn in;
    __LocalSpwmOut out;
}__LocalSpwm;
// --------------------------- Svpwm 七段式SVPWM模块 ---------------------------
typedef struct{
    SguanQ we;
    SguanQ wh;
}__LocalSvpwmIn;

typedef struct{
    SguanQ output;
}__LocalSvpwmOut;

typedef struct __LocalSvpwm{
    __LocalSvpwmIn in;
    __LocalSvpwmOut out;
}__LocalSvpwm;
// --------------------------- Swpwm 电调PWM无感方波 ---------------------------
typedef struct{
    SguanQ we;
    SguanQ wh;
}__LocalSwpwmIn;

typedef struct{
    SguanQ output;
}__LocalSwpwmOut;

typedef struct __LocalSwpwm{
    __LocalSwpwmIn in;
    __LocalSwpwmOut out;
}__LocalSwpwm;
// --------------------------- SingleRs 单电阻采样函数 ---------------------------
typedef struct{
    SguanQ input;
}__LocalSingleRsIn;

typedef struct{
    SguanQ output;
}__LocalSingleRsOut;

typedef struct{
    SguanQ t;
    SguanQ wo;

    SguanQ k1;
    SguanQ k2;

    uint8_t recalculate_total_flag;
}__LocalSingleRsParams;

typedef struct __LocalSingleRs{
    __LocalSingleRsIn in;
    __LocalSingleRsOut out;
    __LocalSingleRsParams params;
}__LocalSingleRs;

// ---------------------------工程模块Transfer---------------------------
typedef struct{
    #if CONFIG_TRANSFER1 // 典型一阶传递函数
    __LocalTransfer1 *transfer1_ch[CONFIG_TRANSFER1];
    #endif // CONFIG_TRANSFER1

    #if CONFIG_TRANSFER2 // 典型二阶传递函数
    __LocalTransfer2 *transfer2_ch[CONFIG_TRANSFER2];
    #endif // CONFIG_TRANSFER2

    #if CONFIG_TRANSFER3 // 典型三阶传递函数
    __LocalTransfer3 *transfer3_ch[CONFIG_TRANSFER3];
    #endif // CONFIG_TRANSFER3

    #if CONFIG_TRANSFER4 // 典型四阶传递函数
    __LocalTransfer4 *transfer4_ch[CONFIG_TRANSFER4];
    #endif // CONFIG_TRANSFER4

    #if CONFIG_TRANSFER5 // 典型五阶传递函数
    __LocalTransfer5 *transfer5_ch[CONFIG_TRANSFER5];
    #endif // CONFIG_TRANSFER5

    #if CONFIG_INTEGRATOR // 积分器
    __LocalIntegrator *integrator_ch[CONFIG_INTEGRATOR];
    #endif // CONFIG_INTEGRATOR

    #if CONFIG_DERIVATIVE // 微分器
    __LocalDerivative *derivative_ch[CONFIG_DERIVATIVE];
    #endif // CONFIG_DERIVATIVE

    #if CONFIG_CURVE // S型曲线加减速函数
    __LocalCurve *curve_ch[CONFIG_CURVE];
    #endif // CONFIG_DFT

    #if CONFIG_DFT // 快速傅里叶变换
    __LocalDft *dft_ch[CONFIG_DFT];
    #endif // CONFIG_DFT

    #if CONFIG_HALL // 霍尔编码器
    __LocalHall *hall_ch[CONFIG_HALL];
    #endif // CONFIG_HALL

    #if CONFIG_LADRC1 // 一阶线性自适应抗干扰控制
    __LocalLadrc1 *ladrc1_ch[CONFIG_LADRC1];
    #endif // CONFIG_LADRC1

    #if CONFIG_LADRC2 // 二阶线性自适应抗干扰控制
    __LocalLadrc2 *ladrc2_ch[CONFIG_LADRC2];
    #endif // CONFIG_LADRC2

    #if CONFIG_SMC // 传统指数型趋近率的滑模控制
    __LocalSmc *smc_ch[CONFIG_SMC];
    #endif // CONFIG_SMC

    #if CONFIG_DPCC // 增量式电流预测控制
    __LocalDpcc *dpcc_ch[CONFIG_DPCC];
    #endif // CONFIG_DPCC

    #if CONFIG_PIR // 比例积分谐振调节器
    __LocalPir *pir_ch[CONFIG_PIR];
    #endif // CONFIG_PIR

    #if CONFIG_PID // 传统闭环控制器
    __LocalPid *pid_ch[CONFIG_PID];
    #endif // CONFIG_PID

    #if CONFIG_PLL // 开环锁相环
    __LocalPll *pll_ch[CONFIG_PLL];
    #endif // CONFIG_PLL

    #if CONFIG_LPF1 // 一阶低通滤波器
    __LocalLpf1 *lpf1_ch[CONFIG_LPF1];
    #endif // CONFIG_LPF1

    #if CONFIG_LPF2 // 二阶低通滤波器
    __LocalLpf2 *lpf2_ch[CONFIG_LPF2];
    #endif // CONFIG_LPF2

    #if CONFIG_HPF1 // 一阶高通滤波器
    __LocalHpf1 *hpf1_ch[CONFIG_HPF1];
    #endif // CONFIG_HPF1

    #if CONFIG_HPF2 // 二阶高通滤波器
    __LocalHpf2 *hpf2_ch[CONFIG_HPF2];
    #endif // CONFIG_HPF2

    #if CONFIG_BPF1 // 带通滤波器(一阶低通和高通串联)
    __LocalBpf1 *bpf1_ch[CONFIG_BPF1];
    #endif // CONFIG_BPF1

    #if CONFIG_BPF2 // 带通滤波器(典型二阶系统改型)
    __LocalBpf2 *bpf2_ch[CONFIG_BPF2];
    #endif // CONFIG_BPF2

    #if CONFIG_SOGI // 广义积分器(典型二阶系统改型)
    __LocalSogi *sogi_ch[CONFIG_SOGI];
    #endif // CONFIG_BPF2

    #if CONFIG_NF // 陷波滤波器(典型二阶系统改型)
    __LocalNf *nf_ch[CONFIG_NF];
    #endif // CONFIG_NF

    #if CONFIG_TPNF // 陷波滤波器(三参数陷波滤波器)
    __LocalTpnf *tpnf_ch[CONFIG_TPNF];
    #endif // CONFIG_TPNF

    #if CONFIG_DOB // 超螺旋滑模扰动观测器
    __LocalDob *dob_ch[CONFIG_DOB];
    #endif // CONFIG_DOB

    #if CONFIG_RLS // 电机参数在线辨识观测器
    __LocalRls *rls_ch[CONFIG_RLS];
    #endif // CONFIG_RLS

    #if CONFIG_SMO // (无感)滑模观测器
    __LocalSmo *smo_ch[CONFIG_SMO];
    #endif // CONFIG_SMO

    #if CONFIG_NLFO // (无感)非线性磁链观测器
    __LocalNlfo *nlfo_ch[CONFIG_NLFO];
    #endif // CONFIG_NLFO

    #if CONFIG_VCFO // (无感)电压电流互补磁链观测器
    __LocalVcfo *vcfo_ch[CONFIG_VCFO];
    #endif // CONFIG_VCFO

    #if CONFIG_HFI // (无感)高频正弦波注入
    __LocalHfi *hfi_ch[CONFIG_HFI];
    #endif // CONFIG_HFI

    #if CONFIG_ROLO // (无感)降阶龙伯格观测器
    __LocalRolo *rolo_ch[CONFIG_ROLO];
    #endif // CONFIG_ROLO

    #if CONFIG_MARS // (无感)模型参考自适应观测器
    __LocalMars *mars_ch[CONFIG_MARS];
    #endif // CONFIG_MARS

    #if CONFIG_EKF // (无感)扩展卡尔曼滤波
    __LocalEkf *ekf_ch[CONFIG_EKF];
    #endif // CONFIG_EKF

    #if CONFIG_DELAY1 // 延时函数(延时一拍)
    __LocalDelay1 *delay1_ch[CONFIG_DELAY1];
    #endif // CONFIG_DELAY1

    #if CONFIG_DELAY2 // 延时函数(延时两拍)
    __LocalDelay2 *delay2_ch[CONFIG_DELAY2];
    #endif // CONFIG_DELAY2

    #if CONFIG_DELAY3 // 延时函数(延时三拍)
    __LocalDelay3 *delay3_ch[CONFIG_DELAY3];
    #endif // CONFIG_DELAY3

    __LocalSine *sine;                   // 正弦发生器
    __LocalCosine *cosine;               // 余弦发生器
    __LocalSinCos *sincos;               // 正余弦发生器
    __LocalTan *tan;                     // 正切求解器
    __LocalAtan *atan;                   // 反正切求解器
    __LocalLimit *limit;                 // 限幅函数
    __LocalSign *sign;                   // 符号函数
    __LocalClarke *clarke;               // 克拉克变换
    __LocalPark *park;                   // 帕克变换
    __LocalIpark *ipark;                 // 帕克逆变换
    __LocalSpwm0 *spwm0;                 // 零序注入的SPWM模块
    __LocalSpwm *spwm;                   // 普通SPWM模块
    __LocalSvpwm *svpwm;                 // 七段式SVPWM模块
    __LocalSwpwm *swpwm;                 // 电调PWM无感方波
    __LocalSingleRs *singlers;           // 单电阻采样函数
}Transfer;

void transfer_init(Transfer *transfer);

void transfer_transfer1_loop(__LocalTransfer1 *transfer);
void transfer_transfer2_loop(__LocalTransfer2 *transfer);
void transfer_transfer3_loop(__LocalTransfer3 *transfer);
void transfer_transfer4_loop(__LocalTransfer4 *transfer);
void transfer_transfer5_loop(__LocalTransfer5 *transfer);
void transfer_integrator_loop(__LocalIntegrator *integrator);
void transfer_derivative_loop(__LocalDerivative *derivative);
void transfer_curve_loop(__LocalCurve *curve);
void transfer_dft_loop(__LocalDft *dft);
void transfer_hall_loop(__LocalHall *hall);
void transfer_ladrc1_loop(__LocalLadrc1 *ladrc);
void transfer_ladrc2_loop(__LocalLadrc2 *ladrc);
void transfer_smc_loop(__LocalSmc *smc);
void transfer_dpcc_loop(__LocalDpcc *dpcc);
void transfer_pir_loop(__LocalPir *pir);
void transfer_pid_loop(__LocalPid *pid);
void transfer_pll_loop(__LocalPll *pll);
void transfer_lpf1_loop(__LocalLpf1 *lpf);
void transfer_lpf2_loop(__LocalLpf2 *lpf);
void transfer_hpf1_loop(__LocalHpf1 *hpf);
void transfer_hpf2_loop(__LocalHpf2 *hpf);
void transfer_bpf1_loop(__LocalBpf1 *bpf);
void transfer_bpf2_loop(__LocalBpf2 *bpf);
void transfer_sogi_loop(__LocalSogi *sogi);
void transfer_nf_loop(__LocalNf *nf);
void transfer_tpnf_loop(__LocalTpnf *tpnf);
void transfer_dob_loop(__LocalDob *dob);
void transfer_rls_loop(__LocalRls *rls);
void transfer_smo_loop(__LocalSmo *smo);
void transfer_nlfo_loop(__LocalNlfo *nlfo);
void transfer_vcfo_loop(__LocalVcfo *vcfo);
void transfer_hfi_loop(__LocalHfi *hfi);
void transfer_rolo_loop(__LocalRolo *rolo);
void transfer_mars_loop(__LocalMars *mars);
void transfer_ekf_loop(__LocalEkf *ekf);
void transfer_delay1_loop(__LocalDelay1 *delay);
void transfer_delay2_loop(__LocalDelay2 *delay);
void transfer_delay3_loop(__LocalDelay3 *delay);

void transfer_sine_loop(__LocalSine *sine);
void transfer_cosine_loop(__LocalCosine *cosine);
void transfer_sincos_loop(__LocalSinCos *sincos);
void transfer_tan_loop(__LocalTan *tan);
void transfer_atan_loop(__LocalAtan *atan);
void transfer_limit_loop(__LocalLimit *limit);
void transfer_sign_loop(__LocalSign *sign);
void transfer_clarke_loop(__LocalClarke *clarke);
void transfer_park_loop(__LocalPark *park);
void transfer_ipark_loop(__LocalIpark *ipark);
void transfer_spwm0_loop(__LocalSpwm0 *spwm);
void transfer_spwm_loop(__LocalSpwm *spwm);
void transfer_svpwm_loop(__LocalSvpwm *svpwm);
void transfer_swpwm_loop(__LocalSwpwm *swpwm);
void transfer_singlers_loop(__LocalSingleRs *singlers);


#endif // SGUAN_TRANSFER_H

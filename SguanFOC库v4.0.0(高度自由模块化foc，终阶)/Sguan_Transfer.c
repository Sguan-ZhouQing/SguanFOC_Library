#include "Sguan_Transfer.h"

/* SguanFOC配置文件声明 */
#include "UnitLib_Patch.h"

// ================================================================================
// 1.局部函数__local基础
static void __local_transfer1_init(__LocalTransfer1 *transfer);
static void __local_transfer2_init(__LocalTransfer2 *transfer);
static void __local_transfer3_init(__LocalTransfer3 *transfer);
static void __local_transfer4_init(__LocalTransfer4 *transfer);
static void __local_transfer5_init(__LocalTransfer5 *transfer);
static void __local_integrator_init(__LocalIntegrator *integrator);
static void __local_derivative_init(__LocalDerivative *derivative);
static void __local_curve_init(__LocalCurve *curve);
static void __local_dft_init(__LocalDft *dft);
static void __local_hall_init(__LocalHall *hall);
static void __local_ladrc1_init(__LocalLadrc1 *ladrc);
static void __local_ladrc2_init(__LocalLadrc2 *ladrc);
static void __local_smc_init(__LocalSmc *smc);
static void __local_dpcc_init(__LocalDpcc *dpcc);
static void __local_pir_init(__LocalPir *pir);
static void __local_pid_init(__LocalPid *pid);
static void __local_pll_init(__LocalPll *pll);
static void __local_lpf1_init(__LocalLpf1 *lpf);
static void __local_lpf2_init(__LocalLpf2 *lpf);
static void __local_hpf1_init(__LocalHpf1 *hpf);
static void __local_hpf2_init(__LocalHpf2 *hpf);
static void __local_bpf1_init(__LocalBpf1 *bpf);
static void __local_bpf2_init(__LocalBpf2 *bpf);
static void __local_sogi_init(__LocalSogi *sogi);
static void __local_nf_init(__LocalNf *nf);
static void __local_tpnf_init(__LocalTpnf *tpnf);
static void __local_dob_init(__LocalDob *dob);
static void __local_rls_init(__LocalRls *rls);
static void __local_smo_init(__LocalSmo *smo);
static void __local_nfo_init(__LocalNfo *nfo);
static void __local_afo_init(__LocalAfo *afo);
static void __local_hfi_init(__LocalHfi *hfi);
static void __local_rolo_init(__LocalRolo *rolo);
static void __local_mars_init(__LocalMars *mars);
static void __local_ekf_init(__LocalEkf *ekf);
static void __local_delay1_init(__LocalDelay1 *delay);
static void __local_delay2_init(__LocalDelay2 *delay);
static void __local_delay3_init(__LocalDelay3 *delay);

// ================================================================================
// 2.实例获取静态池(声明)
#if CONFIG_TRANSFER1
__LocalTransfer1 *__local_transfer1_get(uint8_t motor, int ch);
#endif
#if CONFIG_TRANSFER2
__LocalTransfer2 *__local_transfer2_get(uint8_t motor, int ch);
#endif
#if CONFIG_TRANSFER3
__LocalTransfer3 *__local_transfer3_get(uint8_t motor, int ch);
#endif
#if CONFIG_TRANSFER4
__LocalTransfer4 *__local_transfer4_get(uint8_t motor, int ch);
#endif
#if CONFIG_TRANSFER5
__LocalTransfer5 *__local_transfer5_get(uint8_t motor, int ch);
#endif
#if CONFIG_INTEGRATOR
__LocalIntegrator *__local_integrator_get(uint8_t motor, int ch);
#endif
#if CONFIG_DERIVATIVE
__LocalDerivative *__local_derivative_get(uint8_t motor, int ch);
#endif
#if CONFIG_DFT
__LocalDft *__local_dft_get(uint8_t motor, int ch);
#endif
#if CONFIG_CURVE
__LocalCurve *__local_curve_get(uint8_t motor, int ch);
#endif
#if CONFIG_HALL
__LocalHall *__local_hall_get(uint8_t motor, int ch);
#endif
#if CONFIG_LADRC1
__LocalLadrc1 *__local_ladrc1_get(uint8_t motor, int ch);
#endif
#if CONFIG_LADRC2
__LocalLadrc2 *__local_ladrc2_get(uint8_t motor, int ch);
#endif
#if CONFIG_SMC
__LocalSmc *__local_smc_get(uint8_t motor, int ch);
#endif
#if CONFIG_DPCC
__LocalDpcc *__local_dpcc_get(uint8_t motor, int ch);
#endif
#if CONFIG_PIR
__LocalPir *__local_pir_get(uint8_t motor, int ch);
#endif
#if CONFIG_PID
__LocalPid *__local_pid_get(uint8_t motor, int ch);
#endif
#if CONFIG_PLL
__LocalPll *__local_pll_get(uint8_t motor, int ch);
#endif
#if CONFIG_LPF1
__LocalLpf1 *__local_lpf1_get(uint8_t motor, int ch);
#endif
#if CONFIG_LPF2
__LocalLpf2 *__local_lpf2_get(uint8_t motor, int ch);
#endif
#if CONFIG_HPF1
__LocalHpf1 *__local_hpf1_get(uint8_t motor, int ch);
#endif
#if CONFIG_HPF2
__LocalHpf2 *__local_hpf2_get(uint8_t motor, int ch);
#endif
#if CONFIG_BPF1
__LocalBpf1 *__local_bpf1_get(uint8_t motor, int ch);
#endif
#if CONFIG_BPF2
__LocalBpf2 *__local_bpf2_get(uint8_t motor, int ch);
#endif
#if CONFIG_SOGI
__LocalSogi *__local_sogi_get(uint8_t motor, int ch);
#endif
#if CONFIG_NF
__LocalNf *__local_nf_get(uint8_t motor, int ch);
#endif
#if CONFIG_TPNF
__LocalTpnf *__local_tpnf_get(uint8_t motor, int ch);
#endif
#if CONFIG_DOB
__LocalDob *__local_dob_get(uint8_t motor, int ch);
#endif
#if CONFIG_RLS
__LocalRls *__local_rls_get(uint8_t motor, int ch);
#endif
#if CONFIG_SMO
__LocalSmo *__local_smo_get(uint8_t motor, int ch);
#endif
#if CONFIG_NLFO
__LocalNfo *__local_nfo_get(uint8_t motor, int ch);
#endif
#if CONFIG_VCFO
__LocalAfo *__local_afo_get(uint8_t motor, int ch);
#endif
#if CONFIG_HFI
__LocalHfi *__local_hfi_get(uint8_t motor, int ch);
#endif
#if CONFIG_ROLO
__LocalRolo *__local_rolo_get(uint8_t motor, int ch);
#endif
#if CONFIG_MARS
__LocalMars *__local_mars_get(uint8_t motor, int ch);
#endif
#if CONFIG_EKF
__LocalEkf *__local_ekf_get(uint8_t motor, int ch);
#endif
#if CONFIG_DELAY1
__LocalDelay1 *__local_delay1_get(uint8_t motor, int ch);
#endif
#if CONFIG_DELAY2
__LocalDelay2 *__local_delay2_get(uint8_t motor, int ch);
#endif
#if CONFIG_DELAY3
__LocalDelay3 *__local_delay3_get(uint8_t motor, int ch);
#endif

__LocalSine *__local_sine_get(uint8_t motor);
__LocalCosine *__local_cosine_get(uint8_t motor);
__LocalSinCos *__local_sincos_get(uint8_t motor);
__LocalTan *__local_tan_get(uint8_t motor);
__LocalAtan *__local_atan_get(uint8_t motor);
__LocalLimit *__local_limit_get(uint8_t motor);
__LocalSign *__local_sign_get(uint8_t motor);
__LocalClarke *__local_clarke_get(uint8_t motor);
__LocalPark *__local_park_get(uint8_t motor);
__LocalIpark *__local_ipark_get(uint8_t motor);
__LocalSpwm *__local_spwm_get(uint8_t motor);
__LocalSvpwm *__local_svpwm_get(uint8_t motor);
__LocalSwpwm *__local_swpwm_get(uint8_t motor);
__LocalSingleRs *__local_singlers_get(uint8_t motor);

// ================================================================================
// 3.全局函数transfer基础
// (典型一阶传递函数，计数1)----------------------------------------------------------
struct __LocalTransfer1Data{
    SguanQ i;
    SguanQ o;

    SguanQ data_num[2];
    SguanQ data_den[2];
};

// (典型一阶传递函数，计数1)
static void __local_transfer1_init(__LocalTransfer1 *transfer){
    #if VALUE_MODE_MATH>=VALUE_MODE_Q15
    patch_transfer1_init(transfer);
    #else // VALUE_MODE_MATH
    // 传递函数运算系数固定计算
    transfer->data->data_num[0] = 2.0f*transfer->params.num1 + transfer->params.num0*transfer->params.t;
    transfer->data->data_num[1] = -2.0f*transfer->params.num1 + transfer->params.num0*transfer->params.t;

    transfer->data->data_den[0] = 2.0f*transfer->params.den1 + transfer->params.den0*transfer->params.t;
    transfer->data->data_num[1] = -2.0f*transfer->params.den1 + transfer->params.den0*transfer->params.t;

    // 初始化为零
    transfer->data->i = 0.0f;
    transfer->data->o = 0.0f;

    transfer->in.input = 0.0f;
    transfer->out.output = 0.0f;
    #endif // VALUE_MODE_MATH
}

// (典型一阶传递函数，计数1)
void transfer_transfer1_loop(__LocalTransfer1 *transfer){
    #if VALUE_MODE_MATH>=VALUE_MODE_Q15
    patch_transfer1_loop(transfer);
    #else // VALUE_MODE_MATH
    // 1.传递函数运算系数动态计算
    if (transfer->params.recalculate_total_flag){
        transfer->data->data_num[0] = 2.0f*transfer->params.num1 + transfer->params.num0*transfer->params.t;
        transfer->data->data_num[1] = -2.0f*transfer->params.num1 + transfer->params.num0*transfer->params.t;

        transfer->data->data_den[0] = 2.0f*transfer->params.den1 + transfer->params.den0*transfer->params.t;
        transfer->data->data_num[1] = -2.0f*transfer->params.den1 + transfer->params.den0*transfer->params.t;
    }

    // 2.运算传递函数
    transfer->out.output = (transfer->in.input*transfer->data->data_num[0] + transfer->data->i*transfer->data->data_num[1] - transfer->data->o*transfer->data->data_den[1])/transfer->data->data_den[0];

    // 3.更新历史数值
    transfer->data->i = transfer->in.input;
    transfer->data->o = transfer->out.output;
    #endif // VALUE_MODE_MATH
}

// (典型二阶传递函数，计数2)----------------------------------------------------------
struct __LocalTransfer2Data{
    SguanQ i[2];
    SguanQ o[2];

    SguanQ data_num[3];
    SguanQ data_den[3];
};

// (典型二阶传递函数，计数2)
static void __local_transfer2_init(__LocalTransfer2 *transfer){
    #if VALUE_MODE_MATH>=VALUE_MODE_Q15
    patch_transfer2_init(transfer);
    #else // VALUE_MODE_MATH
    // 传递函数运算系数固定计算
    transfer->data->data_num[0] = 4.0f*transfer->params.num2 + 2.0f*transfer->params.num1*transfer->params.t + transfer->params.num0*transfer->params.t*transfer->params.t;
    transfer->data->data_num[1] = -8.0f*transfer->params.num2 + 2.0f*transfer->params.num0*transfer->params.t*transfer->params.t;
    transfer->data->data_num[2] = 4.0f*transfer->params.num2 - 2.0f*transfer->params.num1*transfer->params.t + transfer->params.num0*transfer->params.t*transfer->params.t;
    
    transfer->data->data_den[0] = 4.0f*transfer->params.den2 + 2.0f*transfer->params.den1*transfer->params.t + transfer->params.den0*transfer->params.t*transfer->params.t;
    transfer->data->data_den[1] = -8.0f*transfer->params.den2 + 2.0f*transfer->params.den0*transfer->params.t*transfer->params.t;
    transfer->data->data_den[2] = 4.0f*transfer->params.den2 - 2.0f*transfer->params.den1*transfer->params.t + transfer->params.den0*transfer->params.t*transfer->params.t;

    // 初始化为零
    transfer->data->i[0] = 0.0f;
    transfer->data->i[1] = 0.0f;
    
    transfer->data->o[1] = 0.0f;
    transfer->data->o[1] = 0.0f;

    transfer->in.input = 0.0f;
    transfer->out.output = 0.0f;
    #endif // VALUE_MODE_MATH
}

// (典型二阶传递函数，计数2)
void transfer_transfer2_loop(__LocalTransfer2 *transfer){
    #if VALUE_MODE_MATH>=VALUE_MODE_Q15
    patch_transfer2_loop(transfer);
    #else // VALUE_MODE_MATH
    // 1.传递函数运算系数动态计算
    if (transfer->params.recalculate_total_flag){
        transfer->data->data_num[0] = 4.0f*transfer->params.num2 + 2.0f*transfer->params.num1*transfer->params.t + transfer->params.num0*transfer->params.t*transfer->params.t;
        transfer->data->data_num[1] = -8.0f*transfer->params.num2 + 2.0f*transfer->params.num0*transfer->params.t*transfer->params.t;
        transfer->data->data_num[2] = 4.0f*transfer->params.num2 - 2.0f*transfer->params.num1*transfer->params.t + transfer->params.num0*transfer->params.t*transfer->params.t;
        
        transfer->data->data_den[0] = 4.0f*transfer->params.den2 + 2.0f*transfer->params.den1*transfer->params.t + transfer->params.den0*transfer->params.t*transfer->params.t;
        transfer->data->data_den[1] = -8.0f*transfer->params.den2 + 2.0f*transfer->params.den0*transfer->params.t*transfer->params.t;
        transfer->data->data_den[2] = 4.0f*transfer->params.den2 - 2.0f*transfer->params.den1*transfer->params.t + transfer->params.den0*transfer->params.t*transfer->params.t;
    }

    // 2.运算传递函数
    transfer->out.output = (transfer->in.input*transfer->data->data_num[0] + transfer->data->i[0]*transfer->data->data_num[1] + transfer->data->i[1]*transfer->data->data_num[2] - transfer->data->o[0]*transfer->data->data_den[1] - transfer->data->o[1]*transfer->data->data_den[2])/transfer->data->data_den[0];

    // 3.更新历史数值
    transfer->data->i[1] = transfer->data->i[0];
    transfer->data->i[0] = transfer->in.input;

    transfer->data->o[1] = transfer->data->o[0];
    transfer->data->o[0] = transfer->out.output;
    #endif // VALUE_MODE_MATH
}

// (典型三阶传递函数，计数3)----------------------------------------------------------
struct __LocalTransfer3Data{
    SguanQ i[3];
    SguanQ o[3];

    SguanQ data_num[4];
    SguanQ data_den[4];
};

// (典型三阶传递函数，计数3)
static void __local_transfer3_init(__LocalTransfer3 *transfer){
    #if VALUE_MODE_MATH>=VALUE_MODE_Q15
    patch_transfer3_init(transfer);
    #else // VALUE_MODE_MATH
    // 传递函数运算系数固定计算
    transfer->data->data_num[0] = 8.0f*transfer->params.num3 + 4.0f*transfer->params.num2*transfer->params.t + 2.0f*transfer->params.num1*transfer->params.t*transfer->params.t + transfer->params.num0*transfer->params.t*transfer->params.t*transfer->params.t;
    transfer->data->data_num[1] = -24.0f*transfer->params.num3 - 4.0f*transfer->params.num2*transfer->params.t + 2.0f*transfer->params.num1*transfer->params.t*transfer->params.t + 3.0f*transfer->params.num0*transfer->params.t*transfer->params.t*transfer->params.t;
    transfer->data->data_num[2] = 24.0f*transfer->params.num3 - 4.0f*transfer->params.num2*transfer->params.t - 2.0f*transfer->params.num1*transfer->params.t*transfer->params.t + 3.0f*transfer->params.num0*transfer->params.t*transfer->params.t*transfer->params.t;
    transfer->data->data_num[3] = -8.0f*transfer->params.num3 + 4.0f*transfer->params.num2*transfer->params.t - 2.0f*transfer->params.num1*transfer->params.t*transfer->params.t + transfer->params.num0*transfer->params.t*transfer->params.t*transfer->params.t;
    
    transfer->data->data_den[0] = 8.0f*transfer->params.den3 + 4.0f*transfer->params.den2*transfer->params.t + 2.0f*transfer->params.den1*transfer->params.t*transfer->params.t + transfer->params.den0*transfer->params.t*transfer->params.t*transfer->params.t;
    transfer->data->data_den[1] = -24.0f*transfer->params.den3 - 4.0f*transfer->params.den2*transfer->params.t + 2.0f*transfer->params.den1*transfer->params.t*transfer->params.t + 3.0f*transfer->params.den0*transfer->params.t*transfer->params.t*transfer->params.t;
    transfer->data->data_den[2] = 24.0f*transfer->params.den3 - 4.0f*transfer->params.den2*transfer->params.t - 2.0f*transfer->params.den1*transfer->params.t*transfer->params.t + 3.0f*transfer->params.den0*transfer->params.t*transfer->params.t*transfer->params.t;
    transfer->data->data_den[3] = -8.0f*transfer->params.den3 + 4.0f*transfer->params.den2*transfer->params.t - 2.0f*transfer->params.den1*transfer->params.t*transfer->params.t + transfer->params.den0*transfer->params.t*transfer->params.t*transfer->params.t;
    
    // 初始化为零
    transfer->data->i[0] = 0.0f;
    transfer->data->i[1] = 0.0f;
    transfer->data->i[2] = 0.0f;
    
    transfer->data->o[0] = 0.0f;
    transfer->data->o[1] = 0.0f;
    transfer->data->o[2] = 0.0f;

    transfer->in.input = 0.0f;
    transfer->out.output = 0.0f;
    #endif // VALUE_MODE_MATH
}

// (典型三阶传递函数，计数3)
void transfer_transfer3_loop(__LocalTransfer3 *transfer){
    #if VALUE_MODE_MATH>=VALUE_MODE_Q15
    patch_transfer3_loop(transfer);
    #else // VALUE_MODE_MATH
    // 1.传递函数运算系数动态计算
    if (transfer->params.recalculate_total_flag){
        transfer->data->data_num[0] = 8.0f*transfer->params.num3 + 4.0f*transfer->params.num2*transfer->params.t + 2.0f*transfer->params.num1*transfer->params.t*transfer->params.t + transfer->params.num0*transfer->params.t*transfer->params.t*transfer->params.t;
        transfer->data->data_num[1] = -24.0f*transfer->params.num3 - 4.0f*transfer->params.num2*transfer->params.t + 2.0f*transfer->params.num1*transfer->params.t*transfer->params.t + 3.0f*transfer->params.num0*transfer->params.t*transfer->params.t*transfer->params.t;
        transfer->data->data_num[2] = 24.0f*transfer->params.num3 - 4.0f*transfer->params.num2*transfer->params.t - 2.0f*transfer->params.num1*transfer->params.t*transfer->params.t + 3.0f*transfer->params.num0*transfer->params.t*transfer->params.t*transfer->params.t;
        transfer->data->data_num[3] = -8.0f*transfer->params.num3 + 4.0f*transfer->params.num2*transfer->params.t - 2.0f*transfer->params.num1*transfer->params.t*transfer->params.t + transfer->params.num0*transfer->params.t*transfer->params.t*transfer->params.t;
        
        transfer->data->data_den[0] = 8.0f*transfer->params.den3 + 4.0f*transfer->params.den2*transfer->params.t + 2.0f*transfer->params.den1*transfer->params.t*transfer->params.t + transfer->params.den0*transfer->params.t*transfer->params.t*transfer->params.t;
        transfer->data->data_den[1] = -24.0f*transfer->params.den3 - 4.0f*transfer->params.den2*transfer->params.t + 2.0f*transfer->params.den1*transfer->params.t*transfer->params.t + 3.0f*transfer->params.den0*transfer->params.t*transfer->params.t*transfer->params.t;
        transfer->data->data_den[2] = 24.0f*transfer->params.den3 - 4.0f*transfer->params.den2*transfer->params.t - 2.0f*transfer->params.den1*transfer->params.t*transfer->params.t + 3.0f*transfer->params.den0*transfer->params.t*transfer->params.t*transfer->params.t;
        transfer->data->data_den[3] = -8.0f*transfer->params.den3 + 4.0f*transfer->params.den2*transfer->params.t - 2.0f*transfer->params.den1*transfer->params.t*transfer->params.t + transfer->params.den0*transfer->params.t*transfer->params.t*transfer->params.t;
    }

    // 2.运算传递函数
    transfer->out.output = (transfer->in.input*transfer->data->data_num[0] + transfer->data->i[0]*transfer->data->data_num[1] + transfer->data->i[1]*transfer->data->data_num[2] + transfer->data->i[2]*transfer->data->data_num[3] - transfer->data->o[0]*transfer->data->data_den[1] - transfer->data->o[1]*transfer->data->data_den[2] - transfer->data->o[2]*transfer->data->data_den[3])/transfer->data->data_den[0];

    // 3.更新历史数值
    transfer->data->i[2] = transfer->data->i[1];
    transfer->data->i[1] = transfer->data->i[0];
    transfer->data->i[0] = transfer->in.input;

    transfer->data->o[2] = transfer->data->o[1];
    transfer->data->o[1] = transfer->data->o[0];
    transfer->data->o[0] = transfer->out.output;
    #endif // VALUE_MODE_MATH
}

// (典型四阶传递函数，计数4)----------------------------------------------------------
struct __LocalTransfer4Data{
    SguanQ i[4];
    SguanQ o[4];

    SguanQ data_num[5];
    SguanQ data_den[5];
};

// (典型四阶传递函数，计数4)
static void __local_transfer4_init(__LocalTransfer4 *transfer){
    #if VALUE_MODE_MATH>=VALUE_MODE_Q15
    patch_transfer4_init(transfer);
    #else // VALUE_MODE_MATH
    // 传递函数运算系数固定计算
    transfer->data->data_num[0] = 16.0f*transfer->params.num4 + 8.0f*transfer->params.num3*transfer->params.t + 4.0f*transfer->params.num2*transfer->params.t*transfer->params.t + 2.0f*transfer->params.num1*transfer->params.t*transfer->params.t*transfer->params.t + transfer->params.num0*transfer->params.t*transfer->params.t*transfer->params.t*transfer->params.t;
    transfer->data->data_num[1] = -64.0f*transfer->params.num4 - 16.0f*transfer->params.num3*transfer->params.t + 4.0f*transfer->params.num1*transfer->params.t*transfer->params.t*transfer->params.t + 4.0f*transfer->params.num0*transfer->params.t*transfer->params.t*transfer->params.t*transfer->params.t;
    transfer->data->data_num[2] = 96.0f*transfer->params.num4 - 8.0f*transfer->params.num2*transfer->params.t*transfer->params.t + 6.0f*transfer->params.num0*transfer->params.t*transfer->params.t*transfer->params.t*transfer->params.t;
    transfer->data->data_num[3] = -64.0f*transfer->params.num4 + 16.0f*transfer->params.num3*transfer->params.t - 4.0f*transfer->params.num1*transfer->params.t*transfer->params.t*transfer->params.t + 4.0f*transfer->params.num0*transfer->params.t*transfer->params.t*transfer->params.t*transfer->params.t;
    transfer->data->data_num[4] = 16.0f*transfer->params.num4 - 8.0f*transfer->params.num3*transfer->params.t + 4.0f*transfer->params.num2*transfer->params.t*transfer->params.t - 2.0f*transfer->params.num1*transfer->params.t*transfer->params.t*transfer->params.t + transfer->params.num0*transfer->params.t*transfer->params.t*transfer->params.t*transfer->params.t;
    
    transfer->data->data_den[0] = 16.0f*transfer->params.den4 + 8.0f*transfer->params.den3*transfer->params.t + 4.0f*transfer->params.den2*transfer->params.t*transfer->params.t + 2.0f*transfer->params.den1*transfer->params.t*transfer->params.t*transfer->params.t + transfer->params.den0*transfer->params.t*transfer->params.t*transfer->params.t*transfer->params.t;
    transfer->data->data_den[1] = -64.0f*transfer->params.den4 - 16.0f*transfer->params.den3*transfer->params.t + 4.0f*transfer->params.den1*transfer->params.t*transfer->params.t*transfer->params.t + 4.0f*transfer->params.den0*transfer->params.t*transfer->params.t*transfer->params.t*transfer->params.t;
    transfer->data->data_den[2] = 96.0f*transfer->params.den4 - 8.0f*transfer->params.den2*transfer->params.t*transfer->params.t + 6.0f*transfer->params.den0*transfer->params.t*transfer->params.t*transfer->params.t*transfer->params.t;
    transfer->data->data_den[3] = -64.0f*transfer->params.den4 + 16.0f*transfer->params.den3*transfer->params.t - 4.0f*transfer->params.den1*transfer->params.t*transfer->params.t*transfer->params.t + 4.0f*transfer->params.den0*transfer->params.t*transfer->params.t*transfer->params.t*transfer->params.t;
    transfer->data->data_den[4] = 16.0f*transfer->params.den4 - 8.0f*transfer->params.den3*transfer->params.t + 4.0f*transfer->params.den2*transfer->params.t*transfer->params.t - 2.0f*transfer->params.den1*transfer->params.t*transfer->params.t*transfer->params.t + transfer->params.den0*transfer->params.t*transfer->params.t*transfer->params.t*transfer->params.t;

    // 初始化为零
    transfer->data->i[0] = 0.0f;
    transfer->data->i[1] = 0.0f;
    transfer->data->i[2] = 0.0f;
    transfer->data->i[3] = 0.0f;

    transfer->data->o[0] = 0.0f;
    transfer->data->o[1] = 0.0f;
    transfer->data->o[2] = 0.0f;
    transfer->data->o[3] = 0.0f;

    transfer->in.input = 0.0f;
    transfer->out.output = 0.0f;
    #endif // VALUE_MODE_MATH
}

// (典型四阶传递函数，计数4)
void transfer_transfer4_loop(__LocalTransfer4 *transfer){
    #if VALUE_MODE_MATH>=VALUE_MODE_Q15
    patch_transfer4_loop(transfer);
    #else // VALUE_MODE_MATH
    // 1.传递函数运算系数动态计算
    if (transfer->params.recalculate_total_flag){
        transfer->data->data_num[0] = 16.0f*transfer->params.num4 + 8.0f*transfer->params.num3*transfer->params.t + 4.0f*transfer->params.num2*transfer->params.t*transfer->params.t + 2.0f*transfer->params.num1*transfer->params.t*transfer->params.t*transfer->params.t + transfer->params.num0*transfer->params.t*transfer->params.t*transfer->params.t*transfer->params.t;
        transfer->data->data_num[1] = -64.0f*transfer->params.num4 - 16.0f*transfer->params.num3*transfer->params.t + 4.0f*transfer->params.num1*transfer->params.t*transfer->params.t*transfer->params.t + 4.0f*transfer->params.num0*transfer->params.t*transfer->params.t*transfer->params.t*transfer->params.t;
        transfer->data->data_num[2] = 96.0f*transfer->params.num4 - 8.0f*transfer->params.num2*transfer->params.t*transfer->params.t + 6.0f*transfer->params.num0*transfer->params.t*transfer->params.t*transfer->params.t*transfer->params.t;
        transfer->data->data_num[3] = -64.0f*transfer->params.num4 + 16.0f*transfer->params.num3*transfer->params.t - 4.0f*transfer->params.num1*transfer->params.t*transfer->params.t*transfer->params.t + 4.0f*transfer->params.num0*transfer->params.t*transfer->params.t*transfer->params.t*transfer->params.t;
        transfer->data->data_num[4] = 16.0f*transfer->params.num4 - 8.0f*transfer->params.num3*transfer->params.t + 4.0f*transfer->params.num2*transfer->params.t*transfer->params.t - 2.0f*transfer->params.num1*transfer->params.t*transfer->params.t*transfer->params.t + transfer->params.num0*transfer->params.t*transfer->params.t*transfer->params.t*transfer->params.t;
        
        transfer->data->data_den[0] = 16.0f*transfer->params.den4 + 8.0f*transfer->params.den3*transfer->params.t + 4.0f*transfer->params.den2*transfer->params.t*transfer->params.t + 2.0f*transfer->params.den1*transfer->params.t*transfer->params.t*transfer->params.t + transfer->params.den0*transfer->params.t*transfer->params.t*transfer->params.t*transfer->params.t;
        transfer->data->data_den[1] = -64.0f*transfer->params.den4 - 16.0f*transfer->params.den3*transfer->params.t + 4.0f*transfer->params.den1*transfer->params.t*transfer->params.t*transfer->params.t + 4.0f*transfer->params.den0*transfer->params.t*transfer->params.t*transfer->params.t*transfer->params.t;
        transfer->data->data_den[2] = 96.0f*transfer->params.den4 - 8.0f*transfer->params.den2*transfer->params.t*transfer->params.t + 6.0f*transfer->params.den0*transfer->params.t*transfer->params.t*transfer->params.t*transfer->params.t;
        transfer->data->data_den[3] = -64.0f*transfer->params.den4 + 16.0f*transfer->params.den3*transfer->params.t - 4.0f*transfer->params.den1*transfer->params.t*transfer->params.t*transfer->params.t + 4.0f*transfer->params.den0*transfer->params.t*transfer->params.t*transfer->params.t*transfer->params.t;
        transfer->data->data_den[4] = 16.0f*transfer->params.den4 - 8.0f*transfer->params.den3*transfer->params.t + 4.0f*transfer->params.den2*transfer->params.t*transfer->params.t - 2.0f*transfer->params.den1*transfer->params.t*transfer->params.t*transfer->params.t + transfer->params.den0*transfer->params.t*transfer->params.t*transfer->params.t*transfer->params.t;
    }
    
    // 2.运算传递函数
    transfer->out.output = (transfer->in.input*transfer->data->data_num[0] + transfer->data->i[0]*transfer->data->data_num[1] + transfer->data->i[1]*transfer->data->data_num[2] + transfer->data->i[2]*transfer->data->data_num[3] + transfer->data->i[3]*transfer->data->data_num[4] - transfer->data->o[0]*transfer->data->data_den[1] - transfer->data->o[1]*transfer->data->data_den[2] - transfer->data->o[2]*transfer->data->data_den[3] - transfer->data->o[3]*transfer->data->data_den[4])/transfer->data->data_den[0];

    // 3.更新历史数值
    transfer->data->i[3] = transfer->data->i[2];
    transfer->data->i[2] = transfer->data->i[1];
    transfer->data->i[1] = transfer->data->i[0];
    transfer->data->i[0] = transfer->in.input;

    transfer->data->o[3] = transfer->data->o[2];
    transfer->data->o[2] = transfer->data->o[1];
    transfer->data->o[1] = transfer->data->o[0];
    transfer->data->o[0] = transfer->out.output;
    #endif // VALUE_MODE_MATH
}

// (典型五阶传递函数，计数5)----------------------------------------------------------
struct __LocalTransfer5Data{
    SguanQ i[5];
    SguanQ o[5];

    SguanQ data_num[6];
    SguanQ data_den[6];
};

// (典型五阶传递函数，计数5)
static void __local_transfer5_init(__LocalTransfer5 *transfer){
    #if VALUE_MODE_MATH>=VALUE_MODE_Q15
    patch_transfer5_init(transfer);
    #else // VALUE_MODE_MATH
    // 传递函数运算系数固定计算
    transfer->data->data_num[0] = 32.0f*transfer->params.num5 + 16.0f*transfer->params.num4*transfer->params.t + 8.0f*transfer->params.num3*transfer->params.t*transfer->params.t + 4.0f*transfer->params.num2*transfer->params.t*transfer->params.t*transfer->params.t + 2.0f*transfer->params.num1*transfer->params.t*transfer->params.t*transfer->params.t*transfer->params.t + transfer->params.num0*transfer->params.t*transfer->params.t*transfer->params.t*transfer->params.t*transfer->params.t;
    transfer->data->data_num[1] = -160.0f*transfer->params.num5 - 48.0f*transfer->params.num4*transfer->params.t - 8.0f*transfer->params.num3*transfer->params.t*transfer->params.t + 4.0f*transfer->params.num2*transfer->params.t*transfer->params.t*transfer->params.t + 6.0f*transfer->params.num1*transfer->params.t*transfer->params.t*transfer->params.t*transfer->params.t + 5.0f*transfer->params.num0*transfer->params.t*transfer->params.t*transfer->params.t*transfer->params.t*transfer->params.t;
    transfer->data->data_num[2] = 320.0f*transfer->params.num5 + 32.0f*transfer->params.num4*transfer->params.t - 16.0f*transfer->params.num3*transfer->params.t*transfer->params.t - 8.0f*transfer->params.num2*transfer->params.t*transfer->params.t*transfer->params.t + 4.0f*transfer->params.num1*transfer->params.t*transfer->params.t*transfer->params.t*transfer->params.t + 10.0f*transfer->params.num0*transfer->params.t*transfer->params.t*transfer->params.t*transfer->params.t*transfer->params.t;
    transfer->data->data_num[3] = -320.0f*transfer->params.num5 + 32.0f*transfer->params.num4*transfer->params.t + 16.0f*transfer->params.num3*transfer->params.t*transfer->params.t - 8.0f*transfer->params.num2*transfer->params.t*transfer->params.t*transfer->params.t - 4.0f*transfer->params.num1*transfer->params.t*transfer->params.t*transfer->params.t*transfer->params.t + 10.0f*transfer->params.num0*transfer->params.t*transfer->params.t*transfer->params.t*transfer->params.t*transfer->params.t;
    transfer->data->data_num[4] = 160.0f*transfer->params.num5 - 48.0f*transfer->params.num4*transfer->params.t + 8.0f*transfer->params.num3*transfer->params.t*transfer->params.t + 4.0f*transfer->params.num2*transfer->params.t*transfer->params.t*transfer->params.t - 6.0f*transfer->params.num1*transfer->params.t*transfer->params.t*transfer->params.t*transfer->params.t + 5.0f*transfer->params.num0*transfer->params.t*transfer->params.t*transfer->params.t*transfer->params.t*transfer->params.t;
    transfer->data->data_num[5] = -32.0f*transfer->params.num5 + 16.0f*transfer->params.num4*transfer->params.t - 8.0f*transfer->params.num3*transfer->params.t*transfer->params.t + 4.0f*transfer->params.num2*transfer->params.t*transfer->params.t*transfer->params.t - 2.0f*transfer->params.num1*transfer->params.t*transfer->params.t*transfer->params.t*transfer->params.t + transfer->params.num0*transfer->params.t*transfer->params.t*transfer->params.t*transfer->params.t*transfer->params.t;
    
    transfer->data->data_den[0] = 32.0f*transfer->params.den5 + 16.0f*transfer->params.den4*transfer->params.t + 8.0f*transfer->params.den3*transfer->params.t*transfer->params.t + 4.0f*transfer->params.den2*transfer->params.t*transfer->params.t*transfer->params.t + 2.0f*transfer->params.den1*transfer->params.t*transfer->params.t*transfer->params.t*transfer->params.t + transfer->params.den0*transfer->params.t*transfer->params.t*transfer->params.t*transfer->params.t*transfer->params.t;
    transfer->data->data_den[1] = -160.0f*transfer->params.den5 - 48.0f*transfer->params.den4*transfer->params.t - 8.0f*transfer->params.den3*transfer->params.t*transfer->params.t + 4.0f*transfer->params.den2*transfer->params.t*transfer->params.t*transfer->params.t + 6.0f*transfer->params.den1*transfer->params.t*transfer->params.t*transfer->params.t*transfer->params.t + 5.0f*transfer->params.den0*transfer->params.t*transfer->params.t*transfer->params.t*transfer->params.t*transfer->params.t;
    transfer->data->data_den[2] = 320.0f*transfer->params.den5 + 32.0f*transfer->params.den4*transfer->params.t - 16.0f*transfer->params.den3*transfer->params.t*transfer->params.t - 8.0f*transfer->params.den2*transfer->params.t*transfer->params.t*transfer->params.t + 4.0f*transfer->params.den1*transfer->params.t*transfer->params.t*transfer->params.t*transfer->params.t + 10.0f*transfer->params.den0*transfer->params.t*transfer->params.t*transfer->params.t*transfer->params.t*transfer->params.t;
    transfer->data->data_den[3] = -320.0f*transfer->params.den5 + 32.0f*transfer->params.den4*transfer->params.t + 16.0f*transfer->params.den3*transfer->params.t*transfer->params.t - 8.0f*transfer->params.den2*transfer->params.t*transfer->params.t*transfer->params.t - 4.0f*transfer->params.den1*transfer->params.t*transfer->params.t*transfer->params.t*transfer->params.t + 10.0f*transfer->params.den0*transfer->params.t*transfer->params.t*transfer->params.t*transfer->params.t*transfer->params.t;
    transfer->data->data_den[4] = 160.0f*transfer->params.den5 - 48.0f*transfer->params.den4*transfer->params.t + 8.0f*transfer->params.den3*transfer->params.t*transfer->params.t + 4.0f*transfer->params.den2*transfer->params.t*transfer->params.t*transfer->params.t - 6.0f*transfer->params.den1*transfer->params.t*transfer->params.t*transfer->params.t*transfer->params.t + 5.0f*transfer->params.den0*transfer->params.t*transfer->params.t*transfer->params.t*transfer->params.t*transfer->params.t;
    transfer->data->data_den[5] = -32.0f*transfer->params.den5 + 16.0f*transfer->params.den4*transfer->params.t - 8.0f*transfer->params.den3*transfer->params.t*transfer->params.t + 4.0f*transfer->params.den2*transfer->params.t*transfer->params.t*transfer->params.t - 2.0f*transfer->params.den1*transfer->params.t*transfer->params.t*transfer->params.t*transfer->params.t + transfer->params.den0*transfer->params.t*transfer->params.t*transfer->params.t*transfer->params.t*transfer->params.t;
    
    // 初始化为零
    transfer->data->i[0] = 0.0f;
    transfer->data->i[1] = 0.0f;
    transfer->data->i[2] = 0.0f;
    transfer->data->i[3] = 0.0f;
    transfer->data->i[4] = 0.0f;

    transfer->data->o[0] = 0.0f;
    transfer->data->o[1] = 0.0f;
    transfer->data->o[2] = 0.0f;
    transfer->data->o[3] = 0.0f;
    transfer->data->o[4] = 0.0f;

    transfer->in.input = 0.0f;
    transfer->out.output = 0.0f;
    #endif // VALUE_MODE_MATH
}

// (典型五阶传递函数，计数5)
void transfer_transfer5_loop(__LocalTransfer5 *transfer){
    #if VALUE_MODE_MATH>=VALUE_MODE_Q15
    patch_transfer5_loop(transfer);
    #else // VALUE_MODE_MATH
    // 1.传递函数运算系数动态计算
    if (transfer->params.recalculate_total_flag){
        transfer->data->data_num[0] = 32.0f*transfer->params.num5 + 16.0f*transfer->params.num4*transfer->params.t + 8.0f*transfer->params.num3*transfer->params.t*transfer->params.t + 4.0f*transfer->params.num2*transfer->params.t*transfer->params.t*transfer->params.t + 2.0f*transfer->params.num1*transfer->params.t*transfer->params.t*transfer->params.t*transfer->params.t + transfer->params.num0*transfer->params.t*transfer->params.t*transfer->params.t*transfer->params.t*transfer->params.t;
        transfer->data->data_num[1] = -160.0f*transfer->params.num5 - 48.0f*transfer->params.num4*transfer->params.t - 8.0f*transfer->params.num3*transfer->params.t*transfer->params.t + 4.0f*transfer->params.num2*transfer->params.t*transfer->params.t*transfer->params.t + 6.0f*transfer->params.num1*transfer->params.t*transfer->params.t*transfer->params.t*transfer->params.t + 5.0f*transfer->params.num0*transfer->params.t*transfer->params.t*transfer->params.t*transfer->params.t*transfer->params.t;
        transfer->data->data_num[2] = 320.0f*transfer->params.num5 + 32.0f*transfer->params.num4*transfer->params.t - 16.0f*transfer->params.num3*transfer->params.t*transfer->params.t - 8.0f*transfer->params.num2*transfer->params.t*transfer->params.t*transfer->params.t + 4.0f*transfer->params.num1*transfer->params.t*transfer->params.t*transfer->params.t*transfer->params.t + 10.0f*transfer->params.num0*transfer->params.t*transfer->params.t*transfer->params.t*transfer->params.t*transfer->params.t;
        transfer->data->data_num[3] = -320.0f*transfer->params.num5 + 32.0f*transfer->params.num4*transfer->params.t + 16.0f*transfer->params.num3*transfer->params.t*transfer->params.t - 8.0f*transfer->params.num2*transfer->params.t*transfer->params.t*transfer->params.t - 4.0f*transfer->params.num1*transfer->params.t*transfer->params.t*transfer->params.t*transfer->params.t + 10.0f*transfer->params.num0*transfer->params.t*transfer->params.t*transfer->params.t*transfer->params.t*transfer->params.t;
        transfer->data->data_num[4] = 160.0f*transfer->params.num5 - 48.0f*transfer->params.num4*transfer->params.t + 8.0f*transfer->params.num3*transfer->params.t*transfer->params.t + 4.0f*transfer->params.num2*transfer->params.t*transfer->params.t*transfer->params.t - 6.0f*transfer->params.num1*transfer->params.t*transfer->params.t*transfer->params.t*transfer->params.t + 5.0f*transfer->params.num0*transfer->params.t*transfer->params.t*transfer->params.t*transfer->params.t*transfer->params.t;
        transfer->data->data_num[5] = -32.0f*transfer->params.num5 + 16.0f*transfer->params.num4*transfer->params.t - 8.0f*transfer->params.num3*transfer->params.t*transfer->params.t + 4.0f*transfer->params.num2*transfer->params.t*transfer->params.t*transfer->params.t - 2.0f*transfer->params.num1*transfer->params.t*transfer->params.t*transfer->params.t*transfer->params.t + transfer->params.num0*transfer->params.t*transfer->params.t*transfer->params.t*transfer->params.t*transfer->params.t;
        
        transfer->data->data_den[0] = 32.0f*transfer->params.den5 + 16.0f*transfer->params.den4*transfer->params.t + 8.0f*transfer->params.den3*transfer->params.t*transfer->params.t + 4.0f*transfer->params.den2*transfer->params.t*transfer->params.t*transfer->params.t + 2.0f*transfer->params.den1*transfer->params.t*transfer->params.t*transfer->params.t*transfer->params.t + transfer->params.den0*transfer->params.t*transfer->params.t*transfer->params.t*transfer->params.t*transfer->params.t;
        transfer->data->data_den[1] = -160.0f*transfer->params.den5 - 48.0f*transfer->params.den4*transfer->params.t - 8.0f*transfer->params.den3*transfer->params.t*transfer->params.t + 4.0f*transfer->params.den2*transfer->params.t*transfer->params.t*transfer->params.t + 6.0f*transfer->params.den1*transfer->params.t*transfer->params.t*transfer->params.t*transfer->params.t + 5.0f*transfer->params.den0*transfer->params.t*transfer->params.t*transfer->params.t*transfer->params.t*transfer->params.t;
        transfer->data->data_den[2] = 320.0f*transfer->params.den5 + 32.0f*transfer->params.den4*transfer->params.t - 16.0f*transfer->params.den3*transfer->params.t*transfer->params.t - 8.0f*transfer->params.den2*transfer->params.t*transfer->params.t*transfer->params.t + 4.0f*transfer->params.den1*transfer->params.t*transfer->params.t*transfer->params.t*transfer->params.t + 10.0f*transfer->params.den0*transfer->params.t*transfer->params.t*transfer->params.t*transfer->params.t*transfer->params.t;
        transfer->data->data_den[3] = -320.0f*transfer->params.den5 + 32.0f*transfer->params.den4*transfer->params.t + 16.0f*transfer->params.den3*transfer->params.t*transfer->params.t - 8.0f*transfer->params.den2*transfer->params.t*transfer->params.t*transfer->params.t - 4.0f*transfer->params.den1*transfer->params.t*transfer->params.t*transfer->params.t*transfer->params.t + 10.0f*transfer->params.den0*transfer->params.t*transfer->params.t*transfer->params.t*transfer->params.t*transfer->params.t;
        transfer->data->data_den[4] = 160.0f*transfer->params.den5 - 48.0f*transfer->params.den4*transfer->params.t + 8.0f*transfer->params.den3*transfer->params.t*transfer->params.t + 4.0f*transfer->params.den2*transfer->params.t*transfer->params.t*transfer->params.t - 6.0f*transfer->params.den1*transfer->params.t*transfer->params.t*transfer->params.t*transfer->params.t + 5.0f*transfer->params.den0*transfer->params.t*transfer->params.t*transfer->params.t*transfer->params.t*transfer->params.t;
        transfer->data->data_den[5] = -32.0f*transfer->params.den5 + 16.0f*transfer->params.den4*transfer->params.t - 8.0f*transfer->params.den3*transfer->params.t*transfer->params.t + 4.0f*transfer->params.den2*transfer->params.t*transfer->params.t*transfer->params.t - 2.0f*transfer->params.den1*transfer->params.t*transfer->params.t*transfer->params.t*transfer->params.t + transfer->params.den0*transfer->params.t*transfer->params.t*transfer->params.t*transfer->params.t*transfer->params.t;
    }
    
    // 2.运算传递函数
    transfer->out.output = (transfer->in.input*transfer->data->data_num[0] + transfer->data->i[0]*transfer->data->data_num[1] + transfer->data->i[1]*transfer->data->data_num[2] + transfer->data->i[2]*transfer->data->data_num[3] + transfer->data->i[3]*transfer->data->data_num[4] + transfer->data->i[4]*transfer->data->data_num[5] - transfer->data->o[0]*transfer->data->data_den[1] - transfer->data->o[1]*transfer->data->data_den[2] - transfer->data->o[2]*transfer->data->data_den[3] - transfer->data->o[3]*transfer->data->data_den[4] - transfer->data->o[4]*transfer->data->data_den[5])/transfer->data->data_den[0];

    // 3.更新历史数值
    transfer->data->i[4] = transfer->data->i[3];
    transfer->data->i[3] = transfer->data->i[2];
    transfer->data->i[2] = transfer->data->i[1];
    transfer->data->i[1] = transfer->data->i[0];
    transfer->data->i[0] = transfer->in.input;

    transfer->data->o[4] = transfer->data->o[3];
    transfer->data->o[3] = transfer->data->o[2];
    transfer->data->o[2] = transfer->data->o[1];
    transfer->data->o[1] = transfer->data->o[0];
    transfer->data->o[0] = transfer->out.output;
    #endif // VALUE_MODE_MATH
}

// (积分器，计数6)-------------------------------------------------------------------
struct __LocalIntegratorData{
    SguanQ i;

    SguanQ data_num;
};

// (积分器，计数6)
static void __local_integrator_init(__LocalIntegrator *integrator){
    #if VALUE_MODE_MATH>=VALUE_MODE_Q15
    patch_integrator_init(integrator);
    #else // VALUE_MODE_MATH
    // 传递函数运算系数固定计算
    integrator->data->data_num = integrator->params.t/2.0f;

    // 初始化为零
    integrator->data->i = 0.0f;

    integrator->in.input = 0.0f;
    integrator->out.output = 0.0f;
    #endif // VALUE_MODE_MATH
}

// (积分器，计数6)
void transfer_integrator_loop(__LocalIntegrator *integrator){
    #if VALUE_MODE_MATH>=VALUE_MODE_Q15
    patch_integrator_loop(integrator);
    #else // VALUE_MODE_MATH
    // 1.运算传递函数
    integrator->out.output += (integrator->in.input + integrator->data->i)*integrator->data->data_num;

    // 2.更新历史数值
    integrator->data->i = integrator->in.input;
    #endif // VALUE_MODE_MATH
}

// (微分器，计数7)-------------------------------------------------------------------
struct __LocalDerivativeData{
    SguanQ i;
    SguanQ o;

    SguanQ data_num[2];
    SguanQ data_den[2];
};

// (微分器，计数7)
static void __local_derivative_init(__LocalDerivative *derivative){
    #if VALUE_MODE_MATH>=VALUE_MODE_Q15
    patch_derivative_init(derivative);
    #else // VALUE_MODE_MATH
    // 传递函数运算系数固定计算
    derivative->data->data_num[0] = 2.0f*derivative->params.wc;
    derivative->data->data_num[1] = -2.0f*derivative->params.wc;

    derivative->data->data_den[0] = 2.0f + derivative->params.t*derivative->params.wc;
    derivative->data->data_den[1] = -2.0f + derivative->params.t*derivative->params.wc;

    // 初始化为零
    derivative->data->i = 0.0f;
    derivative->data->o = 0.0f;

    derivative->in.input = 0.0f;
    derivative->out.output = 0.0f;
    #endif // VALUE_MODE_MATH
}

// (微分器，计数7)
void transfer_derivative_loop(__LocalDerivative *derivative){
    #if VALUE_MODE_MATH>=VALUE_MODE_Q15
    patch_derivative_loop(derivative);
    #else // VALUE_MODE_MATH
    // 1.传递函数运算系数动态计算
    if (derivative->params.recalculate_total_flag){
        derivative->data->data_num[0] = 2.0f*derivative->params.wc;
        derivative->data->data_num[1] = -2.0f*derivative->params.wc;
        
        derivative->data->data_den[0] = 2.0f + derivative->params.t*derivative->params.wc;
        derivative->data->data_den[1] = -2.0f + derivative->params.t*derivative->params.wc;
    }

    // 2.运算传递函数
    derivative->out.output = (derivative->in.input*derivative->data->data_num[0] + derivative->data->i*derivative->data->data_num[1] - derivative->data->o*derivative->data->data_den[1])/derivative->data->data_den[0];

    // 3.更新历史数值
    derivative->data->i = derivative->in.input;
    derivative->data->o = derivative->out.output;
    #endif // VALUE_MODE_MATH
}

// (S型曲线加减速函数，计数8)---------------------------------------------------------
static const float speed_tab[100] = {
    0.000000000000000f, 0.000204061405384f, 0.000816243642475f, 0.001836552750319f,
    0.003264978528023f, 0.005101525224745f, 0.007346192840487f, 0.009998980909586f,
    0.013059905730188f, 0.016528937965631f, 0.020406091585755f, 0.024691363796592f,
    0.029384758323431f, 0.034486301243305f, 0.039995938539505f, 0.045913696289063f,
    0.052239574491978f, 0.058973576873541f, 0.066115736961365f, 0.073665976524353f,
    0.081624343991280f, 0.089990831911564f, 0.098765432834625f, 0.107948206365108f,
    0.117539055645466f, 0.127538025379181f, 0.137945115566254f, 0.148760333657265f,
    0.159983724355698f, 0.171615183353424f, 0.183654755353928f, 0.196102455258369f,
    0.208958268165588f, 0.222222283482552f, 0.235894337296486f, 0.249974519014359f,
    0.264462828636169f, 0.279359251260757f, 0.294663876295090f, 0.310376554727554f,
    0.326497316360474f, 0.343026250600815f, 0.359963268041611f, 0.377308517694473f,
    0.395061790943146f, 0.413223177194595f, 0.431792706251144f, 0.450770318508148f,
    0.470156192779541f, 0.489950060844421f, 0.510050058364868f, 0.529843926429749f,
    0.549229741096497f, 0.568207383155823f, 0.586776912212372f, 0.604938328266144f,
    0.622691571712494f, 0.640036821365356f, 0.656973838806152f, 0.673502743244171f,
    0.689623534679413f, 0.705336213111877f, 0.720640838146210f, 0.735537230968475f,
    0.750025570392609f, 0.764105737209320f, 0.777777791023254f, 0.791041791439056f,
    0.803897619247437f, 0.816345334053040f, 0.828384876251221f, 0.840016305446625f,
    0.851239740848541f, 0.862054944038391f, 0.872462034225464f, 0.882461011409760f,
    0.892051815986633f, 0.901234626770020f, 0.910009205341339f, 0.918375670909882f,
    0.926334083080292f, 0.933884322643280f, 0.941026449203491f, 0.947760462760925f,
    0.954086363315582f, 0.960004091262817f, 0.965513706207275f, 0.970615267753601f,
    0.975308656692505f, 0.979593932628632f, 0.983471095561981f, 0.986940085887909f,
    0.990001022815704f, 0.992653846740723f, 0.994898498058319f, 0.996735036373138f,
    0.998163461685181f, 0.999183773994446f, 0.999795913696289f, 1.000000000000000f
};

// (S型曲线加减速函数，计数8)
struct __LocalCurveData{
    SguanQ i;
    SguanQ o;

    SguanQ data_num[2];
    SguanQ data_den[2];
};

// (S型曲线加减速函数，计数8)
static void __local_curve_init(__LocalCurve *curve){
    #if VALUE_MODE_MATH>=VALUE_MODE_Q15
    patch_curve_init(curve);
    #else // VALUE_MODE_MATH
    
    #endif // VALUE_MODE_MATH
}

// (S型曲线加减速函数，计数8)
void transfer_curve_loop(__LocalCurve *curve){
    #if VALUE_MODE_MATH>=VALUE_MODE_Q15
    patch_curve_loop(curve);
    #else // VALUE_MODE_MATH
    
    #endif // VALUE_MODE_MATH
}

// (傅里叶变换，计数9)---------------------------------------------------------------
struct __LocalDftData{
    SguanQ i;
    SguanQ o;

    SguanQ data_num[2];
    SguanQ data_den[2];
};

// (傅里叶变换，计数9)
static void __local_dft_init(__LocalDft *dft){
    #if VALUE_MODE_MATH>=VALUE_MODE_Q15
    patch_dft_init(dft);
    #else // VALUE_MODE_MATH
    
    #endif // VALUE_MODE_MATH
}

// (傅里叶变换，计数9)
void transfer_dft_loop(__LocalDft *dft){
    #if VALUE_MODE_MATH>=VALUE_MODE_Q15
    patch_dft_loop(dft);
    #else // VALUE_MODE_MATH
    
    #endif // VALUE_MODE_MATH
}

// (霍尔编码器，计数10)--------------------------------------------------------------
struct __LocalHallData{
    SguanQ num;
    SguanQ den;
};

// (霍尔编码器，计数10)
static void __local_hall_init(__LocalHall *hall){
    #if VALUE_MODE_MATH>=VALUE_MODE_Q15
    patch_hall_init(hall);
    #else // VALUE_MODE_MATH
    // 传递函数运算系数固定计算



    // 初始化为零

    
    #endif // VALUE_MODE_MATH
}

// (霍尔编码器，计数10)
void transfer_hall_loop(__LocalHall *hall){
    #if VALUE_MODE_MATH>=VALUE_MODE_Q15
    patch_hall_loop(hall);
    #else // VALUE_MODE_MATH




    #endif // VALUE_MODE_MATH
}

// (一阶线性自适应抗干扰控制，计数11)-------------------------------------------------
struct __LocalLadrc1Data{
    SguanQ num;
    SguanQ den;
};

// (一阶线性自适应抗干扰控制，计数11)
static void __local_ladrc1_init(__LocalLadrc1 *ladrc){
    #if VALUE_MODE_MATH>=VALUE_MODE_Q15
    patch_ladrc1_init(ladrc);
    #else // VALUE_MODE_MATH
    // 传递函数运算系数固定计算



    // 初始化为零

    
    #endif // VALUE_MODE_MATH
}

// (一阶线性自适应抗干扰控制，计数11)
void transfer_ladrc1_loop(__LocalLadrc1 *ladrc){
    #if VALUE_MODE_MATH>=VALUE_MODE_Q15
    patch_ladrc1_loop(ladrc);
    #else // VALUE_MODE_MATH


    

    #endif // VALUE_MODE_MATH
}

// (二阶线性自适应抗干扰控制，计数12)--------------------------------------------------
struct __LocalLadrc2Data{
    SguanQ num;
    SguanQ den;
};

// (二阶线性自适应抗干扰控制，计数12)
static void __local_ladrc2_init(__LocalLadrc2 *ladrc){
    #if VALUE_MODE_MATH>=VALUE_MODE_Q15
    patch_ladrc2_init(ladrc);
    #else // VALUE_MODE_MATH


    
    #endif // VALUE_MODE_MATH
}

// (二阶线性自适应抗干扰控制，计数12)
void transfer_ladrc2_loop(__LocalLadrc2 *ladrc){
    #if VALUE_MODE_MATH>=VALUE_MODE_Q15
    patch_ladrc2_loop(ladrc);
    #else // VALUE_MODE_MATH


    

    #endif // VALUE_MODE_MATH
}

// (传统指数型趋近率的滑模控制，计数13)------------------------------------------------
struct __LocalSmcData{
    SguanQ num;
    SguanQ den;
};

// (传统指数型趋近率的滑模控制，计数13)
static void __local_smc_init(__LocalSmc *smc){
    #if VALUE_MODE_MATH>=VALUE_MODE_Q15
    patch_smc_init(smc);
    #else // VALUE_MODE_MATH


    
    #endif // VALUE_MODE_MATH
}

// (传统指数型趋近率的滑模控制，计数13)
void transfer_smc_loop(__LocalSmc *smc){
    #if VALUE_MODE_MATH>=VALUE_MODE_Q15
    patch_smc_loop(smc);
    #else // VALUE_MODE_MATH


    

    #endif // VALUE_MODE_MATH
}

// (增量式电流预测控制，计数14)-------------------------------------------------------
struct __LocalDpccData{
    SguanQ num;
    SguanQ den;
};

// (增量式电流预测控制，计数14)
static void __local_dpcc_init(__LocalDpcc *dpcc){
    #if VALUE_MODE_MATH>=VALUE_MODE_Q15
    patch_dpcc_init(dpcc);
    #else // VALUE_MODE_MATH


    
    #endif // VALUE_MODE_MATH
}

// (增量式电流预测控制，计数14)
void transfer_dpcc_loop(__LocalDpcc *dpcc){
    #if VALUE_MODE_MATH>=VALUE_MODE_Q15
    patch_dpcc_loop(dpcc);
    #else // VALUE_MODE_MATH


    

    #endif // VALUE_MODE_MATH
}

// (比例积分谐振调节器，计数15)-------------------------------------------------------
struct __LocalPirData{
    SguanQ num;
    SguanQ den;
};

// (比例积分谐振调节器，计数15)
static void __local_pir_init(__LocalPir *pir){
    #if VALUE_MODE_MATH>=VALUE_MODE_Q15
    patch_pir_init(pir);
    #else // VALUE_MODE_MATH


    
    #endif // VALUE_MODE_MATH
}

// (比例积分谐振调节器，计数15)
void transfer_pir_loop(__LocalPir *pir){
    #if VALUE_MODE_MATH>=VALUE_MODE_Q15
    patch_pir_loop(pir);
    #else // VALUE_MODE_MATH


    

    #endif // VALUE_MODE_MATH
}

// (传统闭环控制器，计数16)-----------------------------------------------------------
struct __LocalPidData{
    SguanQ num;
    SguanQ den;
};

// (传统闭环控制器，计数16)
static void __local_pid_init(__LocalPid *pid){
    #if VALUE_MODE_MATH>=VALUE_MODE_Q15
    patch_pid_init(pid);
    #else // VALUE_MODE_MATH


    
    #endif // VALUE_MODE_MATH
}

// (传统闭环控制器，计数16)
void transfer_pid_loop(__LocalPid *pid){
    #if VALUE_MODE_MATH>=VALUE_MODE_Q15
    patch_pid_loop(pid);
    #else // VALUE_MODE_MATH


    

    #endif // VALUE_MODE_MATH
}

// (开环锁相环，计数17)---------------------------------------------------------------
struct __LocalPllData{
    SguanQ num;
    SguanQ den;
};

// (开环锁相环，计数17)
static void __local_pll_init(__LocalPll *pll){
    #if VALUE_MODE_MATH>=VALUE_MODE_Q15
    patch_pll_init(pll);
    #else // VALUE_MODE_MATH


    
    #endif // VALUE_MODE_MATH
}

// (开环锁相环，计数17)
void transfer_pll_loop(__LocalPll *pll){
    #if VALUE_MODE_MATH>=VALUE_MODE_Q15
    patch_pll_loop(pll);
    #else // VALUE_MODE_MATH


    

    #endif // VALUE_MODE_MATH
}

// (一阶低通滤波器，计数18)-----------------------------------------------------------
struct __LocalLpf1Data{
    SguanQ i;
    SguanQ o;

    SguanQ data_num;
    SguanQ data_den[2];
};

// (一阶低通滤波器，计数18)
static void __local_lpf1_init(__LocalLpf1 *lpf){
    #if VALUE_MODE_MATH>=VALUE_MODE_Q15
    patch_lpf1_init(lpf);
    #else // VALUE_MODE_MATH
    // 传递函数运算系数固定计算
    lpf->data->data_num = lpf->params.t*lpf->params.wc;

    lpf->data->data_den[0] = 2.0f + lpf->params.t*lpf->params.wc;
    lpf->data->data_den[1] = -2.0f + lpf->params.t*lpf->params.wc;

    // 初始化为零
    lpf->data->i = 0.0f;
    lpf->data->o = 0.0f;

    lpf->in.input = 0.0f;
    lpf->out.output = 0.0f;
    #endif // VALUE_MODE_MATH
}

// (一阶低通滤波器，计数18)
void transfer_lpf1_loop(__LocalLpf1 *lpf){
    #if VALUE_MODE_MATH>=VALUE_MODE_Q15
    patch_lpf1_loop(lpf);
    #else // VALUE_MODE_MATH
    // 1.传递函数运算系数动态计算
    if (lpf->params.recalculate_total_flag){
        lpf->data->data_num = lpf->params.t*lpf->params.wc;
    
        lpf->data->data_den[0] = 2.0f + lpf->params.t*lpf->params.wc;
        lpf->data->data_den[1] = -2.0f + lpf->params.t*lpf->params.wc;
    }

    // 2.运算传递函数
    lpf->out.output = (lpf->in.input+lpf->data->i - lpf->data->o*lpf->data->data_den[1])/lpf->data->data_den[0];

    // 3.更新历史数值
    lpf->data->i = lpf->in.input;
    lpf->data->o = lpf->out.output;
    #endif // VALUE_MODE_MATH
}

// (二阶低通滤波器，计数19)-----------------------------------------------------------
struct __LocalLpf2Data{
    SguanQ i[2];
    SguanQ o[2];
    
    // (data_num)0->现在和LLast
    // (data_num)1->仅Last的系数
    // (传递函数分子系数)
    SguanQ data_num[2];
    SguanQ data_den[3];
};

// (二阶低通滤波器，计数19)
static void __local_lpf2_init(__LocalLpf2 *lpf){
    #if VALUE_MODE_MATH>=VALUE_MODE_Q15
    patch_lpf2_init(lpf);
    #else // VALUE_MODE_MATH
    // 传递函数运算系数固定计算
    lpf->data->data_num[0] = lpf->params.t*lpf->params.wc*lpf->params.t*lpf->params.wc;
    lpf->data->data_num[1] = 2.0f*lpf->params.t*lpf->params.wc*lpf->params.t*lpf->params.wc;

    lpf->data->data_den[0] = 4.0f + VALUE_F_2_SQRT2*lpf->params.t*lpf->params.wc + lpf->params.t*lpf->params.wc*lpf->params.t*lpf->params.wc;
    lpf->data->data_den[1] = -8.0f + 2.0f*lpf->params.t*lpf->params.wc*lpf->params.t*lpf->params.wc;
    lpf->data->data_den[2] = 4.0f - VALUE_F_2_SQRT2*lpf->params.t*lpf->params.wc + lpf->params.t*lpf->params.wc*lpf->params.t*lpf->params.wc;

    // 初始化为零
    lpf->data->i[0] = 0.0f;
    lpf->data->i[1] = 0.0f;
    lpf->data->o[0] = 0.0f;
    lpf->data->o[1] = 0.0f;
    
    lpf->in.input = 0.0f;
    lpf->out.output = 0.0f;
    #endif // VALUE_MODE_MATH
}

// (二阶低通滤波器，计数19)
void transfer_lpf2_loop(__LocalLpf2 *lpf){
    #if VALUE_MODE_MATH>=VALUE_MODE_Q15
    patch_lpf2_loop(lpf);
    #else // VALUE_MODE_MATH
    // 1.传递函数运算系数动态计算
    if (lpf->params.recalculate_total_flag){
        lpf->data->data_num[0] = lpf->params.t*lpf->params.wc*lpf->params.t*lpf->params.wc;
        lpf->data->data_num[1] = 2.0f*lpf->params.t*lpf->params.wc*lpf->params.t*lpf->params.wc;

        lpf->data->data_den[0] = 4.0f + VALUE_F_2_SQRT2*lpf->params.t*lpf->params.wc + lpf->params.t*lpf->params.wc*lpf->params.t*lpf->params.wc;
        lpf->data->data_den[1] = -8.0f + 2.0f*lpf->params.t*lpf->params.wc*lpf->params.t*lpf->params.wc;
        lpf->data->data_den[2] = 4.0f - VALUE_F_2_SQRT2*lpf->params.t*lpf->params.wc + lpf->params.t*lpf->params.wc*lpf->params.t*lpf->params.wc;
    }

    // 2.运算传递函数
    lpf->out.output = ((lpf->in.input + lpf->data->i[1])*lpf->data->data_num[0] + lpf->data->i[0]*lpf->data->data_num[1] - lpf->data->o[0]*lpf->data->data_den[1] - lpf->data->o[1]*lpf->data->data_den[2])/lpf->data->data_den[0];

    // 3.更新历史数值
    lpf->data->i[1] = lpf->data->i[0];
    lpf->data->i[0] = lpf->in.input;
    
    lpf->data->o[1] = lpf->data->o[0];
    lpf->data->o[0] = lpf->out.output;
    #endif // VALUE_MODE_MATH
}

// (一阶高通滤波器，计数20)-----------------------------------------------------------
struct __LocalHpf1Data{
    SguanQ i;
    SguanQ o;

    SguanQ data_num[2];
    SguanQ data_den[2];
};

// (一阶高通滤波器，计数20)
static void __local_hpf1_init(__LocalHpf1 *hpf){
    #if VALUE_MODE_MATH>=VALUE_MODE_Q15
    patch_hpf1_init(hpf);
    #else // VALUE_MODE_MATH
    // 传递函数运算系数固定计算
    hpf->data->data_num[0] = 2.0f;
    hpf->data->data_num[1] = -2.0f;

    hpf->data->data_den[0] = 2.0f + hpf->params.t*hpf->params.wc;
    hpf->data->data_den[1] = -2.0f + hpf->params.t*hpf->params.wc;

    // 初始化为零
    hpf->data->i = 0.0f;
    hpf->data->o = 0.0f;

    hpf->in.input = 0.0f;
    hpf->out.output = 0.0f;
    #endif // VALUE_MODE_MATH
}

// (一阶高通滤波器，计数20)
void transfer_hpf1_loop(__LocalHpf1 *hpf){
    #if VALUE_MODE_MATH>=VALUE_MODE_Q15
    patch_hpf1_loop(hpf);
    #else // VALUE_MODE_MATH
    // 1.传递函数运算系数动态计算
    if (hpf->params.recalculate_total_flag){
        hpf->data->data_num[0] = 2.0f;
        hpf->data->data_num[1] = -2.0f;

        hpf->data->data_den[0] = 2.0f + hpf->params.t*hpf->params.wc;
        hpf->data->data_den[1] = -2.0f + hpf->params.t*hpf->params.wc;
    }

    // 2.运算传递函数
    hpf->out.output = (hpf->in.input*hpf->data->data_num[0] + hpf->data->i*hpf->data->data_num[1] - hpf->data->o*hpf->data->data_den[1])/hpf->data->data_den[0];

    // 3.更新历史数值
    hpf->data->i = hpf->in.input;
    hpf->data->o = hpf->out.output;
    #endif // VALUE_MODE_MATH
}

// (二阶高通滤波器，计数21)-----------------------------------------------------------
struct __LocalHpf2Data{
    SguanQ i[2];
    SguanQ o[2];

    // (data_num)0->现在和LLast
    // (data_num)1->仅Last的系数
    // (传递函数分子系数)
    SguanQ data_num[2];
    SguanQ data_den[3];
};

// (二阶高通滤波器，计数21)
static void __local_hpf2_init(__LocalHpf2 *hpf){
    #if VALUE_MODE_MATH>=VALUE_MODE_Q15
    patch_hpf2_init(hpf);
    #else // VALUE_MODE_MATH
    // 传递函数运算系数固定计算
    hpf->data->data_num[0] = 4.0f;
    hpf->data->data_num[1] = -8.0f;

    hpf->data->data_den[0] = 4.0f + VALUE_F_2_SQRT2*hpf->params.t*hpf->params.wc + hpf->params.t*hpf->params.wc*hpf->params.t*hpf->params.wc;
    hpf->data->data_den[1] = -8.0f + 2.0f*hpf->params.t*hpf->params.wc*hpf->params.t*hpf->params.wc;
    hpf->data->data_den[2] = 4.0f - VALUE_F_2_SQRT2*hpf->params.t*hpf->params.wc + hpf->params.t*hpf->params.wc*hpf->params.t*hpf->params.wc;

    // 初始化为零
    hpf->data->i[0] = 0.0f;
    hpf->data->i[1] = 0.0f;

    hpf->data->o[0] = 0.0f;
    hpf->data->o[1] = 0.0f;

    hpf->in.input = 0.0f;
    hpf->out.output = 0.0f;
    #endif // VALUE_MODE_MATH
}

// (二阶高通滤波器，计数21)
void transfer_hpf2_loop(__LocalHpf2 *hpf){
    #if VALUE_MODE_MATH>=VALUE_MODE_Q15
    patch_hpf2_loop(hpf);
    #else // VALUE_MODE_MATH
    // 1.传递函数运算系数动态计算
    if (hpf->params.recalculate_total_flag){
        hpf->data->data_num[0] = 4.0f;
        hpf->data->data_num[1] = -8.0f;

        hpf->data->data_den[0] = 4.0f + VALUE_F_2_SQRT2*hpf->params.t*hpf->params.wc + hpf->params.t*hpf->params.wc*hpf->params.t*hpf->params.wc;
        hpf->data->data_den[1] = -8.0f + 2.0f*hpf->params.t*hpf->params.wc*hpf->params.t*hpf->params.wc;
        hpf->data->data_den[2] = 4.0f - VALUE_F_2_SQRT2*hpf->params.t*hpf->params.wc + hpf->params.t*hpf->params.wc*hpf->params.t*hpf->params.wc;
    }

    // 2.运算传递函数
    hpf->out.output = ((hpf->in.input + hpf->data->i[1])*hpf->data->data_num[0] + hpf->data->i[0]*hpf->data->data_num[1] - hpf->data->o[0]*hpf->data->data_den[1] - hpf->data->o[1]*hpf->data->data_den[2])/hpf->data->data_den[0];

    // 3.更新历史数值
    hpf->data->i[1] = hpf->data->i[0];
    hpf->data->i[0] = hpf->in.input;
    
    hpf->data->o[1] = hpf->data->o[0];
    hpf->data->o[0] = hpf->out.output;
    #endif // VALUE_MODE_MATH
}

// (带通滤波器"一阶低通和高通串联"，计数22)--------------------------------------------
struct __LocalBpf1Data{
    SguanQ i[2];
    SguanQ o[2];

    // (data_num)0->现在的系数
    // (data_num)1->LLast的系数
    // (传递函数分子系数)
    SguanQ data_num[2];
    SguanQ data_den[3];
};

// (带通滤波器"一阶低通和高通串联"，计数22)
static void __local_bpf1_init(__LocalBpf1 *bpf){
    #if VALUE_MODE_MATH>=VALUE_MODE_Q15
    patch_bpf1_init(bpf);
    #else // VALUE_MODE_MATH
    // 传递函数运算系数固定计算
    bpf->data->data_num[0] = 2.0f*bpf->params.t*bpf->params.wc_low;
    bpf->data->data_num[1] = -2.0f*bpf->params.t*bpf->params.wc_low;

    bpf->data->data_den[0] = 4.0f + 2.0f*bpf->params.t*bpf->params.wc_high + 2.0f*bpf->params.t*bpf->params.wc_low + bpf->params.t*bpf->params.t*bpf->params.wc_high*bpf->params.wc_low;
    bpf->data->data_den[1] = -8.0f + 2.0f*bpf->params.t*bpf->params.t*bpf->params.wc_high*bpf->params.wc_low;
    bpf->data->data_den[2] = 4.0f - 2.0f*bpf->params.t*bpf->params.wc_high - 2.0f*bpf->params.t*bpf->params.wc_low + bpf->params.t*bpf->params.t*bpf->params.wc_high*bpf->params.wc_low;

    // 初始化为零
    bpf->data->i[0] = 0.0f;
    bpf->data->i[1] = 0.0f;

    bpf->data->o[0] = 0.0f;
    bpf->data->o[1] = 0.0f;
    
    bpf->in.input = 0.0f;
    bpf->out.output = 0.0f;
    #endif // VALUE_MODE_MATH
}

// (带通滤波器"一阶低通和高通串联"，计数22)
void transfer_bpf1_loop(__LocalBpf1 *bpf){
    #if VALUE_MODE_MATH>=VALUE_MODE_Q15
    patch_bpf1_loop(bpf);
    #else // VALUE_MODE_MATH
    // 1.传递函数运算系数动态计算
    if (bpf->params.recalculate_total_flag){
        bpf->data->data_num[0] = 2.0f*bpf->params.t*bpf->params.wc_low;
        bpf->data->data_num[1] = -2.0f*bpf->params.t*bpf->params.wc_low;

        bpf->data->data_den[0] = 4.0f + 2.0f*bpf->params.t*bpf->params.wc_high + 2.0f*bpf->params.t*bpf->params.wc_low + bpf->params.t*bpf->params.t*bpf->params.wc_high*bpf->params.wc_low;
        bpf->data->data_den[1] = -8.0f + 2.0f*bpf->params.t*bpf->params.t*bpf->params.wc_high*bpf->params.wc_low;
        bpf->data->data_den[2] = 4.0f - 2.0f*bpf->params.t*bpf->params.wc_high - 2.0f*bpf->params.t*bpf->params.wc_low + bpf->params.t*bpf->params.t*bpf->params.wc_high*bpf->params.wc_low;
    }

    // 2.运算传递函数
    bpf->out.output = (bpf->in.input*bpf->data->data_num[0] + bpf->data->i[1]*bpf->data->data_num[1] - bpf->data->o[0]*bpf->data->data_den[1] - bpf->data->o[1]*bpf->data->data_den[2])/bpf->data->data_den[0];

    // 3.更新历史数值
    bpf->data->i[1] = bpf->data->i[0];
    bpf->data->i[0] = bpf->in.input;

    bpf->data->o[1] = bpf->data->o[0];
    bpf->data->o[0] = bpf->out.output;
    #endif // VALUE_MODE_MATH
}

// (带通滤波器"典型二阶系统改型"，计数23)----------------------------------------------
struct __LocalBpf2Data{
    SguanQ i[2];
    SguanQ o[2];

    // (data_num)0->现在的系数
    // (data_num)1->LLast的系数
    // (传递函数分子系数)
    SguanQ data_num[2];
    SguanQ data_den[3];
};

// (带通滤波器"典型二阶系统改型"，计数23)
static void __local_bpf2_init(__LocalBpf2 *bpf){
    #if VALUE_MODE_MATH>=VALUE_MODE_Q15
    patch_bpf2_init(bpf);
    #else // VALUE_MODE_MATH
    // 传递函数运算系数固定计算
    bpf->data->data_num[0] = 4.0f*bpf->params.t*bpf->params.wo*bpf->params.zeta;
    bpf->data->data_num[1] = -4.0f*bpf->params.t*bpf->params.wo*bpf->params.zeta;

    bpf->data->data_den[0] = 4.0f + 4.0f*bpf->params.t*bpf->params.wo*bpf->params.zeta + bpf->params.t*bpf->params.wo*bpf->params.t*bpf->params.wo;
    bpf->data->data_den[1] = -8.0f + 2.0f*bpf->params.t*bpf->params.wo*bpf->params.t*bpf->params.wo;
    bpf->data->data_den[2] = 4.0f - 4.0f*bpf->params.t*bpf->params.wo*bpf->params.zeta + bpf->params.t*bpf->params.wo*bpf->params.t*bpf->params.wo;

    // 初始化为零
    bpf->data->i[0] = 0.0f;
    bpf->data->i[1] = 0.0f;

    bpf->data->o[0] = 0.0f;
    bpf->data->o[1] = 0.0f;
    
    bpf->in.input = 0.0f;
    bpf->out.output = 0.0f;
    #endif // VALUE_MODE_MATH
}

// (带通滤波器"典型二阶系统改型"，计数23)
void transfer_bpf2_loop(__LocalBpf2 *bpf){
    #if VALUE_MODE_MATH>=VALUE_MODE_Q15
    patch_bpf2_loop(bpf);
    #else // VALUE_MODE_MATH
    // 1.传递函数运算系数动态计算
    if (bpf->params.recalculate_total_flag){
        bpf->data->data_num[0] = 4.0f*bpf->params.t*bpf->params.wo*bpf->params.zeta;
        bpf->data->data_num[1] = -4.0f*bpf->params.t*bpf->params.wo*bpf->params.zeta;

        bpf->data->data_den[0] = 4.0f + 4.0f*bpf->params.t*bpf->params.wo*bpf->params.zeta + bpf->params.t*bpf->params.wo*bpf->params.t*bpf->params.wo;
        bpf->data->data_den[1] = -8.0f + 2.0f*bpf->params.t*bpf->params.wo*bpf->params.t*bpf->params.wo;
        bpf->data->data_den[2] = 4.0f - 4.0f*bpf->params.t*bpf->params.wo*bpf->params.zeta + bpf->params.t*bpf->params.wo*bpf->params.t*bpf->params.wo;
    }

    // 2.运算传递函数
    bpf->out.output = (bpf->in.input*bpf->data->data_num[0] + bpf->data->i[1]*bpf->data->data_num[1] - bpf->data->o[0]*bpf->data->data_den[1] - bpf->data->o[1]*bpf->data->data_den[2])/bpf->data->data_den[0];

    // 3.更新历史数值
    bpf->data->i[1] = bpf->data->i[0];
    bpf->data->i[0] = bpf->in.input;

    bpf->data->o[1] = bpf->data->o[0];
    bpf->data->o[0] = bpf->out.output;
    #endif // VALUE_MODE_MATH
}

// (广义积分器"典型二阶系统改型"，计数24)----------------------------------------------
struct __LocalSogiData{
    SguanQ i[2];
    SguanQ o[2];

    // (data_num)0->现在的系数
    // (data_num)1->LLast的系数
    // (传递函数分子系数)
    SguanQ data_num[2];
    SguanQ data_den[3];
};

// (广义积分器"典型二阶系统改型"，计数24)
static void __local_sogi_init(__LocalSogi *sogi){
    #if VALUE_MODE_MATH>=VALUE_MODE_Q15
    patch_sogi_init(sogi);
    #else // VALUE_MODE_MATH
    // 传递函数运算系数固定计算


    // 初始化为零

    #endif // VALUE_MODE_MATH
}

// (广义积分器"典型二阶系统改型"，计数24)
void transfer_sogi_loop(__LocalSogi *sogi){
    #if VALUE_MODE_MATH>=VALUE_MODE_Q15
    patch_sogi_loop(sogi);
    #else // VALUE_MODE_MATH
    // 1.传递函数运算系数动态计算
    if (1){

    }

    // 2.运算传递函数

    #endif // VALUE_MODE_MATH
}

// (陷波滤波器"典型二阶系统改型"，计数25)----------------------------------------------
struct __LocalNfData{
    SguanQ i[2];
    SguanQ o[2];

    // (data_num)0->现在和LLast
    // (data_num)1->仅Last的系数
    // (传递函数分子系数)
    SguanQ data_num[2];
    SguanQ data_den[3];
};

// (陷波滤波器"典型二阶系统改型"，计数25)
static void __local_nf_init(__LocalNf *nf){
    #if VALUE_MODE_MATH>=VALUE_MODE_Q15
    patch_nf_init(nf);
    #else // VALUE_MODE_MATH
    // 传递函数运算系数固定计算
    nf->data->data_num[0] = 4.0f + nf->params.t*nf->params.wo*nf->params.t*nf->params.wo;
    nf->data->data_num[1] = -8.0f + 2.0f*nf->params.t*nf->params.wo*nf->params.t*nf->params.wo;

    nf->data->data_den[0] = 4.0f + 4.0f*nf->params.t*nf->params.wo*nf->params.zeta + nf->params.t*nf->params.wo*nf->params.t*nf->params.wo;
    nf->data->data_den[1] = -8.0f + 2.0f*nf->params.t*nf->params.wo*nf->params.t*nf->params.wo;
    nf->data->data_den[2] = 4.0f - 4.0f*nf->params.t*nf->params.wo*nf->params.zeta + nf->params.t*nf->params.wo*nf->params.t*nf->params.wo;

    // 初始化为零
    nf->data->i[0] = 0.0f;
    nf->data->i[1] = 0.0f;

    nf->data->o[0] = 0.0f;
    nf->data->o[1] = 0.0f;

    nf->in.input = 0.0f;
    nf->out.output = 0.0f;
    #endif // VALUE_MODE_MATH
}

// (陷波滤波器"典型二阶系统改型"，计数25)
void transfer_nf_loop(__LocalNf *nf){
    #if VALUE_MODE_MATH>=VALUE_MODE_Q15
    patch_nf_loop(nf);
    #else // VALUE_MODE_MATH
    // 1.传递函数运算系数动态计算
    if (nf->params.recalculate_total_flag){
        nf->data->data_num[0] = 4.0f + nf->params.t*nf->params.wo*nf->params.t*nf->params.wo;
        nf->data->data_num[1] = -8.0f + 2.0f*nf->params.t*nf->params.wo*nf->params.t*nf->params.wo;

        nf->data->data_den[0] = 4.0f + 4.0f*nf->params.t*nf->params.wo*nf->params.zeta + nf->params.t*nf->params.wo*nf->params.t*nf->params.wo;
        nf->data->data_den[1] = -8.0f + 2.0f*nf->params.t*nf->params.wo*nf->params.t*nf->params.wo;
        nf->data->data_den[2] = 4.0f - 4.0f*nf->params.t*nf->params.wo*nf->params.zeta + nf->params.t*nf->params.wo*nf->params.t*nf->params.wo;
    }

    // 2.运算传递函数
    nf->out.output = ((nf->in.input + nf->data->i[1])*nf->data->data_num[0] + nf->data->i[0]*nf->data->data_num[1] - nf->data->o[0]*nf->data->data_den[1] - nf->data->o[1]*nf->data->data_den[2])/nf->data->data_den[0];

    // 3.更新历史数值
    nf->data->i[1] = nf->data->i[0];
    nf->data->i[0] = nf->in.input;
    
    nf->data->o[1] = nf->data->o[0];
    nf->data->o[0] = nf->out.output;
    #endif // VALUE_MODE_MATH
}

// (陷波滤波器"三参数陷波滤波器"，计数26)----------------------------------------------
struct __LocalTpnfData{
    SguanQ i[2];
    SguanQ o[2];

    SguanQ data_num[3];
    SguanQ data_den[3];
};

// (陷波滤波器"三参数陷波滤波器"，计数26)
static void __local_tpnf_init(__LocalTpnf *tpnf){
    #if VALUE_MODE_MATH>=VALUE_MODE_Q15
    patch_tpnf_init(tpnf);
    #else // VALUE_MODE_MATH
    // 传递函数运算系数固定计算
    tpnf->data->data_num[0] = 4.0f + 4.0f*tpnf->params.k2*tpnf->params.t*tpnf->params.wo + tpnf->params.t*tpnf->params.wo*tpnf->params.t*tpnf->params.wo;
    tpnf->data->data_num[1] = -8.0f + 2.0f*tpnf->params.t*tpnf->params.wo*tpnf->params.t*tpnf->params.wo;
    tpnf->data->data_num[2] = 4.0f - 4.0f*tpnf->params.k2*tpnf->params.t*tpnf->params.wo + tpnf->params.t*tpnf->params.wo*tpnf->params.t*tpnf->params.wo;

    tpnf->data->data_den[0] = 4.0f + 4.0f*tpnf->params.k1*tpnf->params.t*tpnf->params.wo + tpnf->params.t*tpnf->params.wo*tpnf->params.t*tpnf->params.wo;
    tpnf->data->data_den[1] = -8.0f + 2.0f*tpnf->params.t*tpnf->params.wo*tpnf->params.t*tpnf->params.wo;
    tpnf->data->data_den[2] = 4.0f - 4.0f*tpnf->params.k1*tpnf->params.t*tpnf->params.wo + tpnf->params.t*tpnf->params.wo*tpnf->params.t*tpnf->params.wo;

    // 初始化为零
    tpnf->data->i[0] = 0.0f;
    tpnf->data->i[1] = 0.0f;

    tpnf->data->o[0] = 0.0f;
    tpnf->data->o[1] = 0.0f;

    tpnf->in.input = 0.0f;
    tpnf->out.output = 0.0f;
    #endif // VALUE_MODE_MATH
}

// (陷波滤波器"三参数陷波滤波器"，计数26)
void transfer_tpnf_loop(__LocalTpnf *tpnf){
    #if VALUE_MODE_MATH>=VALUE_MODE_Q15
    patch_tpnf_loop(tpnf);
    #else // VALUE_MODE_MATH
    // 1.传递函数运算系数动态计算
    if (tpnf->params.recalculate_total_flag){
        tpnf->data->data_num[0] = 4.0f + 4.0f*tpnf->params.k2*tpnf->params.t*tpnf->params.wo + tpnf->params.t*tpnf->params.wo*tpnf->params.t*tpnf->params.wo;
        tpnf->data->data_num[1] = -8.0f + 2.0f*tpnf->params.t*tpnf->params.wo*tpnf->params.t*tpnf->params.wo;
        tpnf->data->data_num[2] = 4.0f - 4.0f*tpnf->params.k2*tpnf->params.t*tpnf->params.wo + tpnf->params.t*tpnf->params.wo*tpnf->params.t*tpnf->params.wo;

        tpnf->data->data_den[0] = 4.0f + 4.0f*tpnf->params.k1*tpnf->params.t*tpnf->params.wo + tpnf->params.t*tpnf->params.wo*tpnf->params.t*tpnf->params.wo;
        tpnf->data->data_den[1] = -8.0f + 2.0f*tpnf->params.t*tpnf->params.wo*tpnf->params.t*tpnf->params.wo;
        tpnf->data->data_den[2] = 4.0f - 4.0f*tpnf->params.k1*tpnf->params.t*tpnf->params.wo + tpnf->params.t*tpnf->params.wo*tpnf->params.t*tpnf->params.wo;
    }

    // 2.运算传递函数
    tpnf->out.output = (tpnf->in.input*tpnf->data->data_num[0] + tpnf->data->i[0]*tpnf->data->data_num[1] + tpnf->data->i[1]*tpnf->data->data_num[2] - tpnf->data->o[0]*tpnf->data->data_den[1] - tpnf->data->o[1]*tpnf->data->data_den[2])/tpnf->data->data_den[0];

    // 3.更新历史数值
    tpnf->data->i[1] = tpnf->data->i[0];
    tpnf->data->i[0] = tpnf->in.input;
    
    tpnf->data->o[1] = tpnf->data->o[0];
    tpnf->data->o[0] = tpnf->out.output;
    #endif // VALUE_MODE_MATH
}

// (超螺旋滑模扰动观测器，计数27)-----------------------------------------------------
struct __LocalDobData{
    SguanQ num;
    SguanQ den;
};

// (超螺旋滑模扰动观测器，计数27)
static void __local_dob_init(__LocalDob *dob){
    #if VALUE_MODE_MATH>=VALUE_MODE_Q15
    patch_dob_init(dob);
    #else // VALUE_MODE_MATH


    
    #endif // VALUE_MODE_MATH
}

// (超螺旋滑模扰动观测器，计数27)
void transfer_dob_loop(__LocalDob *dob){
    #if VALUE_MODE_MATH>=VALUE_MODE_Q15
    patch_dob_loop(dob);
    #else // VALUE_MODE_MATH


    

    #endif // VALUE_MODE_MATH
}

// (电机参数在线辨识观测器，计数28)----------------------------------------------------
struct __LocalRlsData{
    SguanQ num;
    SguanQ den;
};

// (电机参数在线辨识观测器，计数28)
static void __local_rls_init(__LocalRls *rls){
    #if VALUE_MODE_MATH>=VALUE_MODE_Q15
    patch_rls_init(rls);
    #else // VALUE_MODE_MATH


    
    #endif // VALUE_MODE_MATH
}

// (电机参数在线辨识观测器，计数28)
void transfer_rls_loop(__LocalRls *rls){
    #if VALUE_MODE_MATH>=VALUE_MODE_Q15
    patch_rls_loop(rls);
    #else // VALUE_MODE_MATH


    

    #endif // VALUE_MODE_MATH
}

// ("无感"滑模观测器，计数29)---------------------------------------------------------
struct __LocalSmoData{
    SguanQ num;
    SguanQ den;
};

// ("无感"滑模观测器，计数29)
static void __local_smo_init(__LocalSmo *smo){
    #if VALUE_MODE_MATH>=VALUE_MODE_Q15
    patch_smo_init(smo);
    #else // VALUE_MODE_MATH


    
    #endif // VALUE_MODE_MATH
}

// ("无感"滑模观测器，计数29)
void transfer_smo_loop(__LocalSmo *smo){
    #if VALUE_MODE_MATH>=VALUE_MODE_Q15
    patch_smo_loop(smo);
    #else // VALUE_MODE_MATH


    

    #endif // VALUE_MODE_MATH
}

// ("无感"非线性磁链观测器，计数30)---------------------------------------------------
struct __LocalNfoData{
    SguanQ num;
    SguanQ den;
};

// ("无感"非线性磁链观测器，计数30)
static void __local_nfo_init(__LocalNfo *nfo){
    #if VALUE_MODE_MATH>=VALUE_MODE_Q15
    patch_nfo_init(nfo);
    #else // VALUE_MODE_MATH


    
    #endif // VALUE_MODE_MATH
}

// ("无感"非线性磁链观测器，计数30)
void transfer_nfo_loop(__LocalNfo *nfo){
    #if VALUE_MODE_MATH>=VALUE_MODE_Q15
    patch_nfo_loop(nfo);
    #else // VALUE_MODE_MATH


    

    #endif // VALUE_MODE_MATH
}

// ("无感"有效磁链观测器，计数31)----------------------------------------------
struct __LocalAfoData{
    SguanQ num;
    SguanQ den;
};

// ("无感"有效磁链观测器，计数31)
static void __local_afo_init(__LocalAfo *afo){
    #if VALUE_MODE_MATH>=VALUE_MODE_Q15
    patch_afo_init(afo);
    #else // VALUE_MODE_MATH


    
    #endif // VALUE_MODE_MATH
}

// ("无感"有效磁链观测器，计数31)
void transfer_afo_loop(__LocalAfo *afo){
    #if VALUE_MODE_MATH>=VALUE_MODE_Q15
    patch_afo_loop(afo);
    #else // VALUE_MODE_MATH


    

    #endif // VALUE_MODE_MATH
}

// ("无感"高频正弦波注入，计数32)-----------------------------------------------------
struct __LocalHfiData{
    SguanQ num;
    SguanQ den;
};

// ("无感"高频正弦波注入，计数32)
static void __local_hfi_init(__LocalHfi *hfi){
    #if VALUE_MODE_MATH>=VALUE_MODE_Q15
    patch_hfi_init(hfi);
    #else // VALUE_MODE_MATH


    
    #endif // VALUE_MODE_MATH
}

// ("无感"高频正弦波注入，计数32)
void transfer_hfi_loop(__LocalHfi *hfi){
    #if VALUE_MODE_MATH>=VALUE_MODE_Q15
    patch_hfi_loop(hfi);
    #else // VALUE_MODE_MATH


    

    #endif // VALUE_MODE_MATH
}

// ("无感"降阶龙伯格观测器，计数33)---------------------------------------------------
struct __LocalRoloData{
    SguanQ num;
    SguanQ den;
};

// ("无感"降阶龙伯格观测器，计数33)
static void __local_rolo_init(__LocalRolo *rolo){
    #if VALUE_MODE_MATH>=VALUE_MODE_Q15
    patch_rolo_init(rolo);
    #else // VALUE_MODE_MATH


    
    #endif // VALUE_MODE_MATH
}

// ("无感"降阶龙伯格观测器，计数33)
void transfer_rolo_loop(__LocalRolo *rolo){
    #if VALUE_MODE_MATH>=VALUE_MODE_Q15
    patch_rolo_loop(rolo);
    #else // VALUE_MODE_MATH


    

    #endif // VALUE_MODE_MATH
}

// ("无感"模型参考自适应观测器，计数34)------------------------------------------------
struct __LocalMarsData{
    SguanQ num;
    SguanQ den;
};

// ("无感"模型参考自适应观测器，计数34)
static void __local_mars_init(__LocalMars *mars){
    #if VALUE_MODE_MATH>=VALUE_MODE_Q15
    patch_mars_init(mars);
    #else // VALUE_MODE_MATH


    
    #endif // VALUE_MODE_MATH
}

// ("无感"模型参考自适应观测器，计数34)
void transfer_mars_loop(__LocalMars *mars){
    #if VALUE_MODE_MATH>=VALUE_MODE_Q15
    patch_mars_loop(mars);
    #else // VALUE_MODE_MATH


    

    #endif // VALUE_MODE_MATH
}

// ("无感"扩展卡尔曼滤波，计数35)-----------------------------------------------------
struct __LocalEkfData{
    SguanQ num;
    SguanQ den;
};

// ("无感"扩展卡尔曼滤波，计数35)
static void __local_ekf_init(__LocalEkf *ekf){
    #if VALUE_MODE_MATH>=VALUE_MODE_Q15
    patch_ekf_init(ekf);
    #else // VALUE_MODE_MATH


    
    #endif // VALUE_MODE_MATH
}

// ("无感"扩展卡尔曼滤波，计数35)
void transfer_ekf_loop(__LocalEkf *ekf){
    #if VALUE_MODE_MATH>=VALUE_MODE_Q15
    patch_ekf_loop(ekf);
    #else // VALUE_MODE_MATH


    

    #endif // VALUE_MODE_MATH
}

// (延时函数"延时一拍"，计数36)-------------------------------------------------------
static void __local_delay1_init(__LocalDelay1 *delay){
    #if VALUE_MODE_MATH>=VALUE_MODE_Q15

    #else // VALUE_MODE_MATH
    // 初始化为零
    delay->in.input = 0.0f;
    delay->out.output = 0.0f;
    #endif // VALUE_MODE_MATH
}

// (延时函数"延时一拍"，计数36)
void transfer_delay1_loop(__LocalDelay1 *delay){
    #if VALUE_MODE_MATH>=VALUE_MODE_Q15

    #else // VALUE_MODE_MATH
    // 1.创建局部静态变量
    static SguanQ delay_num;

    // 2.运算结果
    delay->out.output = delay_num;

    // 3.更新历史数值
    delay_num = delay->in.input;
    #endif // VALUE_MODE_MATH
}

// (延时函数"延时二拍"，计数37)-------------------------------------------------------
static void __local_delay2_init(__LocalDelay2 *delay){
    #if VALUE_MODE_MATH>=VALUE_MODE_Q15

    #else // VALUE_MODE_MATH
    // 初始化为零
    delay->in.input = 0.0f;
    delay->out.output = 0.0f;
    #endif // VALUE_MODE_MATH
}

// (延时函数"延时二拍"，计数37)
void transfer_delay2_loop(__LocalDelay2 *delay){
    #if VALUE_MODE_MATH>=VALUE_MODE_Q15

    #else // VALUE_MODE_MATH
    // 1.创建局部静态变量
    static SguanQ delay_num[2];

    // 2.运算结果
    delay->out.output = delay_num[1];

    // 3.更新历史数值
    delay_num[1] = delay_num[0];
    delay_num[0] = delay->in.input;
    #endif // VALUE_MODE_MATH
}

// (延时函数"延时三拍"，计数38)-------------------------------------------------------
static void __local_delay3_init(__LocalDelay3 *delay){
    #if VALUE_MODE_MATH>=VALUE_MODE_Q15

    #else // VALUE_MODE_MATH
    // 初始化为零
    delay->in.input = 0.0f;
    delay->out.output = 0.0f;
    #endif // VALUE_MODE_MATH
}

// (延时函数"延时三拍"，计数38)
void transfer_delay3_loop(__LocalDelay3 *delay){
    #if VALUE_MODE_MATH>=VALUE_MODE_Q15

    #else // VALUE_MODE_MATH
    // 1.创建局部静态变量
    static SguanQ delay_num[3];

    // 2.运算结果
    delay->out.output = delay_num[2];

    // 3.更新历史数值
    delay_num[2] = delay_num[1];
    delay_num[1] = delay_num[0];
    delay_num[0] = delay->in.input;  
    #endif // VALUE_MODE_MATH
}

// (正弦发生器，计数39)--------------------------------------------------------------
void transfer_sine_loop(__LocalSine *sine){
    #if VALUE_MODE_MATH>=VALUE_MODE_Q15

    #else // VALUE_MODE_MATH
    sine->out.output = math_sin(sine->in.input);
    #endif // VALUE_MODE_MATH
}

// (余弦发生器，计数40)--------------------------------------------------------------
void transfer_cosine_loop(__LocalCosine *cosine){
    #if VALUE_MODE_MATH>=VALUE_MODE_Q15

    #else // VALUE_MODE_MATH
    cosine->out.output = math_cos(cosine->in.input);
    #endif // VALUE_MODE_MATH
}

// (正余弦发生器，计数41)------------------------------------------------------------
void transfer_sincos_loop(__LocalSinCos *sincos){
    #if VALUE_MODE_MATH>=VALUE_MODE_Q15

    #else // VALUE_MODE_MATH
    math_sin_cos(sincos->in.input, 
        &sincos->out.sine, 
        &sincos->out.cosine);
    #endif // VALUE_MODE_MATH
}

// (正切求解器，计数42)--------------------------------------------------------------
void transfer_tan_loop(__LocalTan *tan){
    #if VALUE_MODE_MATH>=VALUE_MODE_Q15

    #else // VALUE_MODE_MATH


    

    #endif // VALUE_MODE_MATH
}

// (反正切求解器，计数43)------------------------------------------------------------
void transfer_atan_loop(__LocalAtan *atan){
    #if VALUE_MODE_MATH>=VALUE_MODE_Q15

    #else // VALUE_MODE_MATH


    

    #endif // VALUE_MODE_MATH
}

// (限幅函数，计数44)----------------------------------------------------------------
void transfer_limit_loop(__LocalLimit *limit){
    #if VALUE_MODE_MATH>=VALUE_MODE_Q15

    #else // VALUE_MODE_MATH


    

    #endif // VALUE_MODE_MATH
}

// (符号函数，计数45)----------------------------------------------------------------
void transfer_sign_loop(__LocalSign *sign){
    #if VALUE_MODE_MATH>=VALUE_MODE_Q15

    #else // VALUE_MODE_MATH


    

    #endif // VALUE_MODE_MATH
}

// (克拉克变换，计数46)--------------------------------------------------------------
void transfer_clarke_loop(__LocalClarke *clarke){
    #if VALUE_MODE_MATH>=VALUE_MODE_Q15

    #else // VALUE_MODE_MATH


    

    #endif // VALUE_MODE_MATH
}

// (帕克变换，计数47)----------------------------------------------------------------
void transfer_park_loop(__LocalPark *park){
    #if VALUE_MODE_MATH>=VALUE_MODE_Q15

    #else // VALUE_MODE_MATH


    

    #endif // VALUE_MODE_MATH
}

// (帕克逆变换，计数48)--------------------------------------------------------------
void transfer_ipark_loop(__LocalIpark *ipark){
    #if VALUE_MODE_MATH>=VALUE_MODE_Q15

    #else // VALUE_MODE_MATH


    

    #endif // VALUE_MODE_MATH
}

// (零序注入的SPWM模块，计数49)-------------------------------------------------------
void transfer_spwm_loop(__LocalSpwm *spwm){
    #if VALUE_MODE_MATH>=VALUE_MODE_Q15

    #else // VALUE_MODE_MATH


    

    #endif // VALUE_MODE_MATH
}

// (七段式SVPWM模块，计数50)---------------------------------------------------------
void transfer_svpwm_loop(__LocalSvpwm *svpwm){
    #if VALUE_MODE_MATH>=VALUE_MODE_Q15

    #else // VALUE_MODE_MATH


    

    #endif // VALUE_MODE_MATH
}

// (电调PWM无感方波，计数51)----------------------------------------------------------
void transfer_swpwm_loop(__LocalSwpwm *swpwm){
    #if VALUE_MODE_MATH>=VALUE_MODE_Q15

    #else // VALUE_MODE_MATH
    // TODO: 浮点实现
    #endif // VALUE_MODE_MATH
}

// (单电阻采样函数，计数52)-----------------------------------------------------------
void transfer_singlers_start_loop(__LocalSingleRs *singlers){
    #if VALUE_MODE_MATH>=VALUE_MODE_Q15

    #else // VALUE_MODE_MATH


    

    #endif // VALUE_MODE_MATH
}

void transfer_singlers_end_loop(__LocalSingleRs *singlers){
    #if VALUE_MODE_MATH>=VALUE_MODE_Q15

    #else // VALUE_MODE_MATH


    

    #endif // VALUE_MODE_MATH
}

// ================================================================================
// 4.实例获取静态池(实现)
#if CONFIG_TRANSFER1
static __LocalTransfer1 transfer1_pool[CONFIG_MOTOR][CONFIG_TRANSFER1];
static __LocalTransfer1Data transfer1_data_pool[CONFIG_MOTOR][CONFIG_TRANSFER1];
__LocalTransfer1 *__local_transfer1_get(uint8_t motor, int ch){
    if (motor >= CONFIG_MOTOR || ch >= CONFIG_TRANSFER1) return 0;
    __LocalTransfer1 *self = &transfer1_pool[motor][ch];
    self->data = &transfer1_data_pool[motor][ch];
    return self;
}
#endif
#if CONFIG_TRANSFER2
static __LocalTransfer2 transfer2_pool[CONFIG_MOTOR][CONFIG_TRANSFER2];
static __LocalTransfer2Data transfer2_data_pool[CONFIG_MOTOR][CONFIG_TRANSFER2];
__LocalTransfer2 *__local_transfer2_get(uint8_t motor, int ch){
    if (motor >= CONFIG_MOTOR || ch >= CONFIG_TRANSFER2) return 0;
    __LocalTransfer2 *self = &transfer2_pool[motor][ch];
    self->data = &transfer2_data_pool[motor][ch];
    return self;
}
#endif
#if CONFIG_TRANSFER3
static __LocalTransfer3 transfer3_pool[CONFIG_MOTOR][CONFIG_TRANSFER3];
static __LocalTransfer3Data transfer3_data_pool[CONFIG_MOTOR][CONFIG_TRANSFER3];
__LocalTransfer3 *__local_transfer3_get(uint8_t motor, int ch){
    if (motor >= CONFIG_MOTOR || ch >= CONFIG_TRANSFER3) return 0;
    __LocalTransfer3 *self = &transfer3_pool[motor][ch];
    self->data = &transfer3_data_pool[motor][ch];
    return self;
}
#endif
#if CONFIG_TRANSFER4
static __LocalTransfer4 transfer4_pool[CONFIG_MOTOR][CONFIG_TRANSFER4];
static __LocalTransfer4Data transfer4_data_pool[CONFIG_MOTOR][CONFIG_TRANSFER4];
__LocalTransfer4 *__local_transfer4_get(uint8_t motor, int ch){
    if (motor >= CONFIG_MOTOR || ch >= CONFIG_TRANSFER4) return 0;
    __LocalTransfer4 *self = &transfer4_pool[motor][ch];
    self->data = &transfer4_data_pool[motor][ch];
    return self;
}
#endif
#if CONFIG_TRANSFER5
static __LocalTransfer5 transfer5_pool[CONFIG_MOTOR][CONFIG_TRANSFER5];
static __LocalTransfer5Data transfer5_data_pool[CONFIG_MOTOR][CONFIG_TRANSFER5];
__LocalTransfer5 *__local_transfer5_get(uint8_t motor, int ch){
    if (motor >= CONFIG_MOTOR || ch >= CONFIG_TRANSFER5) return 0;
    __LocalTransfer5 *self = &transfer5_pool[motor][ch];
    self->data = &transfer5_data_pool[motor][ch];
    return self;
}
#endif
#if CONFIG_INTEGRATOR
static __LocalIntegrator integrator_pool[CONFIG_MOTOR][CONFIG_INTEGRATOR];
static __LocalIntegratorData integrator_data_pool[CONFIG_MOTOR][CONFIG_INTEGRATOR];
__LocalIntegrator *__local_integrator_get(uint8_t motor, int ch){
    if (motor >= CONFIG_MOTOR || ch >= CONFIG_INTEGRATOR) return 0;
    __LocalIntegrator *self = &integrator_pool[motor][ch];
    self->data = &integrator_data_pool[motor][ch];
    return self;
}
#endif
#if CONFIG_DERIVATIVE
static __LocalDerivative derivative_pool[CONFIG_MOTOR][CONFIG_DERIVATIVE];
static __LocalDerivativeData derivative_data_pool[CONFIG_MOTOR][CONFIG_DERIVATIVE];
__LocalDerivative *__local_derivative_get(uint8_t motor, int ch){
    if (motor >= CONFIG_MOTOR || ch >= CONFIG_DERIVATIVE) return 0;
    __LocalDerivative *self = &derivative_pool[motor][ch];
    self->data = &derivative_data_pool[motor][ch];
    return self;
}
#endif
#if CONFIG_CURVE
static __LocalCurve curve_pool[CONFIG_MOTOR][CONFIG_CURVE];
static __LocalCurveData curve_data_pool[CONFIG_MOTOR][CONFIG_CURVE];
__LocalCurve *__local_curve_get(uint8_t motor, int ch){
    if (motor >= CONFIG_MOTOR || ch >= CONFIG_CURVE) return 0;
    __LocalCurve *self = &curve_pool[motor][ch];
    self->data = &curve_data_pool[motor][ch];
    return self;
}
#endif
#if CONFIG_DFT
static __LocalDft dft_pool[CONFIG_MOTOR][CONFIG_DFT];
static __LocalDftData dft_data_pool[CONFIG_MOTOR][CONFIG_DFT];
__LocalDft *__local_dft_get(uint8_t motor, int ch){
    if (motor >= CONFIG_MOTOR || ch >= CONFIG_DFT) return 0;
    __LocalDft *self = &dft_pool[motor][ch];
    self->data = &dft_data_pool[motor][ch];
    return self;
}
#endif
#if CONFIG_HALL
static __LocalHall hall_pool[CONFIG_MOTOR][CONFIG_HALL];
static __LocalHallData hall_data_pool[CONFIG_MOTOR][CONFIG_HALL];
__LocalHall *__local_hall_get(uint8_t motor, int ch){
    if (motor >= CONFIG_MOTOR || ch >= CONFIG_HALL) return 0;
    __LocalHall *self = &hall_pool[motor][ch];
    self->data = &hall_data_pool[motor][ch];
    return self;
}
#endif
#if CONFIG_LADRC1
static __LocalLadrc1 ladrc1_pool[CONFIG_MOTOR][CONFIG_LADRC1];
static __LocalLadrc1Data ladrc1_data_pool[CONFIG_MOTOR][CONFIG_LADRC1];
__LocalLadrc1 *__local_ladrc1_get(uint8_t motor, int ch){
    if (motor >= CONFIG_MOTOR || ch >= CONFIG_LADRC1) return 0;
    __LocalLadrc1 *self = &ladrc1_pool[motor][ch];
    self->data = &ladrc1_data_pool[motor][ch];
    return self;
}
#endif
#if CONFIG_LADRC2
static __LocalLadrc2 ladrc2_pool[CONFIG_MOTOR][CONFIG_LADRC2];
static __LocalLadrc2Data ladrc2_data_pool[CONFIG_MOTOR][CONFIG_LADRC2];
__LocalLadrc2 *__local_ladrc2_get(uint8_t motor, int ch){
    if (motor >= CONFIG_MOTOR || ch >= CONFIG_LADRC2) return 0;
    __LocalLadrc2 *self = &ladrc2_pool[motor][ch];
    self->data = &ladrc2_data_pool[motor][ch];
    return self;
}
#endif
#if CONFIG_SMC
static __LocalSmc smc_pool[CONFIG_MOTOR][CONFIG_SMC];
static __LocalSmcData smc_data_pool[CONFIG_MOTOR][CONFIG_SMC];
__LocalSmc *__local_smc_get(uint8_t motor, int ch){
    if (motor >= CONFIG_MOTOR || ch >= CONFIG_SMC) return 0;
    __LocalSmc *self = &smc_pool[motor][ch];
    self->data = &smc_data_pool[motor][ch];
    return self;
}
#endif
#if CONFIG_DPCC
static __LocalDpcc dpcc_pool[CONFIG_MOTOR][CONFIG_DPCC];
static __LocalDpccData dpcc_data_pool[CONFIG_MOTOR][CONFIG_DPCC];
__LocalDpcc *__local_dpcc_get(uint8_t motor, int ch){
    if (motor >= CONFIG_MOTOR || ch >= CONFIG_DPCC) return 0;
    __LocalDpcc *self = &dpcc_pool[motor][ch];
    self->data = &dpcc_data_pool[motor][ch];
    return self;
}
#endif
#if CONFIG_PIR
static __LocalPir pir_pool[CONFIG_MOTOR][CONFIG_PIR];
static __LocalPirData pir_data_pool[CONFIG_MOTOR][CONFIG_PIR];
__LocalPir *__local_pir_get(uint8_t motor, int ch){
    if (motor >= CONFIG_MOTOR || ch >= CONFIG_PIR) return 0;
    __LocalPir *self = &pir_pool[motor][ch];
    self->data = &pir_data_pool[motor][ch];
    return self;
}
#endif
#if CONFIG_PID
static __LocalPid pid_pool[CONFIG_MOTOR][CONFIG_PID];
static __LocalPidData pid_data_pool[CONFIG_MOTOR][CONFIG_PID];
__LocalPid *__local_pid_get(uint8_t motor, int ch){
    if (motor >= CONFIG_MOTOR || ch >= CONFIG_PID) return 0;
    __LocalPid *self = &pid_pool[motor][ch];
    self->data = &pid_data_pool[motor][ch];
    return self;
}
#endif
#if CONFIG_PLL
static __LocalPll pll_pool[CONFIG_MOTOR][CONFIG_PLL];
static __LocalPllData pll_data_pool[CONFIG_MOTOR][CONFIG_PLL];
__LocalPll *__local_pll_get(uint8_t motor, int ch){
    if (motor >= CONFIG_MOTOR || ch >= CONFIG_PLL) return 0;
    __LocalPll *self = &pll_pool[motor][ch];
    self->data = &pll_data_pool[motor][ch];
    return self;
}
#endif
#if CONFIG_LPF1
static __LocalLpf1 lpf1_pool[CONFIG_MOTOR][CONFIG_LPF1];
static __LocalLpf1Data lpf1_data_pool[CONFIG_MOTOR][CONFIG_LPF1];
__LocalLpf1 *__local_lpf1_get(uint8_t motor, int ch){
    if (motor >= CONFIG_MOTOR || ch >= CONFIG_LPF1) return 0;
    __LocalLpf1 *self = &lpf1_pool[motor][ch];
    self->data = &lpf1_data_pool[motor][ch];
    return self;
}
#endif
#if CONFIG_LPF2
static __LocalLpf2 lpf2_pool[CONFIG_MOTOR][CONFIG_LPF2];
static __LocalLpf2Data lpf2_data_pool[CONFIG_MOTOR][CONFIG_LPF2];
__LocalLpf2 *__local_lpf2_get(uint8_t motor, int ch){
    if (motor >= CONFIG_MOTOR || ch >= CONFIG_LPF2) return 0;
    __LocalLpf2 *self = &lpf2_pool[motor][ch];
    self->data = &lpf2_data_pool[motor][ch];
    return self;
}
#endif
#if CONFIG_HPF1
static __LocalHpf1 hpf1_pool[CONFIG_MOTOR][CONFIG_HPF1];
static __LocalHpf1Data hpf1_data_pool[CONFIG_MOTOR][CONFIG_HPF1];
__LocalHpf1 *__local_hpf1_get(uint8_t motor, int ch){
    if (motor >= CONFIG_MOTOR || ch >= CONFIG_HPF1) return 0;
    __LocalHpf1 *self = &hpf1_pool[motor][ch];
    self->data = &hpf1_data_pool[motor][ch];
    return self;
}
#endif
#if CONFIG_HPF2
static __LocalHpf2 hpf2_pool[CONFIG_MOTOR][CONFIG_HPF2];
static __LocalHpf2Data hpf2_data_pool[CONFIG_MOTOR][CONFIG_HPF2];
__LocalHpf2 *__local_hpf2_get(uint8_t motor, int ch){
    if (motor >= CONFIG_MOTOR || ch >= CONFIG_HPF2) return 0;
    __LocalHpf2 *self = &hpf2_pool[motor][ch];
    self->data = &hpf2_data_pool[motor][ch];
    return self;
}
#endif
#if CONFIG_BPF1
static __LocalBpf1 bpf1_pool[CONFIG_MOTOR][CONFIG_BPF1];
static __LocalBpf1Data bpf1_data_pool[CONFIG_MOTOR][CONFIG_BPF1];
__LocalBpf1 *__local_bpf1_get(uint8_t motor, int ch){
    if (motor >= CONFIG_MOTOR || ch >= CONFIG_BPF1) return 0;
    __LocalBpf1 *self = &bpf1_pool[motor][ch];
    self->data = &bpf1_data_pool[motor][ch];
    return self;
}
#endif
#if CONFIG_BPF2
static __LocalBpf2 bpf2_pool[CONFIG_MOTOR][CONFIG_BPF2];
static __LocalBpf2Data bpf2_data_pool[CONFIG_MOTOR][CONFIG_BPF2];
__LocalBpf2 *__local_bpf2_get(uint8_t motor, int ch){
    if (motor >= CONFIG_MOTOR || ch >= CONFIG_BPF2) return 0;
    __LocalBpf2 *self = &bpf2_pool[motor][ch];
    self->data = &bpf2_data_pool[motor][ch];
    return self;
}
#endif
#if CONFIG_SOGI
static __LocalSogi sogi_pool[CONFIG_MOTOR][CONFIG_SOGI];
static __LocalSogiData sogi_data_pool[CONFIG_MOTOR][CONFIG_SOGI];
__LocalSogi *__local_sogi_get(uint8_t motor, int ch){
    if (motor >= CONFIG_MOTOR || ch >= CONFIG_SOGI) return 0;
    __LocalSogi *self = &sogi_pool[motor][ch];
    self->data = &sogi_data_pool[motor][ch];
    return self;
}
#endif
#if CONFIG_NF
static __LocalNf nf_pool[CONFIG_MOTOR][CONFIG_NF];
static __LocalNfData nf_data_pool[CONFIG_MOTOR][CONFIG_NF];
__LocalNf *__local_nf_get(uint8_t motor, int ch){
    if (motor >= CONFIG_MOTOR || ch >= CONFIG_NF) return 0;
    __LocalNf *self = &nf_pool[motor][ch];
    self->data = &nf_data_pool[motor][ch];
    return self;
}
#endif
#if CONFIG_TPNF
static __LocalTpnf tpnf_pool[CONFIG_MOTOR][CONFIG_TPNF];
static __LocalTpnfData tpnf_data_pool[CONFIG_MOTOR][CONFIG_TPNF];
__LocalTpnf *__local_tpnf_get(uint8_t motor, int ch){
    if (motor >= CONFIG_MOTOR || ch >= CONFIG_TPNF) return 0;
    __LocalTpnf *self = &tpnf_pool[motor][ch];
    self->data = &tpnf_data_pool[motor][ch];
    return self;
}
#endif
#if CONFIG_DOB
static __LocalDob dob_pool[CONFIG_MOTOR][CONFIG_DOB];
static __LocalDobData dob_data_pool[CONFIG_MOTOR][CONFIG_DOB];
__LocalDob *__local_dob_get(uint8_t motor, int ch){
    if (motor >= CONFIG_MOTOR || ch >= CONFIG_DOB) return 0;
    __LocalDob *self = &dob_pool[motor][ch];
    self->data = &dob_data_pool[motor][ch];
    return self;
}
#endif
#if CONFIG_RLS
static __LocalRls rls_pool[CONFIG_MOTOR][CONFIG_RLS];
static __LocalRlsData rls_data_pool[CONFIG_MOTOR][CONFIG_RLS];
__LocalRls *__local_rls_get(uint8_t motor, int ch){
    if (motor >= CONFIG_MOTOR || ch >= CONFIG_RLS) return 0;
    __LocalRls *self = &rls_pool[motor][ch];
    self->data = &rls_data_pool[motor][ch];
    return self;
}
#endif
#if CONFIG_SMO
static __LocalSmo smo_pool[CONFIG_MOTOR][CONFIG_SMO];
static __LocalSmoData smo_data_pool[CONFIG_MOTOR][CONFIG_SMO];
__LocalSmo *__local_smo_get(uint8_t motor, int ch){
    if (motor >= CONFIG_MOTOR || ch >= CONFIG_SMO) return 0;
    __LocalSmo *self = &smo_pool[motor][ch];
    self->data = &smo_data_pool[motor][ch];
    return self;
}
#endif
#if CONFIG_NLFO
static __LocalNfo nfo_pool[CONFIG_MOTOR][CONFIG_NLFO];
static __LocalNfoData nfo_data_pool[CONFIG_MOTOR][CONFIG_NLFO];
__LocalNfo *__local_nfo_get(uint8_t motor, int ch){
    if (motor >= CONFIG_MOTOR || ch >= CONFIG_NLFO) return 0;
    __LocalNfo *self = &nfo_pool[motor][ch];
    self->data = &nfo_data_pool[motor][ch];
    return self;
}
#endif
#if CONFIG_VCFO
static __LocalAfo afo_pool[CONFIG_MOTOR][CONFIG_VCFO];
static __LocalAfoData afo_data_pool[CONFIG_MOTOR][CONFIG_VCFO];
__LocalAfo *__local_afo_get(uint8_t motor, int ch){
    if (motor >= CONFIG_MOTOR || ch >= CONFIG_VCFO) return 0;
    __LocalAfo *self = &afo_pool[motor][ch];
    self->data = &afo_data_pool[motor][ch];
    return self;
}
#endif
#if CONFIG_HFI
static __LocalHfi hfi_pool[CONFIG_MOTOR][CONFIG_HFI];
static __LocalHfiData hfi_data_pool[CONFIG_MOTOR][CONFIG_HFI];
__LocalHfi *__local_hfi_get(uint8_t motor, int ch){
    if (motor >= CONFIG_MOTOR || ch >= CONFIG_HFI) return 0;
    __LocalHfi *self = &hfi_pool[motor][ch];
    self->data = &hfi_data_pool[motor][ch];
    return self;
}
#endif
#if CONFIG_ROLO
static __LocalRolo rolo_pool[CONFIG_MOTOR][CONFIG_ROLO];
static __LocalRoloData rolo_data_pool[CONFIG_MOTOR][CONFIG_ROLO];
__LocalRolo *__local_rolo_get(uint8_t motor, int ch){
    if (motor >= CONFIG_MOTOR || ch >= CONFIG_ROLO) return 0;
    __LocalRolo *self = &rolo_pool[motor][ch];
    self->data = &rolo_data_pool[motor][ch];
    return self;
}
#endif
#if CONFIG_MARS
static __LocalMars mars_pool[CONFIG_MOTOR][CONFIG_MARS];
static __LocalMarsData mars_data_pool[CONFIG_MOTOR][CONFIG_MARS];
__LocalMars *__local_mars_get(uint8_t motor, int ch){
    if (motor >= CONFIG_MOTOR || ch >= CONFIG_MARS) return 0;
    __LocalMars *self = &mars_pool[motor][ch];
    self->data = &mars_data_pool[motor][ch];
    return self;
}
#endif
#if CONFIG_EKF
static __LocalEkf ekf_pool[CONFIG_MOTOR][CONFIG_EKF];
static __LocalEkfData ekf_data_pool[CONFIG_MOTOR][CONFIG_EKF];
__LocalEkf *__local_ekf_get(uint8_t motor, int ch){
    if (motor >= CONFIG_MOTOR || ch >= CONFIG_EKF) return 0;
    __LocalEkf *self = &ekf_pool[motor][ch];
    self->data = &ekf_data_pool[motor][ch];
    return self;
}
#endif
#if CONFIG_DELAY1
static __LocalDelay1 delay1_pool[CONFIG_MOTOR][CONFIG_DELAY1];
__LocalDelay1 *__local_delay1_get(uint8_t motor, int ch){
    if (motor >= CONFIG_MOTOR || ch >= CONFIG_DELAY1) return 0;
    return &delay1_pool[motor][ch];
}
#endif
#if CONFIG_DELAY2
static __LocalDelay2 delay2_pool[CONFIG_MOTOR][CONFIG_DELAY2];
__LocalDelay2 *__local_delay2_get(uint8_t motor, int ch){
    if (motor >= CONFIG_MOTOR || ch >= CONFIG_DELAY2) return 0;
    return &delay2_pool[motor][ch];
}
#endif
#if CONFIG_DELAY3
static __LocalDelay3 delay3_pool[CONFIG_MOTOR][CONFIG_DELAY3];
__LocalDelay3 *__local_delay3_get(uint8_t motor, int ch){
    if (motor >= CONFIG_MOTOR || ch >= CONFIG_DELAY3) return 0;
    return &delay3_pool[motor][ch];
}
#endif


static __LocalSine sine_pool[CONFIG_MOTOR];
__LocalSine *__local_sine_get(uint8_t motor){
    if (motor >= CONFIG_MOTOR) return 0;
    return &sine_pool[motor];
}
static __LocalCosine cosine_pool[CONFIG_MOTOR];
__LocalCosine *__local_cosine_get(uint8_t motor){
    if (motor >= CONFIG_MOTOR) return 0;
    return &cosine_pool[motor];
}
static __LocalSinCos sincos_pool[CONFIG_MOTOR];
__LocalSinCos *__local_sincos_get(uint8_t motor){
    if (motor >= CONFIG_MOTOR) return 0;
    return &sincos_pool[motor];
}
static __LocalTan tan_pool[CONFIG_MOTOR];
__LocalTan *__local_tan_get(uint8_t motor){
    if (motor >= CONFIG_MOTOR) return 0;
    return &tan_pool[motor];
}
static __LocalAtan atan_pool[CONFIG_MOTOR];
__LocalAtan *__local_atan_get(uint8_t motor){
    if (motor >= CONFIG_MOTOR) return 0;
    return &atan_pool[motor];
}
static __LocalLimit limit_pool[CONFIG_MOTOR];
__LocalLimit *__local_limit_get(uint8_t motor){
    if (motor >= CONFIG_MOTOR) return 0;
    return &limit_pool[motor];
}
static __LocalSign sign_pool[CONFIG_MOTOR];
__LocalSign *__local_sign_get(uint8_t motor){
    if (motor >= CONFIG_MOTOR) return 0;
    return &sign_pool[motor];
}
static __LocalClarke clarke_pool[CONFIG_MOTOR];
__LocalClarke *__local_clarke_get(uint8_t motor){
    if (motor >= CONFIG_MOTOR) return 0;
    return &clarke_pool[motor];
}
static __LocalPark park_pool[CONFIG_MOTOR];
__LocalPark *__local_park_get(uint8_t motor){
    if (motor >= CONFIG_MOTOR) return 0;
    return &park_pool[motor];
}
static __LocalIpark ipark_pool[CONFIG_MOTOR];
__LocalIpark *__local_ipark_get(uint8_t motor){
    if (motor >= CONFIG_MOTOR) return 0;
    return &ipark_pool[motor];
}
static __LocalSpwm spwm_pool[CONFIG_MOTOR];
__LocalSpwm *__local_spwm_get(uint8_t motor){
    if (motor >= CONFIG_MOTOR) return 0;
    return &spwm_pool[motor];
}
static __LocalSvpwm svpwm_pool[CONFIG_MOTOR];
__LocalSvpwm *__local_svpwm_get(uint8_t motor){
    if (motor >= CONFIG_MOTOR) return 0;
    return &svpwm_pool[motor];
}
static __LocalSwpwm swpwm_pool[CONFIG_MOTOR];
__LocalSwpwm *__local_swpwm_get(uint8_t motor){
    if (motor >= CONFIG_MOTOR) return 0;
    return &swpwm_pool[motor];
}
static __LocalSingleRs singlers_pool[CONFIG_MOTOR];
__LocalSingleRs *__local_singlers_get(uint8_t motor){
    if (motor >= CONFIG_MOTOR) return 0;
    return &singlers_pool[motor];
}

// ================================================================================
// 3.全局函数transfer初始化函数
void transfer_init(Transfer *transfer){

}


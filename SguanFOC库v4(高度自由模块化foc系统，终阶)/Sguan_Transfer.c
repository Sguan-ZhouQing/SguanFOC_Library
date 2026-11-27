/* ===================================================================
 * 文件:     Sguan_Transfer.c
 * 项目:     SguanFOC_v4 —— 开源 FOC 电机控制框架
 * 说明:     1.Sguan固件库_传函模块
 * 链接:     https://github.com/Sguan-ZhouQing/SguanFOC_Library
 * 版权:     Copyright 2026 星必尘Sguan <3464647102@qq.com>
 * 许可证:   Apache License, Version 2.0
 * SPDX:     SPDX-License-Identifier: Apache-2.0
 *
 * 许可说明:
 *   [EN] Licensed under the Apache License, Version 2.0. You may use,
 *        copy, modify, and distribute this software, including for
 *        commercial purposes, provided you retain this copyright
 *        notice and the terms of the License.
 *   [中文] 本软件依据 Apache License 2.0 授权：可自由使用、复制、修改、
 *        分发（含商业用途），但必须保留本版权声明与许可证条款。
 *   [DE] Lizenziert unter der Apache License, Version 2.0. Nutzung,
 *        Vervielfaeltigung, Aenderung und Weitergabe (auch kommerziell)
 *        sind erlaubt, sofern dieser Urheberrechtshinweis und die
 *        Lizenzbedingungen erhalten bleiben.
 *   [JA] Apache License, Version 2.0 に基づき許諾されます。使用・複製・
 *        改変・再配布（商用を含む）が可能ですが、本著作権表示と
 *        ライセンス条項を保持する必要があります。
 * 完整许可文本: 参见仓库根目录下的 LICENSE-APACHE 文件。
 * =================================================================== */
#include "Sguan_Transfer.h"

/* SguanFOC配置文件声明 */
#include "UnitLib_Patch.h"

// ================================================================================
// 1.局部量__local声明
// ()
static void __local_transfer1_initial(__LocalTransfer1 *transfer);
static void __local_transfer2_initial(__LocalTransfer2 *transfer);
static void __local_transfer3_initial(__LocalTransfer3 *transfer);
static void __local_transfer4_initial(__LocalTransfer4 *transfer);
static void __local_transfer5_initial(__LocalTransfer5 *transfer);
// __local_integrator_initial...........此函数无需定义
static void __local_derivative_initial(__LocalDerivative *derivative);
// __local_hall_initial.................此函数无需定义
static void __local_ladrc1_initial(__LocalLadrc1 *ladrc);
static void __local_ladrc2_initial(__LocalLadrc2 *ladrc);
static void __local_smc_initial(__LocalSmc *smc);
static void __local_dpcc_initial(__LocalDpcc *dpcc);
static void __local_pir_initial(__LocalPir *pir);
static void __local_pid_initial(__LocalPid *pid);
static void __local_pll_initial(__LocalPll *pll);
static void __local_lpf1_initial(__LocalLpf1 *lpf);
static void __local_lpf2_initial(__LocalLpf2 *lpf);
static void __local_hpf1_initial(__LocalHpf1 *hpf);
static void __local_hpf2_initial(__LocalHpf2 *hpf);
static void __local_bpf1_initial(__LocalBpf1 *bpf);
static void __local_bpf2_initial(__LocalBpf2 *bpf);
static void __local_sogi_initial(__LocalSogi *sogi);
static void __local_nf_initial(__LocalNf *nf);
static void __local_tpnf_initial(__LocalTpnf *tpnf);
static void __local_dob_initial(__LocalDob *dob);
static void __local_rls_initial(__LocalRls *rls);
static void __local_smo_initial(__LocalSmo *smo);
static void __local_nfo_initial(__LocalNfo *nfo);
static void __local_afo_initial(__LocalAfo *afo);
static void __local_hfi_initial(__LocalHfi *hfi);
static void __local_rolo_initial(__LocalRolo *rolo);
static void __local_mars_initial(__LocalMars *mars);
static void __local_ekf_initial(__LocalEkf *ekf);

// ()
static void __local_transfer1_init(__LocalTransfer1 *transfer);
static void __local_transfer2_init(__LocalTransfer2 *transfer);
static void __local_transfer3_init(__LocalTransfer3 *transfer);
static void __local_transfer4_init(__LocalTransfer4 *transfer);
static void __local_transfer5_init(__LocalTransfer5 *transfer);
static void __local_integrator_init(__LocalIntegrator *integrator);
static void __local_derivative_init(__LocalDerivative *derivative);
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
// (分割线)-------------------------------------------------------------------------
static void __local_delay1_init(__LocalDelay1 *delay);
static void __local_delay2_init(__LocalDelay2 *delay);
static void __local_delay3_init(__LocalDelay3 *delay);

// ()
#define __LOCAL_ZERO_ALL(obj) \
    do{ \
        math_memset(&(obj)->in,  0, sizeof((obj)->in)); \
        math_memset(&(obj)->out, 0, sizeof((obj)->out)); \
        math_memset((obj)->data, 0, sizeof(*(obj)->data)); \
    }while(0)

// ================================================================================
// 2.实例获取静态池(声明)
// ()
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
#if CONFIG_NFO
__LocalNfo *__local_nfo_get(uint8_t motor, int ch);
#endif
#if CONFIG_AFO
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
// (分割线)-------------------------------------------------------------------------
#if CONFIG_DELAY1
__LocalDelay1 *__local_delay1_get(uint8_t motor, int ch);
#endif
#if CONFIG_DELAY2
__LocalDelay2 *__local_delay2_get(uint8_t motor, int ch);
#endif
#if CONFIG_DELAY3
__LocalDelay3 *__local_delay3_get(uint8_t motor, int ch);
#endif

// ()
__LocalSine *__local_sine_get(uint8_t motor);
__LocalCosine *__local_cosine_get(uint8_t motor);
__LocalSinCos *__local_sincos_get(uint8_t motor);
__LocalTan *__local_tan_get(uint8_t motor);
__LocalAtan *__local_atan_get(uint8_t motor);

// ================================================================================
// 3.全局函数transfer高级模块
// (典型一阶传递函数，计数1)--------------------------|------------------------------
struct __LocalTransfer1Data{
    SguanQ i;                                       // (状态)历史输入
    SguanQ o;                                       // (状态)历史输出

    SguanQ data_num[2];                             // (系数)分子系数表
    SguanQ data_den[2];                             // (系数)分母系数表
};

// (典型一阶传递函数，计数1)
static void __local_transfer1_initial(__LocalTransfer1 *transfer){
    transfer->data->data_num[0] = 
        VALUE_F_TWO*transfer->params.num1 + 
        transfer->params.num0*transfer->params.t;
    transfer->data->data_num[1] = 
        -VALUE_F_TWO*transfer->params.num1 + 
        transfer->params.num0*transfer->params.t;

    transfer->data->data_den[0] = 
        VALUE_F_TWO*transfer->params.den1 + 
        transfer->params.den0*transfer->params.t;
    transfer->data->data_den[1] = 
        -VALUE_F_TWO*transfer->params.den1 + 
        transfer->params.den0*transfer->params.t;
}

// (典型一阶传递函数，计数1)
static void __local_transfer1_init(__LocalTransfer1 *transfer){
    #if VALUE_MODE_MATH>=VALUE_MODE_Q15
    patch_transfer1_init(transfer);
    #else // VALUE_MODE_MATH
    // 初始化为零
    __LOCAL_ZERO_ALL(transfer);

    // 传递函数运算系数固定计算
    __local_transfer1_initial(transfer);
    #endif // VALUE_MODE_MATH
}

// (典型一阶传递函数，计数1)
void transfer_transfer1_loop(__LocalTransfer1 *transfer){
    #if VALUE_MODE_MATH>=VALUE_MODE_Q15
    patch_transfer1_loop(transfer);
    #else // VALUE_MODE_MATH
    // 1.传递函数运算系数动态计算
    if (transfer->params.recalculate_total_flag){
        __local_transfer1_initial(transfer);
    }

    // 2.运算传递函数
    transfer->out.output = 
        (transfer->in.input*transfer->data->data_num[0] + 
        transfer->data->i*transfer->data->data_num[1] - 
        transfer->data->o*transfer->data->data_den[1])/transfer->data->data_den[0];

    // 3.更新历史数值
    transfer->data->i = transfer->in.input;
    transfer->data->o = transfer->out.output;
    #endif // VALUE_MODE_MATH
}

// (典型二阶传递函数，计数2)--------------------------|------------------------------
struct __LocalTransfer2Data{
    SguanQ t_pow2;                                  // (系数)t的平方

    SguanQ i[2];                                    // (状态)输入历史
    SguanQ o[2];                                    // (状态)输出历史

    SguanQ data_num[3];                             // (系数)分子系数表
    SguanQ data_den[3];                             // (系数)分母系数表
};

// (典型二阶传递函数，计数2)
static void __local_transfer2_initial(__LocalTransfer2 *transfer){
    transfer->data->data_num[0] = 
        VALUE_F_FOUR*transfer->params.num2 + 
        VALUE_F_TWO*transfer->params.num1*transfer->params.t + 
        transfer->params.num0*transfer->data->t_pow2;
    transfer->data->data_num[1] = 
        -VALUE_F_EIGHT*transfer->params.num2 + 
        VALUE_F_TWO*transfer->params.num0*transfer->data->t_pow2;
    transfer->data->data_num[2] = 
        VALUE_F_FOUR*transfer->params.num2 - 
        VALUE_F_TWO*transfer->params.num1*transfer->params.t + 
        transfer->params.num0*transfer->data->t_pow2;
    
    transfer->data->data_den[0] = 
        VALUE_F_FOUR*transfer->params.den2 + 
        VALUE_F_TWO*transfer->params.den1*transfer->params.t + 
        transfer->params.den0*transfer->data->t_pow2;
    transfer->data->data_den[1] = 
        -VALUE_F_EIGHT*transfer->params.den2 + 
        VALUE_F_TWO*transfer->params.den0*transfer->data->t_pow2;
    transfer->data->data_den[2] = 
        VALUE_F_FOUR*transfer->params.den2 - 
        VALUE_F_TWO*transfer->params.den1*transfer->params.t + 
        transfer->params.den0*transfer->data->t_pow2;
}

// (典型二阶传递函数，计数2)
static void __local_transfer2_init(__LocalTransfer2 *transfer){
    #if VALUE_MODE_MATH>=VALUE_MODE_Q15
    patch_transfer2_init(transfer);
    #else // VALUE_MODE_MATH
    // 初始化为零
    __LOCAL_ZERO_ALL(transfer);

    // 传递函数运算系数固定计算
    transfer->data->t_pow2 = math_pow(transfer->params.t, 2);
    __local_transfer2_initial(transfer);
    #endif // VALUE_MODE_MATH
}

// (典型二阶传递函数，计数2)
void transfer_transfer2_loop(__LocalTransfer2 *transfer){
    #if VALUE_MODE_MATH>=VALUE_MODE_Q15
    patch_transfer2_loop(transfer);
    #else // VALUE_MODE_MATH
    // 1.传递函数运算系数动态计算
    if (transfer->params.recalculate_total_flag){
        __local_transfer2_initial(transfer);
    }

    // 2.运算传递函数
    transfer->out.output = 
        (transfer->in.input*transfer->data->data_num[0] + 
        transfer->data->i[0]*transfer->data->data_num[1] + 
        transfer->data->i[1]*transfer->data->data_num[2] - 
        transfer->data->o[0]*transfer->data->data_den[1] - 
        transfer->data->o[1]*transfer->data->data_den[2])/transfer->data->data_den[0];

    // 3.更新历史数值
    transfer->data->i[1] = transfer->data->i[0];
    transfer->data->i[0] = transfer->in.input;

    transfer->data->o[1] = transfer->data->o[0];
    transfer->data->o[0] = transfer->out.output;
    #endif // VALUE_MODE_MATH
}

// (典型三阶传递函数，计数3)--------------------------|------------------------------
struct __LocalTransfer3Data{
    SguanQ t_pow2;                                  // (系数)t的平方
    SguanQ t_pow3;                                  // (系数)t的三次方

    SguanQ i[3];                                    // (状态)输入历史
    SguanQ o[3];                                    // (状态)输出历史

    SguanQ data_num[4];                             // (系数)分子系数表
    SguanQ data_den[4];                             // (系数)分母系数表
};

// (典型三阶传递函数，计数3)
static void __local_transfer3_initial(__LocalTransfer3 *transfer){
    transfer->data->data_num[0] = 
        VALUE_F_EIGHT*transfer->params.num3 + 
        VALUE_F_FOUR*transfer->params.num2*transfer->params.t + 
        VALUE_F_TWO*transfer->params.num1*transfer->data->t_pow2 + 
        transfer->params.num0*transfer->data->t_pow3;
    transfer->data->data_num[1] = 
        -VALUE_F_TWENTY_FOUR*transfer->params.num3 - 
        VALUE_F_FOUR*transfer->params.num2*transfer->params.t + 
        VALUE_F_TWO*transfer->params.num1*transfer->data->t_pow2 + 
        VALUE_F_THREE*transfer->params.num0*transfer->data->t_pow3;
    transfer->data->data_num[2] = 
        VALUE_F_TWENTY_FOUR*transfer->params.num3 - 
        VALUE_F_FOUR*transfer->params.num2*transfer->params.t - 
        VALUE_F_TWO*transfer->params.num1*transfer->data->t_pow2 + 
        VALUE_F_THREE*transfer->params.num0*transfer->data->t_pow3;
    transfer->data->data_num[3] = 
        -VALUE_F_EIGHT*transfer->params.num3 + 
        VALUE_F_FOUR*transfer->params.num2*transfer->params.t - 
        VALUE_F_TWO*transfer->params.num1*transfer->data->t_pow2 + 
        transfer->params.num0*transfer->data->t_pow3;
    
    transfer->data->data_den[0] = 
        VALUE_F_EIGHT*transfer->params.den3 + 
        VALUE_F_FOUR*transfer->params.den2*transfer->params.t + 
        VALUE_F_TWO*transfer->params.den1*transfer->data->t_pow2 + 
        transfer->params.den0*transfer->data->t_pow3;
    transfer->data->data_den[1] = 
        -VALUE_F_TWENTY_FOUR*transfer->params.den3 - 
        VALUE_F_FOUR*transfer->params.den2*transfer->params.t + 
        VALUE_F_TWO*transfer->params.den1*transfer->data->t_pow2 + 
        VALUE_F_THREE*transfer->params.den0*transfer->data->t_pow3;
    transfer->data->data_den[2] = 
        VALUE_F_TWENTY_FOUR*transfer->params.den3 - 
        VALUE_F_FOUR*transfer->params.den2*transfer->params.t - 
        VALUE_F_TWO*transfer->params.den1*transfer->data->t_pow2 + 
        VALUE_F_THREE*transfer->params.den0*transfer->data->t_pow3;
    transfer->data->data_den[3] = 
        -VALUE_F_EIGHT*transfer->params.den3 + 
        VALUE_F_FOUR*transfer->params.den2*transfer->params.t - 
        VALUE_F_TWO*transfer->params.den1*transfer->data->t_pow2 + 
        transfer->params.den0*transfer->data->t_pow3;
}

// (典型三阶传递函数，计数3)
static void __local_transfer3_init(__LocalTransfer3 *transfer){
    #if VALUE_MODE_MATH>=VALUE_MODE_Q15
    patch_transfer3_init(transfer);
    #else // VALUE_MODE_MATH
    // 初始化为零
    __LOCAL_ZERO_ALL(transfer);

    // 传递函数运算系数固定计算
    transfer->data->t_pow2 = math_pow(transfer->params.t, 2);
    transfer->data->t_pow3 = math_pow(transfer->params.t, 3);
    __local_transfer3_initial(transfer);
    #endif // VALUE_MODE_MATH
}

// (典型三阶传递函数，计数3)
void transfer_transfer3_loop(__LocalTransfer3 *transfer){
    #if VALUE_MODE_MATH>=VALUE_MODE_Q15
    patch_transfer3_loop(transfer);
    #else // VALUE_MODE_MATH
    // 1.传递函数运算系数动态计算
    if (transfer->params.recalculate_total_flag){
        __local_transfer3_initial(transfer);
    }

    // 2.运算传递函数
    transfer->out.output = 
        (transfer->in.input*transfer->data->data_num[0] + 
        transfer->data->i[0]*transfer->data->data_num[1] + 
        transfer->data->i[1]*transfer->data->data_num[2] + 
        transfer->data->i[2]*transfer->data->data_num[3] - 
        transfer->data->o[0]*transfer->data->data_den[1] - 
        transfer->data->o[1]*transfer->data->data_den[2] - 
        transfer->data->o[2]*transfer->data->data_den[3])/transfer->data->data_den[0];

    // 3.更新历史数值
    transfer->data->i[2] = transfer->data->i[1];
    transfer->data->i[1] = transfer->data->i[0];
    transfer->data->i[0] = transfer->in.input;

    transfer->data->o[2] = transfer->data->o[1];
    transfer->data->o[1] = transfer->data->o[0];
    transfer->data->o[0] = transfer->out.output;
    #endif // VALUE_MODE_MATH
}

// (典型四阶传递函数，计数4)--------------------------|------------------------------
struct __LocalTransfer4Data{
    SguanQ t_pow2;                                  // (系数)t的平方
    SguanQ t_pow3;                                  // (系数)t的三次方
    SguanQ t_pow4;                                  // (系数)t的四次方

    SguanQ i[4];                                    // (状态)输入历史
    SguanQ o[4];                                    // (状态)输出历史

    SguanQ data_num[5];                             // (系数)分子系数表
    SguanQ data_den[5];                             // (系数)分母系数表
};

// (典型四阶传递函数，计数4)
static void __local_transfer4_initial(__LocalTransfer4 *transfer){
    transfer->data->data_num[0] = 
        VALUE_F_SIXTEEN*transfer->params.num4 + 
        VALUE_F_EIGHT*transfer->params.num3*transfer->params.t + 
        VALUE_F_FOUR*transfer->params.num2*transfer->data->t_pow2 + 
        VALUE_F_TWO*transfer->params.num1*transfer->data->t_pow3 + 
        transfer->params.num0*transfer->data->t_pow4;
    transfer->data->data_num[1] = 
        -VALUE_F_SIXTY_FOUR*transfer->params.num4 - 
        VALUE_F_SIXTEEN*transfer->params.num3*transfer->params.t + 
        VALUE_F_FOUR*transfer->params.num1*transfer->data->t_pow3 + 
        VALUE_F_FOUR*transfer->params.num0*transfer->data->t_pow4;
    transfer->data->data_num[2] = 
        VALUE_F_NINETY_SIX*transfer->params.num4 - 
        VALUE_F_EIGHT*transfer->params.num2*transfer->data->t_pow2 + 
        VALUE_F_SIX*transfer->params.num0*transfer->data->t_pow4;
    transfer->data->data_num[3] = 
        -VALUE_F_SIXTY_FOUR*transfer->params.num4 + 
        VALUE_F_SIXTEEN*transfer->params.num3*transfer->params.t - 
        VALUE_F_FOUR*transfer->params.num1*transfer->data->t_pow3 + 
        VALUE_F_FOUR*transfer->params.num0*transfer->data->t_pow4;
    transfer->data->data_num[4] = 
        VALUE_F_SIXTEEN*transfer->params.num4 - 
        VALUE_F_EIGHT*transfer->params.num3*transfer->params.t + 
        VALUE_F_FOUR*transfer->params.num2*transfer->data->t_pow2 - 
        VALUE_F_TWO*transfer->params.num1*transfer->data->t_pow3 + 
        transfer->params.num0*transfer->data->t_pow4;
    
    transfer->data->data_den[0] = 
        VALUE_F_SIXTEEN*transfer->params.den4 + 
        VALUE_F_EIGHT*transfer->params.den3*transfer->params.t + 
        VALUE_F_FOUR*transfer->params.den2*transfer->data->t_pow2 + 
        VALUE_F_TWO*transfer->params.den1*transfer->data->t_pow3 + 
        transfer->params.den0*transfer->data->t_pow4;
    transfer->data->data_den[1] = 
        -VALUE_F_SIXTY_FOUR*transfer->params.den4 - 
        VALUE_F_SIXTEEN*transfer->params.den3*transfer->params.t + 
        VALUE_F_FOUR*transfer->params.den1*transfer->data->t_pow3 + 
        VALUE_F_FOUR*transfer->params.den0*transfer->data->t_pow4;
    transfer->data->data_den[2] = 
        VALUE_F_NINETY_SIX*transfer->params.den4 - 
        VALUE_F_EIGHT*transfer->params.den2*transfer->data->t_pow2 + 
        VALUE_F_SIX*transfer->params.den0*transfer->data->t_pow4;
    transfer->data->data_den[3] = 
        -VALUE_F_SIXTY_FOUR*transfer->params.den4 + 
        VALUE_F_SIXTEEN*transfer->params.den3*transfer->params.t - 
        VALUE_F_FOUR*transfer->params.den1*transfer->data->t_pow3 + 
        VALUE_F_FOUR*transfer->params.den0*transfer->data->t_pow4;
    transfer->data->data_den[4] = 
        VALUE_F_SIXTEEN*transfer->params.den4 - 
        VALUE_F_EIGHT*transfer->params.den3*transfer->params.t + 
        VALUE_F_FOUR*transfer->params.den2*transfer->data->t_pow2 - 
        VALUE_F_TWO*transfer->params.den1*transfer->data->t_pow3 + 
        transfer->params.den0*transfer->data->t_pow4;
}

// (典型四阶传递函数，计数4)
static void __local_transfer4_init(__LocalTransfer4 *transfer){
    #if VALUE_MODE_MATH>=VALUE_MODE_Q15
    patch_transfer4_init(transfer);
    #else // VALUE_MODE_MATH
    // 初始化为零
    __LOCAL_ZERO_ALL(transfer);

    // 传递函数运算系数固定计算
    transfer->data->t_pow2 = math_pow(transfer->params.t, 2);
    transfer->data->t_pow3 = math_pow(transfer->params.t, 3);
    transfer->data->t_pow4 = math_pow(transfer->params.t, 4);
    __local_transfer4_initial(transfer);
    #endif // VALUE_MODE_MATH
}

// (典型四阶传递函数，计数4)
void transfer_transfer4_loop(__LocalTransfer4 *transfer){
    #if VALUE_MODE_MATH>=VALUE_MODE_Q15
    patch_transfer4_loop(transfer);
    #else // VALUE_MODE_MATH
    // 1.传递函数运算系数动态计算
    if (transfer->params.recalculate_total_flag){
        __local_transfer4_initial(transfer);
    }
    
    // 2.运算传递函数
    transfer->out.output = 
        (transfer->in.input*transfer->data->data_num[0] + 
        transfer->data->i[0]*transfer->data->data_num[1] + 
        transfer->data->i[1]*transfer->data->data_num[2] + 
        transfer->data->i[2]*transfer->data->data_num[3] + 
        transfer->data->i[3]*transfer->data->data_num[4] - 
        transfer->data->o[0]*transfer->data->data_den[1] - 
        transfer->data->o[1]*transfer->data->data_den[2] - 
        transfer->data->o[2]*transfer->data->data_den[3] - 
        transfer->data->o[3]*transfer->data->data_den[4])/transfer->data->data_den[0];

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

// (典型五阶传递函数，计数5)--------------------------|------------------------------
struct __LocalTransfer5Data{
    SguanQ t_pow2;                                  // (系数)t的平方
    SguanQ t_pow3;                                  // (系数)t的三次方
    SguanQ t_pow4;                                  // (系数)t的四次方
    SguanQ t_pow5;                                  // (系数)t的五次方

    SguanQ i[5];                                    // (状态)输入历史
    SguanQ o[5];                                    // (状态)输出历史

    SguanQ data_num[6];                             // (系数)分子系数表
    SguanQ data_den[6];                             // (系数)分母系数表
};

// (典型五阶传递函数，计数5)
static void __local_transfer5_initial(__LocalTransfer5 *transfer){
    transfer->data->data_num[0] = 
        VALUE_F_THIRTY_TWO*transfer->params.num5 + 
        VALUE_F_SIXTEEN*transfer->params.num4*transfer->params.t + 
        VALUE_F_EIGHT*transfer->params.num3*transfer->data->t_pow2 + 
        VALUE_F_FOUR*transfer->params.num2*transfer->data->t_pow3 + 
        VALUE_F_TWO*transfer->params.num1*transfer->data->t_pow4 + 
        transfer->params.num0*transfer->data->t_pow5;
    transfer->data->data_num[1] = 
        -VALUE_F_1H_SIXTY*transfer->params.num5 - 
        VALUE_F_FORTY_EIGHT*transfer->params.num4*transfer->params.t - 
        VALUE_F_EIGHT*transfer->params.num3*transfer->data->t_pow2 + 
        VALUE_F_FOUR*transfer->params.num2*transfer->data->t_pow3 + 
        VALUE_F_SIX*transfer->params.num1*transfer->data->t_pow4 + 
        VALUE_F_FIVE*transfer->params.num0*transfer->data->t_pow5;
    transfer->data->data_num[2] = 
        VALUE_F_3H_TWENTY*transfer->params.num5 + 
        VALUE_F_THIRTY_TWO*transfer->params.num4*transfer->params.t - 
        VALUE_F_SIXTEEN*transfer->params.num3*transfer->data->t_pow2 - 
        VALUE_F_EIGHT*transfer->params.num2*transfer->data->t_pow3 + 
        VALUE_F_FOUR*transfer->params.num1*transfer->data->t_pow4 + 
        VALUE_F_TEN*transfer->params.num0*transfer->data->t_pow5;
    transfer->data->data_num[3] = 
        -VALUE_F_3H_TWENTY*transfer->params.num5 + 
        VALUE_F_THIRTY_TWO*transfer->params.num4*transfer->params.t + 
        VALUE_F_SIXTEEN*transfer->params.num3*transfer->data->t_pow2 - 
        VALUE_F_EIGHT*transfer->params.num2*transfer->data->t_pow3 - 
        VALUE_F_FOUR*transfer->params.num1*transfer->data->t_pow4 + 
        VALUE_F_TEN*transfer->params.num0*transfer->data->t_pow5;
    transfer->data->data_num[4] = 
        VALUE_F_1H_SIXTY*transfer->params.num5 - 
        VALUE_F_FORTY_EIGHT*transfer->params.num4*transfer->params.t + 
        VALUE_F_EIGHT*transfer->params.num3*transfer->data->t_pow2 + 
        VALUE_F_FOUR*transfer->params.num2*transfer->data->t_pow3 - 
        VALUE_F_SIX*transfer->params.num1*transfer->data->t_pow4 + 
        VALUE_F_FIVE*transfer->params.num0*transfer->data->t_pow5;
    transfer->data->data_num[5] = 
        -VALUE_F_THIRTY_TWO*transfer->params.num5 + 
        VALUE_F_SIXTEEN*transfer->params.num4*transfer->params.t - 
        VALUE_F_EIGHT*transfer->params.num3*transfer->data->t_pow2 + 
        VALUE_F_FOUR*transfer->params.num2*transfer->data->t_pow3 - 
        VALUE_F_TWO*transfer->params.num1*transfer->data->t_pow4 + 
        transfer->params.num0*transfer->data->t_pow5;
    
    transfer->data->data_den[0] = 
        VALUE_F_THIRTY_TWO*transfer->params.den5 + 
        VALUE_F_SIXTEEN*transfer->params.den4*transfer->params.t + 
        VALUE_F_EIGHT*transfer->params.den3*transfer->data->t_pow2 + 
        VALUE_F_FOUR*transfer->params.den2*transfer->data->t_pow3 + 
        VALUE_F_TWO*transfer->params.den1*transfer->data->t_pow4 + 
        transfer->params.den0*transfer->data->t_pow5;
    transfer->data->data_den[1] = 
        -VALUE_F_1H_SIXTY*transfer->params.den5 - 
        VALUE_F_FORTY_EIGHT*transfer->params.den4*transfer->params.t - 
        VALUE_F_EIGHT*transfer->params.den3*transfer->data->t_pow2 + 
        VALUE_F_FOUR*transfer->params.den2*transfer->data->t_pow3 + 
        VALUE_F_SIX*transfer->params.den1*transfer->data->t_pow4 + 
        VALUE_F_FIVE*transfer->params.den0*transfer->data->t_pow5;
    transfer->data->data_den[2] = 
        VALUE_F_3H_TWENTY*transfer->params.den5 + 
        VALUE_F_THIRTY_TWO*transfer->params.den4*transfer->params.t - 
        VALUE_F_SIXTEEN*transfer->params.den3*transfer->data->t_pow2 - 
        VALUE_F_EIGHT*transfer->params.den2*transfer->data->t_pow3 + 
        VALUE_F_FOUR*transfer->params.den1*transfer->data->t_pow4 + 
        VALUE_F_TEN*transfer->params.den0*transfer->data->t_pow5;
    transfer->data->data_den[3] = 
        -VALUE_F_3H_TWENTY*transfer->params.den5 + 
        VALUE_F_THIRTY_TWO*transfer->params.den4*transfer->params.t + 
        VALUE_F_SIXTEEN*transfer->params.den3*transfer->data->t_pow2 - 
        VALUE_F_EIGHT*transfer->params.den2*transfer->data->t_pow3 - 
        VALUE_F_FOUR*transfer->params.den1*transfer->data->t_pow4 + 
        VALUE_F_TEN*transfer->params.den0*transfer->data->t_pow5;
    transfer->data->data_den[4] = 
        VALUE_F_1H_SIXTY*transfer->params.den5 - 
        VALUE_F_FORTY_EIGHT*transfer->params.den4*transfer->params.t + 
        VALUE_F_EIGHT*transfer->params.den3*transfer->data->t_pow2 + 
        VALUE_F_FOUR*transfer->params.den2*transfer->data->t_pow3 - 
        VALUE_F_SIX*transfer->params.den1*transfer->data->t_pow4 + 
        VALUE_F_FIVE*transfer->params.den0*transfer->data->t_pow5;
    transfer->data->data_den[5] = 
        -VALUE_F_THIRTY_TWO*transfer->params.den5 + 
        VALUE_F_SIXTEEN*transfer->params.den4*transfer->params.t - 
        VALUE_F_EIGHT*transfer->params.den3*transfer->data->t_pow2 + 
        VALUE_F_FOUR*transfer->params.den2*transfer->data->t_pow3 - 
        VALUE_F_TWO*transfer->params.den1*transfer->data->t_pow4 + 
        transfer->params.den0*transfer->data->t_pow5;
}

// (典型五阶传递函数，计数5)
static void __local_transfer5_init(__LocalTransfer5 *transfer){
    #if VALUE_MODE_MATH>=VALUE_MODE_Q15
    patch_transfer5_init(transfer);
    #else // VALUE_MODE_MATH
    // 初始化为零
    __LOCAL_ZERO_ALL(transfer);

    // 传递函数运算系数固定计算
    transfer->data->t_pow2 = math_pow(transfer->params.t, 2);
    transfer->data->t_pow3 = math_pow(transfer->params.t, 3);
    transfer->data->t_pow4 = math_pow(transfer->params.t, 4);
    transfer->data->t_pow5 = math_pow(transfer->params.t, 5);
    __local_transfer5_initial(transfer);
    #endif // VALUE_MODE_MATH
}

// (典型五阶传递函数，计数5)
void transfer_transfer5_loop(__LocalTransfer5 *transfer){
    #if VALUE_MODE_MATH>=VALUE_MODE_Q15
    patch_transfer5_loop(transfer);
    #else // VALUE_MODE_MATH
    // 1.传递函数运算系数动态计算
    if (transfer->params.recalculate_total_flag){
        __local_transfer5_initial(transfer);
    }
    
    // 2.运算传递函数
    transfer->out.output = 
        (transfer->in.input*transfer->data->data_num[0] + 
        transfer->data->i[0]*transfer->data->data_num[1] + 
        transfer->data->i[1]*transfer->data->data_num[2] + 
        transfer->data->i[2]*transfer->data->data_num[3] + 
        transfer->data->i[3]*transfer->data->data_num[4] + 
        transfer->data->i[4]*transfer->data->data_num[5] - 
        transfer->data->o[0]*transfer->data->data_den[1] - 
        transfer->data->o[1]*transfer->data->data_den[2] - 
        transfer->data->o[2]*transfer->data->data_den[3] - 
        transfer->data->o[3]*transfer->data->data_den[4] - 
        transfer->data->o[4]*transfer->data->data_den[5])/transfer->data->data_den[0];

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

// (积分器，计数6)-----------------------------------|------------------------------
struct __LocalIntegratorData{
    SguanQ i;                                       // (状态)历史输入
    SguanQ data_num;                                // (系数)分子系数
};

// (积分器，计数6)
static void __local_integrator_init(__LocalIntegrator *integrator){
    #if VALUE_MODE_MATH>=VALUE_MODE_Q15
    patch_integrator_init(integrator);
    #else // VALUE_MODE_MATH
    // 初始化为零
    __LOCAL_ZERO_ALL(integrator);

    // 传递函数运算系数固定计算
    integrator->data->data_num = integrator->params.t/VALUE_F_TWO;
    #endif // VALUE_MODE_MATH
}

// (积分器，计数6)
void transfer_integrator_loop(__LocalIntegrator *integrator){
    #if VALUE_MODE_MATH>=VALUE_MODE_Q15
    patch_integrator_loop(integrator);
    #else // VALUE_MODE_MATH
    // 1.运算传递函数
    integrator->out.output += 
        (integrator->in.input + integrator->data->i)*integrator->data->data_num;

    // 2.更新历史数值
    integrator->data->i = integrator->in.input;
    #endif // VALUE_MODE_MATH
}

// (微分器，计数7)-----------------------------------|------------------------------
struct __LocalDerivativeData{
    SguanQ i;                                       // (状态)历史输入
    SguanQ o;                                       // (状态)历史输出

    SguanQ data_num[2];                             // (系数)分子系数表
    SguanQ data_den[2];                             // (系数)分母系数表
};

// (微分器，计数7)
static void __local_derivative_initial(__LocalDerivative *derivative){
    derivative->data->data_num[0] = VALUE_F_TWO*derivative->params.wc;
    derivative->data->data_num[1] = -VALUE_F_TWO*derivative->params.wc;

    derivative->data->data_den[0] = 
        VALUE_F_TWO + 
        derivative->params.t*derivative->params.wc;
    derivative->data->data_den[1] = 
        -VALUE_F_TWO + 
        derivative->params.t*derivative->params.wc;
}

// (微分器，计数7)
static void __local_derivative_init(__LocalDerivative *derivative){
    #if VALUE_MODE_MATH>=VALUE_MODE_Q15
    patch_derivative_init(derivative);
    #else // VALUE_MODE_MATH
    // 初始化为零
    __LOCAL_ZERO_ALL(derivative);

    // 传递函数运算系数固定计算
    __local_derivative_initial(derivative);
    #endif // VALUE_MODE_MATH
}

// (微分器，计数7)
void transfer_derivative_loop(__LocalDerivative *derivative){
    #if VALUE_MODE_MATH>=VALUE_MODE_Q15
    patch_derivative_loop(derivative);
    #else // VALUE_MODE_MATH
    // 1.传递函数运算系数动态计算
    if (derivative->params.recalculate_total_flag){
        __local_derivative_initial(derivative);
    }

    // 2.运算传递函数
    derivative->out.output = 
        (derivative->in.input*derivative->data->data_num[0] + 
        derivative->data->i*derivative->data->data_num[1] - 
        derivative->data->o*derivative->data->data_den[1])/derivative->data->data_den[0];

    // 3.更新历史数值
    derivative->data->i = derivative->in.input;
    derivative->data->o = derivative->out.output;
    #endif // VALUE_MODE_MATH
}

// (霍尔编码器，计数8)-------------------------------|-------------------------------
struct __LocalHallData{
    SguanQ hall_a;                                  // (状态)霍尔A滤波值
    SguanQ hall_b;                                  // (状态)霍尔B滤波值
    SguanQ hall_c;                                  // (状态)霍尔C滤波值

    SguanQ gain;                                    // (系数)滤波增益
    SguanQ complement_gain;                         // (系数)互补增益
};

// (霍尔编码器，计数8)
static void __local_hall_init(__LocalHall *hall){
    #if VALUE_MODE_MATH>=VALUE_MODE_Q15
    patch_hall_init(hall);
    #else // VALUE_MODE_MATH
    // 初始化为零
    __LOCAL_ZERO_ALL(hall);

    // 传递函数运算系数固定计算
    hall->data->gain = VALUE_F_ONE;
    hall->data->complement_gain = 0;
    #endif // VALUE_MODE_MATH
}

// (霍尔编码器，计数8)
void transfer_hall_loop(__LocalHall *hall){
    #if VALUE_MODE_MATH>=VALUE_MODE_Q15
    patch_hall_loop(hall);
    #else // VALUE_MODE_MATH
    // 1.传递函数运算系数动态计算
    if (hall->params.recalculate_total_flag){
        hall->data->gain = hall->params.t*hall->params.wc;
        hall->data->complement_gain = VALUE_F_ONE - hall->data->gain;
    }

    // 2.创建函数局部变量
    static uint8_t v[6] = {5, 3, 4, 1, 0, 2};
    uint8_t signal_a,signal_b,signal_c,sector;

    // 3.低通滤波及信号边界判断(防抖动)
    hall->data->hall_a = hall->data->gain*((float)hall->in.gpio_a) + 
                    hall->data->complement_gain*hall->data->hall_a;
    hall->data->hall_b = hall->data->gain*((float)hall->in.gpio_b) + 
                    hall->data->complement_gain*hall->data->hall_a;
    hall->data->hall_c = hall->data->gain*((float)hall->in.gpio_c) + 
                    hall->data->complement_gain*hall->data->hall_a;
    
    if (hall->data->hall_a >= hall->params.hall_high)   signal_a = 1;
    else if(hall->data->hall_a <= hall->params.hall_low)signal_a = 0;
    if (hall->data->hall_b >= hall->params.hall_high)   signal_b = 1;
    else if(hall->data->hall_b <= hall->params.hall_low)signal_b = 0;
    if (hall->data->hall_c >= hall->params.hall_high)   signal_c = 1;
    else if(hall->data->hall_c <= hall->params.hall_low)signal_c = 0;

    // 4.霍尔数据处理并输出角度值
    sector = (signal_a << 2) | (signal_b << 1) | (signal_c);
    hall->out.re = (v[sector - 1]*VALUE_F_2PI)/VALUE_F_SIX;
    #endif // VALUE_MODE_MATH
}

// (一阶线性自抗扰控制，计数9)------------------------|------------------------------
struct __LocalLadrc1Data{
    SguanQ z1;                                      // (状态)输出估计
    SguanQ z2;                                      // (状态)总扰动估计
    SguanQ u_last;                                  // (状态)上一拍实际输出

    SguanQ inv_b0;                                  // (系数)b0的倒数

    SguanQ data_phi[2][2];                          // (系数)状态转移矩阵
    SguanQ data_gu[2];                              // (系数)输入增益
    SguanQ data_gy[2];                              // (系数)反馈增益
};

// (一阶线性自抗扰控制，计数9)
static void __local_ladrc1_initial(__LocalLadrc1 *ladrc){
    SguanQ beta1 = VALUE_F_TWO*ladrc->params.wo;
    SguanQ beta2 = ladrc->params.wo*ladrc->params.wo;
    SguanQ h = ladrc->params.t;
    SguanQ b0 = ladrc->params.b0;
    SguanQ a = VALUE_F_HALF*h*beta1;
    SguanQ c = h*h*beta2/VALUE_F_FOUR;
    SguanQ det = VALUE_F_ONE + a + c;
    SguanQ b0_abs = (b0 >= 0) ? b0 : -b0;

    ladrc->data->data_phi[0][0] = (VALUE_F_ONE - a - c)/det;
    ladrc->data->data_phi[0][1] = h/det;
    ladrc->data->data_phi[1][0] = -h*beta2/det;
    ladrc->data->data_phi[1][1] = (VALUE_F_ONE + a - c)/det;

    ladrc->data->data_gu[0] = h*b0/det;
    ladrc->data->data_gu[1] = -(h*h*b0*beta2)/(VALUE_F_TWO*det);
    ladrc->data->data_gy[0] = 
        h*(beta1 + VALUE_F_HALF*h*beta2)/det;
    ladrc->data->data_gy[1] = 
        h*((VALUE_F_ONE + a)*beta2 - 
        VALUE_F_HALF*h*beta2*beta1)/det;
    ladrc->data->inv_b0 = (b0_abs > SGUAN_Q(1e-12))?(VALUE_F_ONE/b0):0;
}

// (一阶线性自抗扰控制，计数9)
static void __local_ladrc1_init(__LocalLadrc1 *ladrc){
    #if VALUE_MODE_MATH>=VALUE_MODE_Q15
    patch_ladrc1_init(ladrc);
    #else // VALUE_MODE_MATH
    // 初始化为零
    __LOCAL_ZERO_ALL(ladrc);

    // 传递函数运算系数固定计算
    __local_ladrc1_initial(ladrc);
    #endif // VALUE_MODE_MATH
}

// (一阶线性自抗扰控制，计数9)
void transfer_ladrc1_loop(__LocalLadrc1 *ladrc){
    #if VALUE_MODE_MATH>=VALUE_MODE_Q15
    patch_ladrc1_loop(ladrc);
    #else // VALUE_MODE_MATH
    // 1.传递函数运算系数动态计算
    if (ladrc->params.recalculate_total_flag){
        __local_ladrc1_initial(ladrc);
    }

    // 2.创建函数局部变量
    SguanQ z1_next;
    SguanQ z2_next;
    SguanQ output;

    // 3.观测器状态更新并抗饱和
    z1_next = ladrc->data->data_phi[0][0]*ladrc->data->z1 + 
              ladrc->data->data_phi[0][1]*ladrc->data->z2 + 
              ladrc->data->data_gu[0]*ladrc->data->u_last + 
              ladrc->data->data_gy[0]*ladrc->in.fbk;
    z2_next = ladrc->data->data_phi[1][0]*ladrc->data->z1 + 
              ladrc->data->data_phi[1][1]*ladrc->data->z2 + 
              ladrc->data->data_gu[1]*ladrc->data->u_last + 
              ladrc->data->data_gy[1]*ladrc->in.fbk;

    // 4.控制律u=(wc*(ref-z1)-z2)/b0及输出限幅
    output = 
        (ladrc->params.wc*(ladrc->in.ref-z1_next) - z2_next)*
        ladrc->data->inv_b0;
    if (output > ladrc->params.out_max) output = ladrc->params.out_max;
    if (output < ladrc->params.out_min) output = ladrc->params.out_min;

    // 5.更新历史数值
    ladrc->data->z1 = z1_next;
    ladrc->data->z2 = z2_next;
    ladrc->data->u_last = output;
    ladrc->out.output = output;
    #endif // VALUE_MODE_MATH
}

// (二阶线性自抗扰控制，计数10)-----------------------|------------------------------
struct __LocalLadrc2Data{
    SguanQ z1;                                      // (状态)输出估计
    SguanQ z2;                                      // (状态)输出导数估计
    SguanQ z3;                                      // (状态)总扰动估计
    SguanQ u_last;                                  // (状态)上一拍实际输出

    SguanQ wc_pow2;                                 // (系数)wc的平方
    SguanQ inv_b0;                                  // (系数)b0的倒数

    SguanQ data_phi[3][3];                          // (系数)状态转移矩阵
    SguanQ data_gu[3];                              // (系数)输入增益
    SguanQ data_gy[3];                              // (系数)反馈增益
};

// (二阶线性自抗扰控制，计数10)
static void __local_ladrc2_initial(__LocalLadrc2 *ladrc){
    SguanQ beta1 = VALUE_F_THREE*ladrc->params.wo;
    SguanQ beta2 = VALUE_F_THREE*ladrc->params.wo*ladrc->params.wo;
    SguanQ beta3 = ladrc->params.wo*ladrc->params.wo*ladrc->params.wo;
    SguanQ h = ladrc->params.t;
    SguanQ b0 = ladrc->params.b0;
    SguanQ b0_abs = (b0 >= 0) ? b0 : -b0;

    SguanQ mat_m[3][3] = {
        {VALUE_F_ONE+VALUE_F_HALF*h*beta1, -VALUE_F_HALF*h, 0},
        {VALUE_F_HALF*h*beta2, VALUE_F_ONE, -VALUE_F_HALF*h},
        {VALUE_F_HALF*h*beta3, 0, VALUE_F_ONE}
    };
    SguanQ mat_n[3][3] = {
        {VALUE_F_ONE-VALUE_F_HALF*h*beta1, VALUE_F_HALF*h,  0},
        {-VALUE_F_HALF*h*beta2, VALUE_F_ONE, VALUE_F_HALF*h},
        {-VALUE_F_HALF*h*beta3, 0, VALUE_F_ONE}
    };
    SguanQ mat_inv[3][3];
    SguanQ vec_u[3];
    SguanQ vec_y[3];
    math_matrix3_inv(mat_m, mat_inv);

    ladrc->data->data_phi[0][0] = 
        mat_inv[0][0]*mat_n[0][0] + mat_inv[0][1]*mat_n[1][0] + mat_inv[0][2]*mat_n[2][0];
    ladrc->data->data_phi[0][1] = 
        mat_inv[0][0]*mat_n[0][1] + mat_inv[0][1]*mat_n[1][1] + mat_inv[0][2]*mat_n[2][1];
    ladrc->data->data_phi[0][2] = 
        mat_inv[0][0]*mat_n[0][2] + mat_inv[0][1]*mat_n[1][2] + mat_inv[0][2]*mat_n[2][2];
    ladrc->data->data_phi[1][0] = 
        mat_inv[1][0]*mat_n[0][0] + mat_inv[1][1]*mat_n[1][0] + mat_inv[1][2]*mat_n[2][0];
    ladrc->data->data_phi[1][1] = 
        mat_inv[1][0]*mat_n[0][1] + mat_inv[1][1]*mat_n[1][1] + mat_inv[1][2]*mat_n[2][1];
    ladrc->data->data_phi[1][2] = 
        mat_inv[1][0]*mat_n[0][2] + mat_inv[1][1]*mat_n[1][2] + mat_inv[1][2]*mat_n[2][2];
    ladrc->data->data_phi[2][0] = 
        mat_inv[2][0]*mat_n[0][0] + mat_inv[2][1]*mat_n[1][0] + mat_inv[2][2]*mat_n[2][0];
    ladrc->data->data_phi[2][1] = 
        mat_inv[2][0]*mat_n[0][1] + mat_inv[2][1]*mat_n[1][1] + mat_inv[2][2]*mat_n[2][1];
    ladrc->data->data_phi[2][2] = 
        mat_inv[2][0]*mat_n[0][2] + mat_inv[2][1]*mat_n[1][2] + mat_inv[2][2]*mat_n[2][2];

    vec_u[0] = 0;
    vec_u[1] = h*b0;
    vec_u[2] = 0;
    ladrc->data->data_gu[0] = mat_inv[0][1]*vec_u[1];
    ladrc->data->data_gu[1] = mat_inv[1][1]*vec_u[1];
    ladrc->data->data_gu[2] = mat_inv[2][1]*vec_u[1];

    vec_y[0] = h*beta1;
    vec_y[1] = h*beta2;
    vec_y[2] = h*beta3;
    ladrc->data->data_gy[0] = 
        mat_inv[0][0]*vec_y[0] + mat_inv[0][1]*vec_y[1] + mat_inv[0][2]*vec_y[2];
    ladrc->data->data_gy[1] = 
        mat_inv[1][0]*vec_y[0] + mat_inv[1][1]*vec_y[1] + mat_inv[1][2]*vec_y[2];
    ladrc->data->data_gy[2] = 
        mat_inv[2][0]*vec_y[0] + mat_inv[2][1]*vec_y[1] + mat_inv[2][2]*vec_y[2];

    ladrc->data->wc_pow2 = ladrc->params.wc*ladrc->params.wc;
    ladrc->data->inv_b0  = (b0_abs > SGUAN_Q(1e-12)) ? (VALUE_F_ONE/b0) : 0;
}

// (二阶线性自抗扰控制，计数10)
static void __local_ladrc2_init(__LocalLadrc2 *ladrc){
    #if VALUE_MODE_MATH>=VALUE_MODE_Q15
    patch_ladrc2_init(ladrc);
    #else // VALUE_MODE_MATH
    // 初始化为零
    __LOCAL_ZERO_ALL(ladrc);

    // 传递函数运算系数固定计算
    __local_ladrc2_initial(ladrc);
    #endif // VALUE_MODE_MATH
}

// (二阶线性自抗扰控制，计数10)
void transfer_ladrc2_loop(__LocalLadrc2 *ladrc){
    #if VALUE_MODE_MATH>=VALUE_MODE_Q15
    patch_ladrc2_loop(ladrc);
    #else // VALUE_MODE_MATH
    // 1.传递函数运算系数动态计算
    if (ladrc->params.recalculate_total_flag){
        __local_ladrc2_initial(ladrc);
    }

    // 2.创建函数局部变量
    SguanQ z1_next;
    SguanQ z2_next;
    SguanQ z3_next;
    SguanQ output;

    // 3.观测器状态更新(输入用上一拍实际输出，实现抗饱和)
    z1_next = ladrc->data->data_phi[0][0]*ladrc->data->z1 + 
              ladrc->data->data_phi[0][1]*ladrc->data->z2 + 
              ladrc->data->data_phi[0][2]*ladrc->data->z3 + 
              ladrc->data->data_gu[0]*ladrc->data->u_last + 
              ladrc->data->data_gy[0]*ladrc->in.fbk;
    z2_next = ladrc->data->data_phi[1][0]*ladrc->data->z1 + 
              ladrc->data->data_phi[1][1]*ladrc->data->z2 + 
              ladrc->data->data_phi[1][2]*ladrc->data->z3 + 
              ladrc->data->data_gu[1]*ladrc->data->u_last + 
              ladrc->data->data_gy[1]*ladrc->in.fbk;
    z3_next = ladrc->data->data_phi[2][0]*ladrc->data->z1 + 
              ladrc->data->data_phi[2][1]*ladrc->data->z2 + 
              ladrc->data->data_phi[2][2]*ladrc->data->z3 + 
              ladrc->data->data_gu[2]*ladrc->data->u_last + 
              ladrc->data->data_gy[2]*ladrc->in.fbk;

    // 4.控制律u=(wc^2*(ref-z1)-2*wc*z2-z3)/b0及输出限幅
    output = (ladrc->data->wc_pow2*(ladrc->in.ref - z1_next) - 
        VALUE_F_TWO*ladrc->params.wc*z2_next - z3_next)*ladrc->data->inv_b0;
    if (output > ladrc->params.out_max) output = ladrc->params.out_max;
    if (output < ladrc->params.out_min) output = ladrc->params.out_min;

    // 5.更新历史数值
    ladrc->data->z1 = z1_next;
    ladrc->data->z2 = z2_next;
    ladrc->data->z3 = z3_next;
    ladrc->data->u_last = output;
    ladrc->out.output = output;
    #endif // VALUE_MODE_MATH
}

// (传统指数型趋近率的滑模控制，计数11)----------------|------------------------------
struct __LocalSmcData{
    SguanQ num;
    SguanQ den;
};

// (传统指数型趋近率的滑模控制，计数11)
static void __local_smc_initial(__LocalSmc *smc){

}

// (传统指数型趋近率的滑模控制，计数11)
static void __local_smc_init(__LocalSmc *smc){
    #if VALUE_MODE_MATH>=VALUE_MODE_Q15
    patch_smc_init(smc);
    #else // VALUE_MODE_MATH


    
    #endif // VALUE_MODE_MATH
}

// (传统指数型趋近率的滑模控制，计数11)
void transfer_smc_loop(__LocalSmc *smc){
    #if VALUE_MODE_MATH>=VALUE_MODE_Q15
    patch_smc_loop(smc);
    #else // VALUE_MODE_MATH


    

    #endif // VALUE_MODE_MATH
}

// (增量式电流预测控制，计数12)-----------------------|------------------------------
struct __LocalDpccData{
    SguanQ num;
    SguanQ den;
};

// (增量式电流预测控制，计数12)
static void __local_dpcc_initial(__LocalDpcc *dpcc){

}

// (增量式电流预测控制，计数12)
static void __local_dpcc_init(__LocalDpcc *dpcc){
    #if VALUE_MODE_MATH>=VALUE_MODE_Q15
    patch_dpcc_init(dpcc);
    #else // VALUE_MODE_MATH


    
    #endif // VALUE_MODE_MATH
}

// (增量式电流预测控制，计数12)
void transfer_dpcc_loop(__LocalDpcc *dpcc){
    #if VALUE_MODE_MATH>=VALUE_MODE_Q15
    patch_dpcc_loop(dpcc);
    #else // VALUE_MODE_MATH


    

    #endif // VALUE_MODE_MATH
}

// (比例积分谐振调节器，计数13)-----------------------|------------------------------
struct __LocalPirData{
    SguanQ num;
    SguanQ den;
};

// (比例积分谐振调节器，计数13)
static void __local_pir_initial(__LocalPir *pir){

}

// (比例积分谐振调节器，计数13)
static void __local_pir_init(__LocalPir *pir){
    #if VALUE_MODE_MATH>=VALUE_MODE_Q15
    patch_pir_init(pir);
    #else // VALUE_MODE_MATH


    
    #endif // VALUE_MODE_MATH
}

// (比例积分谐振调节器，计数13)
void transfer_pir_loop(__LocalPir *pir){
    #if VALUE_MODE_MATH>=VALUE_MODE_Q15
    patch_pir_loop(pir);
    #else // VALUE_MODE_MATH


    

    #endif // VALUE_MODE_MATH
}

// (传统闭环控制器，计数14)---------------------------|------------------------------
struct __LocalPidData{
    SguanQ num;
    SguanQ den;
};

// (传统闭环控制器，计数14)
static void __local_pid_initial(__LocalPid *pid){

}

// (传统闭环控制器，计数14)
static void __local_pid_init(__LocalPid *pid){
    #if VALUE_MODE_MATH>=VALUE_MODE_Q15
    patch_pid_init(pid);
    #else // VALUE_MODE_MATH


    
    #endif // VALUE_MODE_MATH
}

// (传统闭环控制器，计数14)
void transfer_pid_loop(__LocalPid *pid){
    #if VALUE_MODE_MATH>=VALUE_MODE_Q15
    patch_pid_loop(pid);
    #else // VALUE_MODE_MATH


    

    #endif // VALUE_MODE_MATH
}

// (开环锁相环，计数15)------------------------------|-------------------------------
struct __LocalPllData{
    SguanQ num;
    SguanQ den;
};

// (开环锁相环，计数15)
static void __local_pll_initial(__LocalPll *pll){

}

// (开环锁相环，计数15)
static void __local_pll_init(__LocalPll *pll){
    #if VALUE_MODE_MATH>=VALUE_MODE_Q15
    patch_pll_init(pll);
    #else // VALUE_MODE_MATH


    
    #endif // VALUE_MODE_MATH
}

// (开环锁相环，计数15)
void transfer_pll_loop(__LocalPll *pll){
    #if VALUE_MODE_MATH>=VALUE_MODE_Q15
    patch_pll_loop(pll);
    #else // VALUE_MODE_MATH


    

    #endif // VALUE_MODE_MATH
}

// (一阶低通滤波器，计数16)---------------------------|------------------------------
struct __LocalLpf1Data{
    SguanQ i;                                       // (状态)历史输入
    SguanQ o;                                       // (状态)历史输出

    SguanQ data_num;                                // (系数)分子系数
    SguanQ data_den[2];                             // (系数)分母系数表
};

// (一阶低通滤波器，计数16)
static void __local_lpf1_initial(__LocalLpf1 *lpf){
    SguanQ temp = lpf->params.t*lpf->params.wc;

    lpf->data->data_num = temp;
    lpf->data->data_den[0] = VALUE_F_TWO + temp;
    lpf->data->data_den[1] = -VALUE_F_TWO + temp;
}

// (一阶低通滤波器，计数16)
static void __local_lpf1_init(__LocalLpf1 *lpf){
    #if VALUE_MODE_MATH>=VALUE_MODE_Q15
    patch_lpf1_init(lpf);
    #else // VALUE_MODE_MATH
    // 初始化为零
    __LOCAL_ZERO_ALL(lpf);

    // 传递函数运算系数固定计算
    __local_lpf1_initial(lpf);
    #endif // VALUE_MODE_MATH
}

// (一阶低通滤波器，计数16)
void transfer_lpf1_loop(__LocalLpf1 *lpf){
    #if VALUE_MODE_MATH>=VALUE_MODE_Q15
    patch_lpf1_loop(lpf);
    #else // VALUE_MODE_MATH
    // 1.传递函数运算系数动态计算
    if (lpf->params.recalculate_total_flag){
        __local_lpf1_initial(lpf);
    }

    // 2.运算传递函数
    lpf->out.output = 
        ((lpf->in.input+lpf->data->i)*lpf->data->data_num - 
        lpf->data->o*lpf->data->data_den[1])/lpf->data->data_den[0];

    // 3.更新历史数值
    lpf->data->i = lpf->in.input;
    lpf->data->o = lpf->out.output;
    #endif // VALUE_MODE_MATH
}

// (二阶低通滤波器，计数17)---------------------------|------------------------------
struct __LocalLpf2Data{
    SguanQ t_pow2;                                  // (系数)t的平方

    SguanQ i[2];                                    // (状态)输入历史
    SguanQ o[2];                                    // (状态)输出历史
    
    SguanQ data_num[2];                             // (系数)分子系数表
    SguanQ data_den[3];                             // (系数)分母系数表
};

// (二阶低通滤波器，计数17)
static void __local_lpf2_initial(__LocalLpf2 *lpf){
    SguanQ twc = lpf->params.t*lpf->params.wc;
    SguanQ twc_pow2 = lpf->data->t_pow2*lpf->params.wc*lpf->params.wc;

    lpf->data->data_num[0] = twc_pow2;
    lpf->data->data_num[1] = VALUE_F_TWO*twc_pow2;

    lpf->data->data_den[0] = VALUE_F_FOUR + VALUE_F_2_SQRT2*twc + twc_pow2;
    lpf->data->data_den[1] = -VALUE_F_EIGHT + VALUE_F_TWO*twc_pow2;
    lpf->data->data_den[2] = VALUE_F_FOUR - VALUE_F_2_SQRT2*twc + twc_pow2;
}

// (二阶低通滤波器，计数17)
static void __local_lpf2_init(__LocalLpf2 *lpf){
    #if VALUE_MODE_MATH>=VALUE_MODE_Q15
    patch_lpf2_init(lpf);
    #else // VALUE_MODE_MATH
    // 初始化为零
    __LOCAL_ZERO_ALL(lpf);

    // 传递函数运算系数固定计算
    lpf->data->t_pow2 = math_pow(lpf->params.t, 2);
    __local_lpf2_initial(lpf);
    #endif // VALUE_MODE_MATH
}

// (二阶低通滤波器，计数17)
void transfer_lpf2_loop(__LocalLpf2 *lpf){
    #if VALUE_MODE_MATH>=VALUE_MODE_Q15
    patch_lpf2_loop(lpf);
    #else // VALUE_MODE_MATH
    // 1.传递函数运算系数动态计算
    if (lpf->params.recalculate_total_flag){
        __local_lpf2_initial(lpf);
    }

    // 2.运算传递函数
    lpf->out.output = 
        ((lpf->in.input + lpf->data->i[1])*lpf->data->data_num[0] + 
        lpf->data->i[0]*lpf->data->data_num[1] - 
        lpf->data->o[0]*lpf->data->data_den[1] - 
        lpf->data->o[1]*lpf->data->data_den[2])/lpf->data->data_den[0];

    // 3.更新历史数值
    lpf->data->i[1] = lpf->data->i[0];
    lpf->data->i[0] = lpf->in.input;
    
    lpf->data->o[1] = lpf->data->o[0];
    lpf->data->o[0] = lpf->out.output;
    #endif // VALUE_MODE_MATH
}

// (一阶高通滤波器，计数18)---------------------------|------------------------------
struct __LocalHpf1Data{
    SguanQ i;                                       // (状态)历史输入
    SguanQ o;                                       // (状态)历史输出

    SguanQ data_num[2];                             // (系数)分子系数表
    SguanQ data_den[2];                             // (系数)分母系数表
};

// (一阶高通滤波器，计数18)
static void __local_hpf1_initial(__LocalHpf1 *hpf){
    SguanQ twc = hpf->params.t*hpf->params.wc;

    hpf->data->data_num[0] = VALUE_F_TWO;
    hpf->data->data_num[1] = -VALUE_F_TWO;

    hpf->data->data_den[0] = VALUE_F_TWO + twc;
    hpf->data->data_den[1] = -VALUE_F_TWO + twc;
}

// (一阶高通滤波器，计数18)
static void __local_hpf1_init(__LocalHpf1 *hpf){
    #if VALUE_MODE_MATH>=VALUE_MODE_Q15
    patch_hpf1_init(hpf);
    #else // VALUE_MODE_MATH
    // 初始化为零
    __LOCAL_ZERO_ALL(hpf);

    // 传递函数运算系数固定计算
    __local_hpf1_initial(hpf);
    #endif // VALUE_MODE_MATH
}

// (一阶高通滤波器，计数18)
void transfer_hpf1_loop(__LocalHpf1 *hpf){
    #if VALUE_MODE_MATH>=VALUE_MODE_Q15
    patch_hpf1_loop(hpf);
    #else // VALUE_MODE_MATH
    // 1.传递函数运算系数动态计算
    if (hpf->params.recalculate_total_flag){
        __local_hpf1_initial(hpf);
    }

    // 2.运算传递函数
    hpf->out.output = 
        (hpf->in.input*hpf->data->data_num[0] + 
        hpf->data->i*hpf->data->data_num[1] - 
        hpf->data->o*hpf->data->data_den[1])/hpf->data->data_den[0];

    // 3.更新历史数值
    hpf->data->i = hpf->in.input;
    hpf->data->o = hpf->out.output;
    #endif // VALUE_MODE_MATH
}

// (二阶高通滤波器，计数19)---------------------------|------------------------------
struct __LocalHpf2Data{
    SguanQ i[2];                                    // (状态)输入历史
    SguanQ o[2];                                    // (状态)输出历史

    SguanQ data_num[2];                             // (系数)分子系数表
    SguanQ data_den[3];                             // (系数)分母系数表
};

// (二阶高通滤波器，计数19)
static void __local_hpf2_initial(__LocalHpf2 *hpf){
    SguanQ twc = hpf->params.t*hpf->params.wc;
    SguanQ twc_pow2 = 
        hpf->params.t*hpf->params.wc*hpf->params.t*hpf->params.wc;

    hpf->data->data_num[0] = VALUE_F_FOUR;
    hpf->data->data_num[1] = -VALUE_F_EIGHT;

    hpf->data->data_den[0] = VALUE_F_FOUR + VALUE_F_2_SQRT2*twc + twc_pow2;
    hpf->data->data_den[1] = -VALUE_F_EIGHT + VALUE_F_TWO*twc_pow2;
    hpf->data->data_den[2] = VALUE_F_FOUR - VALUE_F_2_SQRT2*twc + twc_pow2;
}

// (二阶高通滤波器，计数19)
static void __local_hpf2_init(__LocalHpf2 *hpf){
    #if VALUE_MODE_MATH>=VALUE_MODE_Q15
    patch_hpf2_init(hpf);
    #else // VALUE_MODE_MATH
    // 初始化为零
    __LOCAL_ZERO_ALL(hpf);

    // 传递函数运算系数固定计算
    __local_hpf2_initial(hpf);
    #endif // VALUE_MODE_MATH
}

// (二阶高通滤波器，计数19)
void transfer_hpf2_loop(__LocalHpf2 *hpf){
    #if VALUE_MODE_MATH>=VALUE_MODE_Q15
    patch_hpf2_loop(hpf);
    #else // VALUE_MODE_MATH
    // 1.传递函数运算系数动态计算
    if (hpf->params.recalculate_total_flag){
        __local_hpf2_initial(hpf);
    }

    // 2.运算传递函数
    hpf->out.output = 
        ((hpf->in.input + hpf->data->i[1])*hpf->data->data_num[0] + 
        hpf->data->i[0]*hpf->data->data_num[1] - 
        hpf->data->o[0]*hpf->data->data_den[1] - 
        hpf->data->o[1]*hpf->data->data_den[2])/hpf->data->data_den[0];

    // 3.更新历史数值
    hpf->data->i[1] = hpf->data->i[0];
    hpf->data->i[0] = hpf->in.input;
    
    hpf->data->o[1] = hpf->data->o[0];
    hpf->data->o[0] = hpf->out.output;
    #endif // VALUE_MODE_MATH
}

// (带通滤波器"一阶低通和高通串联"，计数20)------------|------------------------------
struct __LocalBpf1Data{
    SguanQ i[2];                                    // (状态)输入历史
    SguanQ o[2];                                    // (状态)输出历史

    SguanQ data_num[2];                             // (系数)分子系数表
    SguanQ data_den[3];                             // (系数)分母系数表
};

// (带通滤波器"一阶低通和高通串联"，计数20)
static void __local_bpf1_initial(__LocalBpf1 *bpf){
    SguanQ twc_low = bpf->params.t*bpf->params.wc_low;
    SguanQ twc_high = bpf->params.t*bpf->params.wc_high;

    bpf->data->data_num[0] = VALUE_F_TWO*twc_low;
    bpf->data->data_num[1] = -VALUE_F_TWO*twc_low;

    bpf->data->data_den[0] = 
        VALUE_F_FOUR + 
        VALUE_F_TWO*twc_high + 
        VALUE_F_TWO*twc_low + 
        twc_high*twc_high;
    bpf->data->data_den[1] = 
        -VALUE_F_EIGHT + 
        VALUE_F_TWO*twc_high*twc_high;
    bpf->data->data_den[2] = 
        VALUE_F_FOUR - 
        VALUE_F_TWO*twc_high - 
        VALUE_F_TWO*twc_low + 
        twc_high*twc_high;
}

// (带通滤波器"一阶低通和高通串联"，计数20)
static void __local_bpf1_init(__LocalBpf1 *bpf){
    #if VALUE_MODE_MATH>=VALUE_MODE_Q15
    patch_bpf1_init(bpf);
    #else // VALUE_MODE_MATH
    // 初始化为零
    __LOCAL_ZERO_ALL(bpf);

    // 传递函数运算系数固定计算
    __local_bpf1_initial(bpf);
    #endif // VALUE_MODE_MATH
}

// (带通滤波器"一阶低通和高通串联"，计数20)
void transfer_bpf1_loop(__LocalBpf1 *bpf){
    #if VALUE_MODE_MATH>=VALUE_MODE_Q15
    patch_bpf1_loop(bpf);
    #else // VALUE_MODE_MATH
    // 1.传递函数运算系数动态计算
    if (bpf->params.recalculate_total_flag){
        __local_bpf1_initial(bpf);
    }

    // 2.运算传递函数
    bpf->out.output = 
        (bpf->in.input*bpf->data->data_num[0] + 
        bpf->data->i[1]*bpf->data->data_num[1] - 
        bpf->data->o[0]*bpf->data->data_den[1] - 
        bpf->data->o[1]*bpf->data->data_den[2])/bpf->data->data_den[0];

    // 3.更新历史数值
    bpf->data->i[1] = bpf->data->i[0];
    bpf->data->i[0] = bpf->in.input;

    bpf->data->o[1] = bpf->data->o[0];
    bpf->data->o[0] = bpf->out.output;
    #endif // VALUE_MODE_MATH
}

// (带通滤波器"典型二阶系统改型"，计数21)--------------|------------------------------
struct __LocalBpf2Data{
    SguanQ i[2];                                    // (状态)输入历史
    SguanQ o[2];                                    // (状态)输出历史

    SguanQ data_num[2];                             // (系数)分子系数表
    SguanQ data_den[3];                             // (系数)分母系数表
};

// (带通滤波器"典型二阶系统改型"，计数21)
static void __local_bpf2_initial(__LocalBpf2 *bpf){
    SguanQ two_zeta = bpf->params.t*bpf->params.wo*bpf->params.zeta;
    SguanQ two_pow2 = bpf->params.t*bpf->params.wo*bpf->params.t*bpf->params.wo;

    bpf->data->data_num[0] = VALUE_F_FOUR*two_zeta;
    bpf->data->data_num[1] = -VALUE_F_FOUR*two_zeta;

    bpf->data->data_den[0] = VALUE_F_FOUR + VALUE_F_FOUR*two_zeta + two_pow2;
    bpf->data->data_den[1] = -VALUE_F_EIGHT + VALUE_F_TWO*two_pow2;
    bpf->data->data_den[2] = VALUE_F_FOUR - VALUE_F_FOUR*two_zeta + two_pow2;
}

// (带通滤波器"典型二阶系统改型"，计数21)
static void __local_bpf2_init(__LocalBpf2 *bpf){
    #if VALUE_MODE_MATH>=VALUE_MODE_Q15
    patch_bpf2_init(bpf);
    #else // VALUE_MODE_MATH
    // 初始化为零
    __LOCAL_ZERO_ALL(bpf);

    // 传递函数运算系数固定计算
    __local_bpf2_initial(bpf);
    #endif // VALUE_MODE_MATH
}

// (带通滤波器"典型二阶系统改型"，计数21)
void transfer_bpf2_loop(__LocalBpf2 *bpf){
    #if VALUE_MODE_MATH>=VALUE_MODE_Q15
    patch_bpf2_loop(bpf);
    #else // VALUE_MODE_MATH
    // 1.传递函数运算系数动态计算
    if (bpf->params.recalculate_total_flag){
        __local_bpf2_initial(bpf);
    }

    // 2.运算传递函数
    bpf->out.output = 
        (bpf->in.input*bpf->data->data_num[0] + 
        bpf->data->i[1]*bpf->data->data_num[1] - 
        bpf->data->o[0]*bpf->data->data_den[1] - 
        bpf->data->o[1]*bpf->data->data_den[2])/bpf->data->data_den[0];

    // 3.更新历史数值
    bpf->data->i[1] = bpf->data->i[0];
    bpf->data->i[0] = bpf->in.input;

    bpf->data->o[1] = bpf->data->o[0];
    bpf->data->o[0] = bpf->out.output;
    #endif // VALUE_MODE_MATH
}

// (广义积分器"典型二阶系统改型"，计数22)--------------|------------------------------
struct __LocalSogiData{
    SguanQ i[2];
    SguanQ o[2];

    SguanQ data_num[2];
    SguanQ data_den[3];
};

// (广义积分器"典型二阶系统改型"，计数22)
static void __local_sogi_initial(__LocalSogi *sogi){

}

// (广义积分器"典型二阶系统改型"，计数22)
static void __local_sogi_init(__LocalSogi *sogi){
    #if VALUE_MODE_MATH>=VALUE_MODE_Q15
    patch_sogi_init(sogi);
    #else // VALUE_MODE_MATH
    // 传递函数运算系数固定计算


    // 初始化为零

    #endif // VALUE_MODE_MATH
}

// (广义积分器"典型二阶系统改型"，计数22)
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

// (陷波滤波器"典型二阶系统改型"，计数23)--------------|------------------------------
struct __LocalNfData{
    SguanQ i[2];                                    // (状态)输入历史
    SguanQ o[2];                                    // (状态)输出历史

    SguanQ data_num[2];                             // (系数)分子系数表
    SguanQ data_den[3];                             // (系数)分母系数表
};

// (陷波滤波器"典型二阶系统改型"，计数23)
static void __local_nf_initial(__LocalNf *nf){
    SguanQ two_zeta = nf->params.t*nf->params.wo*nf->params.zeta;
    SguanQ two_pow2 = nf->params.t*nf->params.wo*nf->params.t*nf->params.wo;

    nf->data->data_num[0] = VALUE_F_FOUR + two_pow2;
    nf->data->data_num[1] = -VALUE_F_EIGHT + VALUE_F_TWO*two_pow2;

    nf->data->data_den[0] = VALUE_F_FOUR + VALUE_F_FOUR*two_zeta + two_pow2;
    nf->data->data_den[1] = -VALUE_F_EIGHT + VALUE_F_TWO*two_pow2;
    nf->data->data_den[2] = VALUE_F_FOUR - VALUE_F_FOUR*two_zeta + two_pow2;
}

// (陷波滤波器"典型二阶系统改型"，计数23)
static void __local_nf_init(__LocalNf *nf){
    #if VALUE_MODE_MATH>=VALUE_MODE_Q15
    patch_nf_init(nf);
    #else // VALUE_MODE_MATH
    // 初始化为零
    __LOCAL_ZERO_ALL(nf);

    // 传递函数运算系数固定计算
    __local_nf_initial(nf);
    #endif // VALUE_MODE_MATH
}

// (陷波滤波器"典型二阶系统改型"，计数23)
void transfer_nf_loop(__LocalNf *nf){
    #if VALUE_MODE_MATH>=VALUE_MODE_Q15
    patch_nf_loop(nf);
    #else // VALUE_MODE_MATH
    // 1.传递函数运算系数动态计算
    if (nf->params.recalculate_total_flag){
        __local_nf_initial(nf);
    }

    // 2.运算传递函数
    nf->out.output = 
        ((nf->in.input + nf->data->i[1])*nf->data->data_num[0] + 
        nf->data->i[0]*nf->data->data_num[1] - 
        nf->data->o[0]*nf->data->data_den[1] - 
        nf->data->o[1]*nf->data->data_den[2])/nf->data->data_den[0];

    // 3.更新历史数值
    nf->data->i[1] = nf->data->i[0];
    nf->data->i[0] = nf->in.input;
    
    nf->data->o[1] = nf->data->o[0];
    nf->data->o[0] = nf->out.output;
    #endif // VALUE_MODE_MATH
}

// (陷波滤波器"三参数陷波滤波器"，计数24)--------------|------------------------------
struct __LocalTpnfData{
    SguanQ i[2];                                    // (状态)输入历史
    SguanQ o[2];                                    // (状态)输出历史

    SguanQ data_num[3];                             // (系数)分子系数表
    SguanQ data_den[3];                             // (系数)分母系数表
};

// (陷波滤波器"三参数陷波滤波器"，计数24)
static void __local_tpnf_initial(__LocalTpnf *tpnf){
    SguanQ two_k2 = tpnf->params.k2*tpnf->params.t*tpnf->params.wo;
    SguanQ two_k1 = tpnf->params.k1*tpnf->params.t*tpnf->params.wo;
    SguanQ two_pow2 = tpnf->params.t*tpnf->params.wo*tpnf->params.t*tpnf->params.wo;

    tpnf->data->data_num[0] = VALUE_F_FOUR + VALUE_F_FOUR*two_k2 + two_pow2;
    tpnf->data->data_num[1] = -VALUE_F_EIGHT + VALUE_F_TWO*two_pow2;
    tpnf->data->data_num[2] = VALUE_F_FOUR - VALUE_F_FOUR*two_k2 + two_pow2;

    tpnf->data->data_den[0] = VALUE_F_FOUR + VALUE_F_FOUR*two_k1 + two_pow2;
    tpnf->data->data_den[1] = -VALUE_F_EIGHT + VALUE_F_TWO*two_pow2;
    tpnf->data->data_den[2] = VALUE_F_FOUR - VALUE_F_FOUR*two_k1 + two_pow2;
}

// (陷波滤波器"三参数陷波滤波器"，计数24)
static void __local_tpnf_init(__LocalTpnf *tpnf){
    #if VALUE_MODE_MATH>=VALUE_MODE_Q15
    patch_tpnf_init(tpnf);
    #else // VALUE_MODE_MATH
    // 初始化为零
    __LOCAL_ZERO_ALL(tpnf);

    // 传递函数运算系数固定计算
    __local_tpnf_initial(tpnf);
    #endif // VALUE_MODE_MATH
}

// (陷波滤波器"三参数陷波滤波器"，计数24)
void transfer_tpnf_loop(__LocalTpnf *tpnf){
    #if VALUE_MODE_MATH>=VALUE_MODE_Q15
    patch_tpnf_loop(tpnf);
    #else // VALUE_MODE_MATH
    // 1.传递函数运算系数动态计算
    if (tpnf->params.recalculate_total_flag){
        __local_tpnf_initial(tpnf);
    }

    // 2.运算传递函数
    tpnf->out.output = 
        (tpnf->in.input*tpnf->data->data_num[0] + 
        tpnf->data->i[0]*tpnf->data->data_num[1] + 
        tpnf->data->i[1]*tpnf->data->data_num[2] - 
        tpnf->data->o[0]*tpnf->data->data_den[1] - 
        tpnf->data->o[1]*tpnf->data->data_den[2])/tpnf->data->data_den[0];

    // 3.更新历史数值
    tpnf->data->i[1] = tpnf->data->i[0];
    tpnf->data->i[0] = tpnf->in.input;
    
    tpnf->data->o[1] = tpnf->data->o[0];
    tpnf->data->o[0] = tpnf->out.output;
    #endif // VALUE_MODE_MATH
}

// (超螺旋滑模扰动观测器，计数25)---------------------|------------------------------
struct __LocalDobData{
    SguanQ num;
    SguanQ den;
};

// (超螺旋滑模扰动观测器，计数25)
static void __local_dob_initial(__LocalDob *dob){

}

// (超螺旋滑模扰动观测器，计数25)
static void __local_dob_init(__LocalDob *dob){
    #if VALUE_MODE_MATH>=VALUE_MODE_Q15
    patch_dob_init(dob);
    #else // VALUE_MODE_MATH


    
    #endif // VALUE_MODE_MATH
}

// (超螺旋滑模扰动观测器，计数25)
void transfer_dob_loop(__LocalDob *dob){
    #if VALUE_MODE_MATH>=VALUE_MODE_Q15
    patch_dob_loop(dob);
    #else // VALUE_MODE_MATH


    

    #endif // VALUE_MODE_MATH
}

// (电机参数在线辨识观测器，计数26)-------------------|-------------------------------
struct __LocalRlsData{
    SguanQ num;
    SguanQ den;
};

// (电机参数在线辨识观测器，计数26)
static void __local_rls_initial(__LocalRls *rls){

}

// (电机参数在线辨识观测器，计数26)
static void __local_rls_init(__LocalRls *rls){
    #if VALUE_MODE_MATH>=VALUE_MODE_Q15
    patch_rls_init(rls);
    #else // VALUE_MODE_MATH


    
    #endif // VALUE_MODE_MATH
}

// (电机参数在线辨识观测器，计数26)
void transfer_rls_loop(__LocalRls *rls){
    #if VALUE_MODE_MATH>=VALUE_MODE_Q15
    patch_rls_loop(rls);
    #else // VALUE_MODE_MATH


    

    #endif // VALUE_MODE_MATH
}

// ("无感"滑模观测器，计数27)-------------------------|------------------------------
struct __LocalSmoData{
    SguanQ num;
    SguanQ den;
};

// ("无感"滑模观测器，计数27)
static void __local_smo_initial(__LocalSmo *smo){

}

// ("无感"滑模观测器，计数27)
static void __local_smo_init(__LocalSmo *smo){
    #if VALUE_MODE_MATH>=VALUE_MODE_Q15
    patch_smo_init(smo);
    #else // VALUE_MODE_MATH


    
    #endif // VALUE_MODE_MATH
}

// ("无感"滑模观测器，计数27)
void transfer_smo_loop(__LocalSmo *smo){
    #if VALUE_MODE_MATH>=VALUE_MODE_Q15
    patch_smo_loop(smo);
    #else // VALUE_MODE_MATH


    

    #endif // VALUE_MODE_MATH
}

// ("无感"非线性磁链观测器，计数28)-------------------|------------------------------
struct __LocalNfoData{
    SguanQ num;
    SguanQ den;
};

// ("无感"非线性磁链观测器，计数28)
static void __local_nfo_initial(__LocalNfo *nfo){

}

// ("无感"非线性磁链观测器，计数28)
static void __local_nfo_init(__LocalNfo *nfo){
    #if VALUE_MODE_MATH>=VALUE_MODE_Q15
    patch_nfo_init(nfo);
    #else // VALUE_MODE_MATH


    
    #endif // VALUE_MODE_MATH
}

// ("无感"非线性磁链观测器，计数28)
void transfer_nfo_loop(__LocalNfo *nfo){
    #if VALUE_MODE_MATH>=VALUE_MODE_Q15
    patch_nfo_loop(nfo);
    #else // VALUE_MODE_MATH


    

    #endif // VALUE_MODE_MATH
}

// ("无感"有效磁链观测器，计数29)---------------------|------------------------------
struct __LocalAfoData{
    SguanQ num;
    SguanQ den;
};

// ("无感"有效磁链观测器，计数29)
static void __local_afo_initial(__LocalAfo *afo){

}

// ("无感"有效磁链观测器，计数29)
static void __local_afo_init(__LocalAfo *afo){
    #if VALUE_MODE_MATH>=VALUE_MODE_Q15
    patch_afo_init(afo);
    #else // VALUE_MODE_MATH


    
    #endif // VALUE_MODE_MATH
}

// ("无感"有效磁链观测器，计数29)
void transfer_afo_loop(__LocalAfo *afo){
    #if VALUE_MODE_MATH>=VALUE_MODE_Q15
    patch_afo_loop(afo);
    #else // VALUE_MODE_MATH


    

    #endif // VALUE_MODE_MATH
}

// ("无感"高频正弦波注入，计数30)---------------------|------------------------------
struct __LocalHfiData{
    SguanQ num;
    SguanQ den;
};

// ("无感"高频正弦波注入，计数30)
static void __local_hfi_initial(__LocalHfi *hfi){

}

// ("无感"高频正弦波注入，计数30)
static void __local_hfi_init(__LocalHfi *hfi){
    #if VALUE_MODE_MATH>=VALUE_MODE_Q15
    patch_hfi_init(hfi);
    #else // VALUE_MODE_MATH


    
    #endif // VALUE_MODE_MATH
}

// ("无感"高频正弦波注入，计数30)
void transfer_hfi_loop(__LocalHfi *hfi){
    #if VALUE_MODE_MATH>=VALUE_MODE_Q15
    patch_hfi_loop(hfi);
    #else // VALUE_MODE_MATH


    

    #endif // VALUE_MODE_MATH
}

// ("无感"降阶龙伯格观测器，计数31)-------------------|------------------------------
struct __LocalRoloData{
    SguanQ num;
    SguanQ den;
};

// ("无感"降阶龙伯格观测器，计数31)
static void __local_rolo_initial(__LocalRolo *rolo){

}

// ("无感"降阶龙伯格观测器，计数31)
static void __local_rolo_init(__LocalRolo *rolo){
    #if VALUE_MODE_MATH>=VALUE_MODE_Q15
    patch_rolo_init(rolo);
    #else // VALUE_MODE_MATH


    
    #endif // VALUE_MODE_MATH
}

// ("无感"降阶龙伯格观测器，计数31)
void transfer_rolo_loop(__LocalRolo *rolo){
    #if VALUE_MODE_MATH>=VALUE_MODE_Q15
    patch_rolo_loop(rolo);
    #else // VALUE_MODE_MATH


    

    #endif // VALUE_MODE_MATH
}

// ("无感"模型参考自适应观测器，计数32)---------------|-------------------------------
struct __LocalMarsData{
    SguanQ num;
    SguanQ den;
};

// ("无感"模型参考自适应观测器，计数32)
static void __local_mars_initial(__LocalMars *mars){

}

// ("无感"模型参考自适应观测器，计数32)
static void __local_mars_init(__LocalMars *mars){
    #if VALUE_MODE_MATH>=VALUE_MODE_Q15
    patch_mars_init(mars);
    #else // VALUE_MODE_MATH


    
    #endif // VALUE_MODE_MATH
}

// ("无感"模型参考自适应观测器，计数32)
void transfer_mars_loop(__LocalMars *mars){
    #if VALUE_MODE_MATH>=VALUE_MODE_Q15
    patch_mars_loop(mars);
    #else // VALUE_MODE_MATH


    

    #endif // VALUE_MODE_MATH
}

// ("无感"扩展卡尔曼滤波，计数33)---------------------|------------------------------
struct __LocalEkfData{
    SguanQ num;
    SguanQ den;
};

// ("无感"扩展卡尔曼滤波，计数33)
static void __local_ekf_initial(__LocalEkf *ekf){

}

// ("无感"扩展卡尔曼滤波，计数33)
static void __local_ekf_init(__LocalEkf *ekf){
    #if VALUE_MODE_MATH>=VALUE_MODE_Q15
    patch_ekf_init(ekf);
    #else // VALUE_MODE_MATH


    
    #endif // VALUE_MODE_MATH
}

// ("无感"扩展卡尔曼滤波，计数33)
void transfer_ekf_loop(__LocalEkf *ekf){
    #if VALUE_MODE_MATH>=VALUE_MODE_Q15
    patch_ekf_loop(ekf);
    #else // VALUE_MODE_MATH


    

    #endif // VALUE_MODE_MATH
}

// ================================================================================
// 4.全局函数transfer延时模块
// (延时函数"延时一拍"，计数①)------------------------|------------------------------
struct __LocalDelay1Data{
    SguanQ last;                                    // (状态)上一拍输入
};

// (延时函数"延时一拍"，计数①)
static void __local_delay1_init(__LocalDelay1 *delay){
    // 初始化为零
    __LOCAL_ZERO_ALL(delay);
}

// (延时函数"延时一拍"，计数①)
void transfer_delay1_loop(__LocalDelay1 *delay){
    // 1.运算结果
    delay->out.output = delay->data->last;

    // 2.更新历史数值
    delay->data->last = delay->in.input;
}

// (延时函数"延时二拍"，计数②)------------------------|------------------------------
struct __LocalDelay2Data{
    SguanQ last[2];                                 // (状态)输入历史
};

// (延时函数"延时二拍"，计数②)
static void __local_delay2_init(__LocalDelay2 *delay){
    // 初始化为零
    __LOCAL_ZERO_ALL(delay);
}

// (延时函数"延时二拍"，计数②)
void transfer_delay2_loop(__LocalDelay2 *delay){
    // 1.运算结果
    delay->out.output = delay->data->last[1];

    // 2.更新历史数值
    delay->data->last[1] = delay->data->last[0];
    delay->data->last[0] = delay->in.input;
}

// (延时函数"延时三拍"，计数③)------------------------|------------------------------
struct __LocalDelay3Data{
    SguanQ last[3];                                 // (状态)输入历史
};

// (延时函数"延时三拍"，计数③)
static void __local_delay3_init(__LocalDelay3 *delay){
    // 初始化为零
    __LOCAL_ZERO_ALL(delay);
}

// (延时函数"延时三拍"，计数③)
void transfer_delay3_loop(__LocalDelay3 *delay){
    // 1.运算结果
    delay->out.output = delay->data->last[2];

    // 2.更新历史数值
    delay->data->last[2] = delay->data->last[1];
    delay->data->last[1] = delay->data->last[0];
    delay->data->last[0] = delay->in.input;  
}

// ================================================================================
// 5.全局函数transfer基础模块
// (正弦发生器，计数1！)------------------------------------------------------------
void transfer_sine_loop(__LocalSine *sine){
    sine->out.output = math_sin(sine->in.input);
}

// (余弦发生器，计数2！)------------------------------------------------------------
void transfer_cosine_loop(__LocalCosine *cosine){
    cosine->out.output = math_cos(cosine->in.input);
}

// (正余弦发生器，计数3！)----------------------------------------------------------
void transfer_sincos_loop(__LocalSinCos *sincos){
    math_sin_cos(sincos->in.input, 
        &sincos->out.sine, 
        &sincos->out.cosine);
}

// (正切求解器，计数4！)------------------------------------------------------------
void transfer_tan_loop(__LocalTan *tan){
    tan->out.output = math_tan(tan->in.input, tan->in.base);
}

// (反正切求解器，计数5！)----------------------------------------------------------
void transfer_atan_loop(__LocalAtan *atan){
    atan->out.output = math_tan(atan->in.input, atan->in.base);
}

// ================================================================================
// 6.实例获取静态池(实现)
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
#if CONFIG_NFO
static __LocalNfo nfo_pool[CONFIG_MOTOR][CONFIG_NFO];
static __LocalNfoData nfo_data_pool[CONFIG_MOTOR][CONFIG_NFO];
__LocalNfo *__local_nfo_get(uint8_t motor, int ch){
    if (motor >= CONFIG_MOTOR || ch >= CONFIG_NFO) return 0;
    __LocalNfo *self = &nfo_pool[motor][ch];
    self->data = &nfo_data_pool[motor][ch];
    return self;
}
#endif
#if CONFIG_AFO
static __LocalAfo afo_pool[CONFIG_MOTOR][CONFIG_AFO];
static __LocalAfoData afo_data_pool[CONFIG_MOTOR][CONFIG_AFO];
__LocalAfo *__local_afo_get(uint8_t motor, int ch){
    if (motor >= CONFIG_MOTOR || ch >= CONFIG_AFO) return 0;
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
// (分割线)------------------------------------------------------------------------
#if CONFIG_DELAY1
static __LocalDelay1 delay1_pool[CONFIG_MOTOR][CONFIG_DELAY1];
static __LocalDelay1Data delay1_data_pool[CONFIG_MOTOR][CONFIG_DELAY1];
__LocalDelay1 *__local_delay1_get(uint8_t motor, int ch){
    if (motor >= CONFIG_MOTOR || ch >= CONFIG_DELAY1) return 0;
    __LocalDelay1 *self = &delay1_pool[motor][ch];
    self->data = &delay1_data_pool[motor][ch];
    return self;
}
#endif
#if CONFIG_DELAY2
static __LocalDelay2 delay2_pool[CONFIG_MOTOR][CONFIG_DELAY2];
static __LocalDelay2Data delay2_data_pool[CONFIG_MOTOR][CONFIG_DELAY2];
__LocalDelay2 *__local_delay2_get(uint8_t motor, int ch){
    if (motor >= CONFIG_MOTOR || ch >= CONFIG_DELAY2) return 0;
    __LocalDelay2 *self = &delay2_pool[motor][ch];
    self->data = &delay2_data_pool[motor][ch];
    return self;
}
#endif
#if CONFIG_DELAY3
static __LocalDelay3 delay3_pool[CONFIG_MOTOR][CONFIG_DELAY3];
static __LocalDelay3Data delay3_data_pool[CONFIG_MOTOR][CONFIG_DELAY3];
__LocalDelay3 *__local_delay3_get(uint8_t motor, int ch){
    if (motor >= CONFIG_MOTOR || ch >= CONFIG_DELAY3) return 0;
    __LocalDelay3 *self = &delay3_pool[motor][ch];
    self->data = &delay3_data_pool[motor][ch];
    return self;
}
#endif
// (分割线)------------------------------------------------------------------------
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

// ================================================================================
// 7.全局函数transfer初始化函数
// (初始化函数，拿指针)
void transfer_init_0(Transfer *transfer){

}

// (失能函数，清空指针)
void transfer_deinit_0(Transfer *transfer){

}

// (初始化函数，功能模块初始化)
void transfer_init_1(Transfer *transfer){

}

// (失能函数，功能模块情况内容)
void transfer_deinit_1(Transfer *transfer){

}



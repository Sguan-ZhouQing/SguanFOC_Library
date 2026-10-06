#ifndef D7BE3C00_F8F8_4C27_98FD_F3BE2E9738F9
#define D7BE3C00_F8F8_4C27_98FD_F3BE2E9738F9

#include "Sguan_Transfer.h"

void patch_transfer1_init(Transfer1 *transfer);
void patch_transfer2_init(Transfer2 *transfer);
void patch_transfer3_init(Transfer3 *transfer);
void patch_transfer4_init(Transfer4 *transfer);
void patch_transfer5_init(Transfer5 *transfer);
void patch_integrator_init(Integrator *integrator);
void patch_derivative_init(Derivative *derivative);
void patch_curve_init(Curve *curve);
void patch_dft_init(Dft *dft);
void patch_hall_init(Hall *hall);
void patch_ladrc1_init(Ladrc1 *ladrc);
void patch_ladrc2_init(Ladrc2 *ladrc);
void patch_smc_init(Smc *smc);
void patch_dpcc_init(Dpcc *dpcc);
void patch_pir_init(Pir *pir);
void patch_pid_init(Pid *pid);
void patch_pll_init(Pll *pll);
void patch_lpf1_init(Lpf1 *lpf);
void patch_lpf2_init(Lpf2 *lpf);
void patch_hpf1_init(Hpf1 *hpf);
void patch_hpf2_init(Hpf2 *hpf);
void patch_bpf1_init(Bpf1 *bpf);
void patch_bpf2_init(Bpf2 *bpf);
void patch_sogi_init(Sogi *sogi);
void patch_nf_init(Nf *nf);
void patch_tpnf_init(Tpnf *tpnf);
void patch_dob_init(Dob *dob);
void patch_rls_init(Rls *rls);
void patch_smo_init(Smo *smo);
void patch_nlfo_init(Nlfo *nlfo);
void patch_vcfo_init(Vcfo *vcfo);
void patch_hfi_init(Hfi *hfi);
void patch_rolo_init(Rolo *rolo);
void patch_mars_init(Mars *mars);
void patch_ekf_init(Ekf *ekf);
void patch_delay1_init(Delay1 *delay);
void patch_delay2_init(Delay2 *delay);
void patch_delay3_init(Delay3 *delay);

void patch_transfer1_loop(Transfer1 *transfer);
void patch_transfer2_loop(Transfer2 *transfer);
void patch_transfer3_loop(Transfer3 *transfer);
void patch_transfer4_loop(Transfer4 *transfer);
void patch_transfer5_loop(Transfer5 *transfer);
void patch_integrator_loop(Integrator *integrator);
void patch_derivative_loop(Derivative *derivative);
void patch_curve_loop(Curve *curve);
void patch_dft_loop(Dft *dft);
void patch_hall_loop(Hall *hall);
void patch_ladrc1_loop(Ladrc1 *ladrc);
void patch_ladrc2_loop(Ladrc2 *ladrc);
void patch_smc_loop(Smc *smc);
void patch_dpcc_loop(Dpcc *dpcc);
void patch_pir_loop(Pir *pir);
void patch_pid_loop(Pid *pid);
void patch_pll_loop(Pll *pll);
void patch_lpf1_loop(Lpf1 *lpf);
void patch_lpf2_loop(Lpf2 *lpf);
void patch_hpf1_loop(Hpf1 *hpf);
void patch_hpf2_loop(Hpf2 *hpf);
void patch_bpf1_loop(Bpf1 *bpf);
void patch_bpf2_loop(Bpf2 *bpf);
void patch_sogi_loop(Sogi *sogi);
void patch_nf_loop(Nf *nf);
void patch_tpnf_loop(Tpnf *tpnf);
void patch_dob_loop(Dob *dob);
void patch_rls_loop(Rls *rls);
void patch_smo_loop(Smo *smo);
void patch_nlfo_loop(Nlfo *nlfo);
void patch_vcfo_loop(Vcfo *vcfo);
void patch_hfi_loop(Hfi *hfi);
void patch_rolo_loop(Rolo *rolo);
void patch_mars_loop(Mars *mars);
void patch_ekf_loop(Ekf *ekf);
void patch_delay1_loop(Delay1 *delay);
void patch_delay2_loop(Delay2 *delay);
void patch_delay3_loop(Delay3 *delay);

void patch_sine_loop(Sine *sine);
void patch_cosine_loop(Cosine *cosine);
void patch_sincos_loop(SinCos *sincos);
void patch_tan_loop(Tan *tan);
void patch_atan_loop(Atan *atan);
void patch_limit_loop(Limit *limit);
void patch_sign_loop(Sign *sign);
void patch_clarke_loop(Clarke *clarke);
void patch_park_loop(Park *park);
void patch_ipark_loop(Ipark *ipark);
void patch_spwm0_loop(Spwm0 *spwm);
void patch_spwm_loop(Spwm *spwm);
void patch_svpwm_loop(Svpwm *svpwm);
void patch_swpwm_loop(Swpwm *swpwm);
void patch_singlers_loop(SingleRs *singlers);

void patch_reset_integrator(void *p);



#endif /* D7BE3C00_F8F8_4C27_98FD_F3BE2E9738F9 */

#ifndef __UNITLIB_PATCH_H
#define __UNITLIB_PATCH_H

#include "Sguan_Transfer.h"

void patch_transfer1_init(__LocalTransfer1 *transfer);
void patch_transfer2_init(__LocalTransfer2 *transfer);
void patch_transfer3_init(__LocalTransfer3 *transfer);
void patch_transfer4_init(__LocalTransfer4 *transfer);
void patch_transfer5_init(__LocalTransfer5 *transfer);
void patch_integrator_init(__LocalIntegrator *integrator);
void patch_derivative_init(__LocalDerivative *derivative);
void patch_curve_init(__LocalCurve *curve);
void patch_dft_init(__LocalDft *dft);
void patch_hall_init(__LocalHall *hall);
void patch_ladrc1_init(__LocalLadrc1 *ladrc);
void patch_ladrc2_init(__LocalLadrc2 *ladrc);
void patch_smc_init(__LocalSmc *smc);
void patch_dpcc_init(__LocalDpcc *dpcc);
void patch_pir_init(__LocalPir *pir);
void patch_pid_init(__LocalPid *pid);
void patch_pll_init(__LocalPll *pll);
void patch_lpf1_init(__LocalLpf1 *lpf);
void patch_lpf2_init(__LocalLpf2 *lpf);
void patch_hpf1_init(__LocalHpf1 *hpf);
void patch_hpf2_init(__LocalHpf2 *hpf);
void patch_bpf1_init(__LocalBpf1 *bpf);
void patch_bpf2_init(__LocalBpf2 *bpf);
void patch_sogi_init(__LocalSogi *sogi);
void patch_nf_init(__LocalNf *nf);
void patch_tpnf_init(__LocalTpnf *tpnf);
void patch_dob_init(__LocalDob *dob);
void patch_rls_init(__LocalRls *rls);
void patch_smo_init(__LocalSmo *smo);
void patch_nlfo_init(__LocalNlfo *nlfo);
void patch_vcfo_init(__LocalVcfo *vcfo);
void patch_hfi_init(__LocalHfi *hfi);
void patch_rolo_init(__LocalRolo *rolo);
void patch_mars_init(__LocalMars *mars);
void patch_ekf_init(__LocalEkf *ekf);

void patch_transfer1_loop(__LocalTransfer1 *transfer);
void patch_transfer2_loop(__LocalTransfer2 *transfer);
void patch_transfer3_loop(__LocalTransfer3 *transfer);
void patch_transfer4_loop(__LocalTransfer4 *transfer);
void patch_transfer5_loop(__LocalTransfer5 *transfer);
void patch_integrator_loop(__LocalIntegrator *integrator);
void patch_derivative_loop(__LocalDerivative *derivative);
void patch_curve_loop(__LocalCurve *curve);
void patch_dft_loop(__LocalDft *dft);
void patch_hall_loop(__LocalHall *hall);
void patch_ladrc1_loop(__LocalLadrc1 *ladrc);
void patch_ladrc2_loop(__LocalLadrc2 *ladrc);
void patch_smc_loop(__LocalSmc *smc);
void patch_dpcc_loop(__LocalDpcc *dpcc);
void patch_pir_loop(__LocalPir *pir);
void patch_pid_loop(__LocalPid *pid);
void patch_pll_loop(__LocalPll *pll);
void patch_lpf1_loop(__LocalLpf1 *lpf);
void patch_lpf2_loop(__LocalLpf2 *lpf);
void patch_hpf1_loop(__LocalHpf1 *hpf);
void patch_hpf2_loop(__LocalHpf2 *hpf);
void patch_bpf1_loop(__LocalBpf1 *bpf);
void patch_bpf2_loop(__LocalBpf2 *bpf);
void patch_sogi_loop(__LocalSogi *sogi);
void patch_nf_loop(__LocalNf *nf);
void patch_tpnf_loop(__LocalTpnf *tpnf);
void patch_dob_loop(__LocalDob *dob);
void patch_rls_loop(__LocalRls *rls);
void patch_smo_loop(__LocalSmo *smo);
void patch_nlfo_loop(__LocalNlfo *nlfo);
void patch_vcfo_loop(__LocalVcfo *vcfo);
void patch_hfi_loop(__LocalHfi *hfi);
void patch_rolo_loop(__LocalRolo *rolo);
void patch_mars_loop(__LocalMars *mars);
void patch_ekf_loop(__LocalEkf *ekf);


#endif // UNITLIB_PATCH_H

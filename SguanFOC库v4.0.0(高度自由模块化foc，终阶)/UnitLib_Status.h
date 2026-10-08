#ifndef __UNITLIB_STATUS_H
#define __UNITLIB_STATUS_H
/* SguanFOC配置文件声明 */
#include "SguanFOC.h"
#include "UnitLib_main.h"


void unitlib_status_standby_loop(void);
void unitlib_status_ready_loop(void);
void unitlib_status_initial_loop(void);
void unitlib_status_increasing_loop(void);
void unitlib_status_decreasing_loop(void);
void unitlib_status_hold_loop(void);
void unitlib_status_overvoltage_loop(void);
void unitlib_status_undervoltage_loop(void);
void unitlib_status_overcurrent_loop(void);
void unitlib_status_overtemp_motor_loop(void);
void unitlib_status_undertemp_motor_loop(void);
void unitlib_status_overtemp_driver_loop(void);
void unitlib_status_undertemp_driver_loop(void);
void unitlib_status_overtemp_pcb_loop(void);
void unitlib_status_undertemp_pcb_loop(void);
void unitlib_status_stuck_loop(void);
void unitlib_status_phase_loss_loop(void);
void unitlib_status_fault_loop(void);



#endif // UNITLIB_STATUS_H

#ifndef __UNITLIB_SITUATION_H
#define __UNITLIB_SITUATION_H

void situation_standby_loop(void);
void situation_ready_loop(void);
void situation_initial_loop(void);
void situation_increasing_loop(void);
void situation_decreasing_loop(void);
void situation_hold_loop(void);
void situation_overvoltage_loop(void);
void situation_undervoltage_loop(void);
void situation_overcurrent_loop(void);
void situation_overtemp_motor_loop(void);
void situation_undertemp_motor_loop(void);
void situation_overtemp_driver_loop(void);
void situation_undertemp_driver_loop(void);
void situation_overtemp_pcb_loop(void);
void situation_undertemp_pcb_loop(void);
void situation_stuck_loop(void);
void situation_phase_loss_loop(void);
void situation_fault_loop(void);


#endif // UNITLIB_SITUATION_H

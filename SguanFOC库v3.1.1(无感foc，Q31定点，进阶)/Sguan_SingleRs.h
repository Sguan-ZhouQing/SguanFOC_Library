#ifndef __SGUAN_SINGLERS_H
#define __SGUAN_SINGLERS_H

/* SguanFOC配置文件声明 */
#include "Sguan_Config.h"

extern Q31_t MOS_Duty;
extern Q31_t ADC_Duty;
extern Q31_t Min_Duty;

extern int16_t MOTOR_Ix[3];
extern Q31_t Duty_min;
extern Q31_t Duty_mid;
extern Q31_t Duty_max;
extern Q31_t Duty_n;
extern Q31_t Duty_m;
extern uint8_t ADC_Yes;

void SingleRs_ReadCurrent(uint16_t Raw_CH0, uint16_t Raw_CH1);
void SingleRs_END_Loop(void);


#endif // SGUAN_SINGLERS_H

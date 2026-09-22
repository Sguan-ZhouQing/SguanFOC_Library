#ifndef __UNITLIB_MAIN_H
#define __UNITLIB_MAIN_H
/* SguanFOC配置文件声明 */
#include "SguanFOC.h"


void main_standby_init(SguanFoc *sguan);
void main_ready_init(SguanFoc *sguan);
void main_initializing_init(SguanFoc *sguan);
void main_goinit0_angle_init(SguanFoc *sguan);
void main_goinit1_current_init(SguanFoc *sguan);

// =====================================================
void main_high_loop_one(SguanFoc *sguan);
void main_high_loop_two(SguanFoc *sguan);
void main_high_loop_three(SguanFoc *sguan);
void main_high_loop_four(SguanFoc *sguan);
void main_high_loop_five(SguanFoc *sguan);
void main_high_loop_six(SguanFoc *sguan);
// =====================================================
void main_high_loop(SguanFoc *sguan);
void main_low_loop(SguanFoc *sguan);



#endif // UNITLIB_MAIN_H

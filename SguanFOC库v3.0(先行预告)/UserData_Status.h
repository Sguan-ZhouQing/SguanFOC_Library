#ifndef __USERDATA_STATUS_H
#define __USERDATA_STATUS_H
#include <stdint.h>
/* 电机控制User用户设置·状态管理 */
/* 用户自己的CODE BEGIN Includes */

/* 用户自己的CODE END Includes */

/* ================= 状态机任务信号(输入) ================= */
static inline uint8_t User_StatusSTOP_Signal(void){
    uint8_t STOP_num;
    /* 急停信号(Emergency stop signal) */
    /* 输出0->正常运行 输出1->启用急停 */
    return STOP_num;
}


/* ================= 状态机任务处理(执行) ================= */
static inline void User_StatusSTOP_Loop(void){
    
}


#endif // USERDATA_STATUS_H

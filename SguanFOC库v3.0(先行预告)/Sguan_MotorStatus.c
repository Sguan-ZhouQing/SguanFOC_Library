/*
 * @Author: 星必尘Sguan
 * @GitHub: https://github.com/Sguan-ZhouQing
 * @Date: 2026-01-26 22:43:42
 * @LastEditors: 星必尘Sguan|3464647102@qq.com
 * @LastEditTime: 2026-02-02 18:42:29
 * @FilePath: \demo_SguanFOCCode\SguanFOC库\Sguan_MotorStatus.c
 * @Description: SguanFOC库的“电机状态机”实现
 * 
 * Copyright (c) 2026 by $星必尘Sguan, All Rights Reserved. 
 */
#include "Sguan_MotorStatus.h"

/* 外部用户设置函数声明 */
#include "UserData_Status.h"

void MotorStatus_Loop(MOTOR_STATUS_ENUM *status){
    switch (*status){
    case MOTOR_STATUS_UNINITIALIZED:
        
        break;
    case MOTOR_STATUS_INITIALIZING:
        
        break;
    case MOTOR_STATUS_CALIBRATING:
        
        break;
    case MOTOR_STATUS_OVERVOLTAGE:
        
        break;
    case MOTOR_STATUS_UNDERVOLTAGE:
        
        break;
    case MOTOR_STATUS_OVERTEMPERATURE:
        
        break;
    case MOTOR_STATUS_UNDERTEMPERATURE:
        
        break;
    case MOTOR_STATUS_OVERCURRENT:
        
        break;
    case MOTOR_STATUS_EMERGENCY_STOP:
        User_StatusSTOP_Loop();
        break;
    default:
        break;
    }
    
}


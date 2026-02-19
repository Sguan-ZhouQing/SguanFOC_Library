/*
 * @Author: 星必尘Sguan
 * @GitHub: https://github.com/Sguan-ZhouQing
 * @Date: 2026-01-27 00:07:53
 * @LastEditors: 星必尘Sguan|3464647102@qq.com
 * @LastEditTime: 2026-02-19 22:52:41
 * @FilePath: \stm_SguanFOCtest\SguanFOC\Sguan_printf.c
 * @Description: SguanFOC库的“JustFloat通讯协议”实现
 * 
 * Copyright (c) 2026 by $星必尘Sguan, All Rights Reserved. 
 */
#include "Sguan_printf.h"
/* 外部函数文件声明 */
#include "UserData_Correspond.h"
#include "UserData_UserControl.h"
#include <string.h>
/* 内部函数文件声明 */
static float Get_Data(void);
uint8_t Sguan_PrintfBuff[200];

/* ================= 重定向设计 BEGIN ================= */
// 支持printf函数，而无需选择MicroLIB
#if 1
#pragma import(__use_no_semihosting)
//标准库需要的支持函数
struct __FILE{ 
	int handle; 
}; 
// 支持printf函数，而不需要选择MicroLIB
FILE __stdout;
//定义_sys_exit避免使用半主机模式
void _sys_exit(int x){ 
	x = x; 
} 
#endif

//串口重定向函数printf，不使用到MicroLIB
int fputc(int ch,FILE *f){
	User_CorrespondSet((uint8_t *)&ch,1);
	return ch;
}
/* ================= 重定向设计 END ================= */


// [接收]数据解析函数(格式：AO=13.14?)
static float Get_Data(void){
    uint8_t data_Start_Num = 0;
    uint8_t data_End_Num = 0;
    uint8_t minus_Flag = 0;
    float data_return = 0;
    // 查找等号和问号的位置
    for(uint8_t i = 0; i < 200; i++){
        if(Sguan_PrintfBuff[i] == '=') data_Start_Num = i + 1;
        if(Sguan_PrintfBuff[i] == '?'){
            data_End_Num = i - 1;
            break;
        }
    }
    if(Sguan_PrintfBuff[data_Start_Num] == '-'){
        data_Start_Num += 1;
        minus_Flag = 1;
    }
    
    data_return = 0;
    uint8_t decimal_point = 0;
    uint8_t decimal_digits = 0;  // 记录小数位数
    // 先找到小数点的位置
    for(uint8_t i = data_Start_Num; i <= data_End_Num; i++){
        if(Sguan_PrintfBuff[i] == '.'){
            decimal_point = i;
            break;
        }
    }
    
    // 处理整数部分和小数部分
    for(uint8_t i = data_Start_Num; i <= data_End_Num; i++){
        if(Sguan_PrintfBuff[i] == '.'){
            continue;  // 跳过小数点
        }
        if(i < decimal_point || decimal_point == 0){
            // 整数部分
            data_return = data_return * 10 + (Sguan_PrintfBuff[i] - '0');
        } else{
            // 小数部分：根据小数位的位置计算权重
            decimal_digits++;
            float decimal_weight = 1.0f;
            for(uint8_t j = 0; j < decimal_digits; j++){
                decimal_weight *= 0.1f;
            }
            data_return += (Sguan_PrintfBuff[i] - '0') * decimal_weight;
        }
    }
    if(minus_Flag == 1) data_return = -data_return;
    return data_return;
}

// [初始化]初始化JustFloat数据帧尾
void Printf_Init(PRINTF_STRUCT *str){
    str->tail[0] = 0x00;
    str->tail[1] = 0x00;
    str->tail[2] = 0x80;
    str->tail[3] = 0x7f;
    /* JustFloa数据帧尾格式 */
}

// [发送]发送数据Tick函数，发送周期可自定
void Printf_Loop(PRINTF_STRUCT *str){
    // 发送JustFloat数据
    User_CorrespondSet((uint8_t *)str,sizeof(PRINTF_STRUCT));
}

// [接收]实时参数调整函数（需要根据你的实际结构体定义进行调整）
void Printf_Adjust(void){
    float data_Get = Get_Data();
    if(Sguan_PrintfBuff[0]=='A' && Sguan_PrintfBuff[1]=='O'){
        User_AO_Adjust(data_Get);
    }
    if(Sguan_PrintfBuff[0]=='B' && Sguan_PrintfBuff[1]=='O'){
        User_BO_Adjust(data_Get);
    }
    if(Sguan_PrintfBuff[0]=='C' && Sguan_PrintfBuff[1]=='O'){
        User_CO_Adjust(data_Get);
    }
    memset(Sguan_PrintfBuff, 0, sizeof(Sguan_PrintfBuff));
}



/*
 * @Author: 星必尘Sguan
 * @GitHub: https://github.com/Sguan-ZhouQing
 * @Date: 2026-01-27 00:07:53
 * @LastEditors: 星必尘Sguan|3464647102@qq.com
 * @LastEditTime: 2026-01-30 14:49:11
 * @FilePath: \demo_SguanFOCCode\SguanFOC库\Sguan_printf.c
 * @Description: SguanFOC库的“JustFloat通讯协议”实现
 * 
 * Copyright (c) 2026 by $星必尘Sguan, All Rights Reserved. 
 */
#include "Sguan_printf.h"
/* 外部函数文件声明 */
#include "UserData_Calculate.h"
#include "SguanFOC.h"
/* 内部函数文件声明 */
static void Printf_Init(PRINTF_STRUCT *str);

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
	User_PrintfSet((uint8_t *)&ch);
	return ch;
}

// 初始化JustFloat数据帧尾
static void Printf_Init(PRINTF_STRUCT *str){
    str->tail[0] = 0x00;
    str->tail[1] = 0x00;
    str->tail[2] = 0x80;
    str->tail[3] = 0x7f;
    /* JustFloa数据帧尾格式 */
}

// 发送数据Tick函数，发送周期可自定
void Printf_Loop(PRINTF_STRUCT *str){
    // 数据帧尾定义
    Printf_Init(str);
    // 发送JustFloat数据
    User_PrintfSet((uint8_t *)str);
}


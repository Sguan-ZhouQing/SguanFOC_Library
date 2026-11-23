/* ===================================================================
 * 文件:     Sguan_printf.c
 * 项目:     SguanFOC_v3.1.0 —— 开源 FOC 电机控制框架（无感FOC · 浮点运算 · 进阶）
 * 链接:     https://github.com/Sguan-ZhouQing/SguanFOC_Library
 * 版权:     Copyright 2026 星必尘Sguan <3464647102@qq.com>
 * 许可证:   MIT License
 * SPDX:     SPDX-License-Identifier: MIT
 *
 * 许可说明:
 *   [EN] Permission is hereby granted, free of charge, to use, copy,
 *        modify, merge, publish, distribute, sublicense, and/or sell
 *        copies of this software, provided this notice is retained.
 *   [中文] 本软件依据 MIT 许可证授权：可免费使用、复制、修改、合并、
 *        出版、分发、再授权及销售，仅须保留本版权与许可声明。
 *   [DE] Hiermit wird jeder Person kostenlos erlaubt, diese Software zu
 *        nutzen, zu kopieren, zu veraendern, zu veroeffentlichen, zu
 *        vertreiben, zu unterlizenzieren und/oder zu verkaufen, sofern
 *        dieser Hinweis erhalten bleibt.
 *   [JA] 本ソフトウェアは MIT ライセンスに基づき、無償で使用・複製・
 *        改変・公開・配布・再許諾・販売が許可されます。ただし本表示を
 *        保持する必要があります。
 * 完整许可文本: 参见仓库根目录下的 LICENSE-MIT 文件。
 * =================================================================== */
#include "SguanFOC.h"

/* UserData外部文件声明 */
#include "UserData_Function.h"
#include "UserData_UserControl.h"
#include <stdio.h>
#include <string.h>

/* 内部私有变量与函数 */
uint8_t Sguan_PrintfBuff[200];
static float Get_Data_Fast(uint8_t start_idx);
static void Printf_Adjust(void);
static void Handle_MOTOR(float value);
static void Handle_Speed(float value);
static void Handle_Position(float value);
static void Handle_Id(float value);
static void Handle_Iq(float value);
static void Handle_Ud(float value);
static void Handle_Uq(float value);

/* ==================== 重定向设计 =================== */
// 支持printf函数，而无需选择MicroLIB
#if 1
#pragma import(__use_no_semihosting)
struct __FILE { int handle; }; 
FILE __stdout;
void _sys_exit(int x) { x = x; } 

int fputc(int ch, FILE *f) {
    User_CorrespondSet((uint8_t *)&ch, 1);
    return ch;
}
#endif


/* ==================== 哈希/字典映射实现 =================== */
// 轻量级字符串哈希函数 (DJB2算法)
static uint32_t Hash_Compute(const char *str, uint8_t len){
    uint32_t hash = 5381;
    for (uint8_t i = 0; i < len; i++) {
        hash = ((hash << 5) + hash) + str[i]; 
    }
    return hash;
}

// 指令处理回调函数库->MOTOR
static void Handle_MOTOR(float value){
    if (value >= 0.5f) Sguan.Func_Start();
    else Sguan.Func_Stop();
    // MOTOR=6.88?
    // motor = value;
}

// 指令处理回调函数库->Speed
static void Handle_Speed(float value){
    Sguan.Func_Set_Velocity(value);
    // Speed=50.0?
    // if(motor <= 1){
    //     CAN_TX(0x11,SPEED,value);
    // }
    // else if(motor <= 2){
    //     CAN_TX(0x22,SPEED,value);
    // }
    // else if(motor <= 3){
    //     CAN_TX(0x33,SPEED,value);
    // }
}

// 指令处理回调函数库->Position
static void Handle_Position(float value){
    Sguan.Func_Set_Position(value);
    // Position=50.0?

    // CAN_TX(id,POSITION,value);
}

// 指令处理回调函数库->Id
static void Handle_Id(float value){
    Sguan.Func_Set_Id(value);
}

// 指令处理回调函数库->Iq
static void Handle_Iq(float value){
    Sguan.Func_Set_Iq(value);
}

// 指令处理回调函数库->Ud
static void Handle_Ud(float value){
    Sguan.Func_Set_Ud(value);
}

// 指令处理回调函数库->Uq
static void Handle_Uq(float value){
    Sguan.Func_Set_Uq(value);
}


// 动态可修改的指令字典（预留 hash_val 位置为 0）
CMD_MAP_STRUCT Cmd_Dict[] = {
    {"MOTOR",       0, Handle_MOTOR},
    {"Speed",       0, Handle_Speed},
    {"Position",    0, Handle_Position},
    {"Id",          0, Handle_Id},
    {"Iq",          0, Handle_Iq},
    {"Ud",          0, Handle_Ud},
    {"Uq",          0, Handle_Uq},
    {"User0",       0, Handle_User0_Adjust},
    {"User1",       0, Handle_User0_Adjust},
    {"User2",       0, Handle_User0_Adjust}
};

#define DICT_COUNT (sizeof(Cmd_Dict) / sizeof(CMD_MAP_STRUCT))

/**
 * @description: [接收]实时参数调整函数 - 真·哈希查表
 */
static void Printf_Adjust(void){
    uint8_t eq_pos = 0;
    
    // 1. 寻找等号，确定指令 Key 的长度
    while (Sguan_PrintfBuff[eq_pos] != '=' && eq_pos < 50){
        if (Sguan_PrintfBuff[eq_pos] == '\0') return;
        eq_pos++;
    }

    // 2. 提取数值
    float data_val = Get_Data_Fast(eq_pos + 1);

    // 3. 计算当前收到指令的哈希值
    uint32_t rx_hash = Hash_Compute((const char*)Sguan_PrintfBuff, eq_pos);

    // 4. 哈希查表匹配 (纯整数比较，速度极快)
    for (uint8_t i = 0; i < DICT_COUNT; i++){
        if (Cmd_Dict[i].hash_val == rx_hash){
            // 严谨起见：发生哈希碰撞时，二次确认字符串内容
            if (strncmp((char*)Sguan_PrintfBuff, Cmd_Dict[i].name, eq_pos) == 0){
                if (Cmd_Dict[i].callback != NULL) {
                    Cmd_Dict[i].callback(data_val);
                }
                break;
            }
        }
    }

    // 5. 清理缓冲区
    memset(Sguan_PrintfBuff, 0, sizeof(Sguan_PrintfBuff));
}

/**
 * @description: 极速浮点解析 (单遍扫描 + 异常数据熔断截断)
 */
static float Get_Data_Fast(uint8_t start_idx){
    float result = 0.0f;
    float frac_weight = 0.1f;
    uint8_t i = start_idx;
    int8_t sign = 1;
    uint8_t in_fraction = 0;

    if (Sguan_PrintfBuff[i] == '-'){
        sign = -1;
        i++;
    }

    while (Sguan_PrintfBuff[i] != '?' && Sguan_PrintfBuff[i] != '\0' && i < 200){
        uint8_t c = Sguan_PrintfBuff[i];
        
        if (c == '.') {
            in_fraction = 1;
        } else if (c >= '0' && c <= '9'){
            if (!in_fraction) {
                result = result * 10.0f + (c - '0');
            } else {
                result += (c - '0') * frac_weight;
                frac_weight *= 0.1f;
            }
        } else {
            // 遇到非数字/小数点的异常字符(如粘包带来的字母)，立刻截断
            break; 
        }
        i++;
    }
    return result * (float)sign;
}

/* ==================== 通用 Loop 函数 =================== */

void Printf_TX_Init(PRINTF_STRUCT *str){
    str->tail[0] = 0x00;
    str->tail[1] = 0x00;
    str->tail[2] = 0x80;
    str->tail[3] = 0x7f;
}

void Printf_TX_Loop(PRINTF_STRUCT *str){
    User_UserTX();
    User_CorrespondSet((uint8_t *)str, sizeof(PRINTF_STRUCT));
}

void Printf_RX_Init(void){
    for (uint8_t i = 0; i < DICT_COUNT; i++){
        uint8_t len = strlen(Cmd_Dict[i].name);
        Cmd_Dict[i].hash_val = Hash_Compute(Cmd_Dict[i].name, len);
    }
}

void Printf_RX_Loop(uint8_t *data, uint16_t length){
    static uint16_t buff_ptr = 0;
    
    for (uint16_t i = 0; i < length; i++){
        uint8_t ch = data[i];

        // 1. 过滤：无视掉换行、回车、空格等无意义字符，防干扰
        if (ch == '\r' || ch == '\n' || ch == ' '){
            continue;
        }
        
        Sguan_PrintfBuff[buff_ptr++] = ch;
        
        // 2. 遇到帧尾 '?' 触发解析
        if (ch == '?') {
            Printf_Adjust();
            buff_ptr = 0; // 重置索引
            continue;
        }
        
        // 3. 熔断保护（防粘包 / 防死滞溢出）
        // 如果积累了 40 个字节还没见到 '?'，直接丢弃清空，防止系统死机
        if (buff_ptr >= 40) {
            memset(Sguan_PrintfBuff, 0, sizeof(Sguan_PrintfBuff));
            buff_ptr = 0;
        }
    }
}

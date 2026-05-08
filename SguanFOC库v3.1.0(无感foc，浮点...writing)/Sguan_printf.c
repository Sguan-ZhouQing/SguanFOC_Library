/*
 * @Author: 星必尘Sguan
 * @Description: SguanFOC库的“JustFloat通讯协议”实现 (终极优化版)
 * 包含特性：真·哈希字典极速映射、防乱码粘包熔断、O(n)单遍解析
 */
#include "Sguan_printf.h"
#include "UserData_Function.h"
#include "UserData_UserControl.h"
#include <stdio.h>
#include <string.h>

/* 内部私有变量与函数 */
uint8_t Sguan_PrintfBuff[200];
static float Get_Data_Fast(uint8_t start_idx);
static void Printf_Adjust(void);

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

/**
 * @description: 轻量级字符串哈希函数 (DJB2算法)
 */
static uint32_t Hash_Compute(const char *str, uint8_t len) {
    uint32_t hash = 5381;
    for (uint8_t i = 0; i < len; i++) {
        hash = ((hash << 5) + hash) + str[i]; 
    }
    return hash;
}

/* 指令处理回调函数库 (在这里根据实际需求添加你的函数) */
static void Handle_Speed(float v)       { /* 示例：User_Speed_Adjust(v); */ }
static void Handle_Pos(float v)         { /* 示例：User_Pos_Adjust(v);   */ }
static void Handle_Target_IF_Iq(float v){ /* 示例：User_SetIF(v);        */ }
static void Handle_Target_Iq(float v)   { /* 示例：User_SetIq(v);        */ }

// 动态可修改的指令字典（预留 hash_val 位置为 0）
CMD_MAP_STRUCT Cmd_Dict[] = {
    {"Speed",        0, Handle_Speed},
    {"Pos",          0, Handle_Pos},
    {"Target_IF_Iq", 0, Handle_Target_IF_Iq},
    {"Target_Iq",    0, Handle_Target_Iq},
    {"AO",           0, User_AO_Adjust},
    {"BO",           0, User_BO_Adjust},
    {"CO",           0, User_CO_Adjust}
};

#define DICT_COUNT (sizeof(Cmd_Dict) / sizeof(CMD_MAP_STRUCT))

/**
 * @description: 初始化函数：预计算所有指令的哈希值
 * @note: 必须在 main() 的 while(1) 前调用一次！
 */
void Printf_Hash_Init(void) {
    for (uint8_t i = 0; i < DICT_COUNT; i++) {
        uint8_t len = strlen(Cmd_Dict[i].name);
        Cmd_Dict[i].hash_val = Hash_Compute(Cmd_Dict[i].name, len);
    }
}

/**
 * @description: [接收]实时参数调整函数 - 真·哈希查表
 */
static void Printf_Adjust(void) {
    uint8_t eq_pos = 0;
    
    // 1. 寻找等号，确定指令 Key 的长度
    while (Sguan_PrintfBuff[eq_pos] != '=' && eq_pos < 50) {
        if (Sguan_PrintfBuff[eq_pos] == '\0') return;
        eq_pos++;
    }

    // 2. 提取数值
    float data_val = Get_Data_Fast(eq_pos + 1);

    // 3. 计算当前收到指令的哈希值
    uint32_t rx_hash = Hash_Compute((const char*)Sguan_PrintfBuff, eq_pos);

    // 4. 哈希查表匹配 (纯整数比较，速度极快)
    for (uint8_t i = 0; i < DICT_COUNT; i++) {
        if (Cmd_Dict[i].hash_val == rx_hash) {
            // 严谨起见：发生哈希碰撞时，二次确认字符串内容
            if (strncmp((char*)Sguan_PrintfBuff, Cmd_Dict[i].name, eq_pos) == 0) {
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
static float Get_Data_Fast(uint8_t start_idx) {
    float result = 0.0f;
    float frac_weight = 0.1f;
    uint8_t i = start_idx;
    int8_t sign = 1;
    uint8_t in_fraction = 0;

    if (Sguan_PrintfBuff[i] == '-') {
        sign = -1;
        i++;
    }

    while (Sguan_PrintfBuff[i] != '?' && Sguan_PrintfBuff[i] != '\0' && i < 200) {
        uint8_t c = Sguan_PrintfBuff[i];
        
        if (c == '.') {
            in_fraction = 1;
        } else if (c >= '0' && c <= '9') {
            if (!in_fraction) {
                result = result * 10.0f + (c - '0');
            } else {
                result += (c - '0') * frac_weight;
                frac_weight *= 0.1f;
            }
        } else {
            // 【防线】遇到非数字/小数点的异常字符(如粘包带来的字母)，立刻截断
            break; 
        }
        i++;
    }
    return result * (float)sign;
}

/* ==================== 通用 Loop 函数 =================== */

void Printf_TX_Init(PRINTF_STRUCT *str) {
    str->tail[0] = 0x00;
    str->tail[1] = 0x00;
    str->tail[2] = 0x80;
    str->tail[3] = 0x7f;
}

void Printf_TX_Loop(PRINTF_STRUCT *str) {
    User_UserTX();
    User_CorrespondSet((uint8_t *)str, sizeof(PRINTF_STRUCT));
}

void Printf_RX_Loop(uint8_t *data, uint16_t length) {
    static uint16_t buff_ptr = 0;
    
    for (uint16_t i = 0; i < length; i++) {
        uint8_t ch = data[i];

        // 1. 过滤：无视掉换行、回车、空格等无意义字符，防干扰
        if (ch == '\r' || ch == '\n' || ch == ' ') {
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

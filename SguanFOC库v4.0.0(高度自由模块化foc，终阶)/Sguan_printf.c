#include "Sguan_Printf.h"

/* UserData外部文件声明 */
#include "UserData_Driver.h"
#include "UserData_UserControl.h"
#include <stdio.h>
#include <string.h>

/* 内部私有变量与函数 */
uint8_t sguan_printfBuff[200];
static float Get_Data_Fast(uint8_t start_idx);
static void Printf_Adjust(void);
static void handle_motor(float value);
static void handle_speed(float value);
static void handle_position(float value);
static void handle_id(float value);
static void handle_iq(float value);
static void handle_ud(float value);
static void handle_uq(float value);


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
static void handle_motor(float value){

}

// 指令处理回调函数库->Speed
static void handle_speed(float value){

}

// 指令处理回调函数库->Position
static void handle_position(float value){

}

// 指令处理回调函数库->Id
static void handle_id(float value){

}

// 指令处理回调函数库->Iq
static void handle_iq(float value){

}

// 指令处理回调函数库->Ud
static void handle_ud(float value){

}

// 指令处理回调函数库->Uq
static void handle_uq(float value){

}


// 动态可修改的指令字典（预留 hash_val 位置为 0）
CmdMap cmd_dict[] = {
    {"MOTOR",       0, handle_motor},
    {"Speed",       0, handle_speed},
    {"Position",    0, handle_position},
    {"Id",          0, handle_id},
    {"Iq",          0, handle_iq},
    {"Ud",          0, handle_ud},
    {"Uq",          0, handle_uq}
};

#define DICT_COUNT (sizeof(cmd_dict) / sizeof(CmdMap))

/**
 * @description: [接收]实时参数调整函数 - 真·哈希查表
 */
static void Printf_Adjust(void){
    uint8_t eq_pos = 0;
    
    // 1. 寻找等号，确定指令 Key 的长度
    while (sguan_printfBuff[eq_pos] != '=' && eq_pos < 50){
        if (sguan_printfBuff[eq_pos] == '\0') return;
        eq_pos++;
    }

    // 2. 提取数值
    float data_val = Get_Data_Fast(eq_pos + 1);

    // 3. 计算当前收到指令的哈希值
    uint32_t rx_hash = Hash_Compute((const char*)sguan_printfBuff, eq_pos);

    // 4. 哈希查表匹配 (纯整数比较，速度极快)
    for (uint8_t i = 0; i < DICT_COUNT; i++){
        if (cmd_dict[i].hash_val == rx_hash){
            // 严谨起见：发生哈希碰撞时，二次确认字符串内容
            if (strncmp((char*)sguan_printfBuff, cmd_dict[i].name, eq_pos) == 0){
                if (cmd_dict[i].callback != NULL) {
                    cmd_dict[i].callback(data_val);
                }
                break;
            }
        }
    }

    // 5. 清理缓冲区
    memset(sguan_printfBuff, 0, sizeof(sguan_printfBuff));
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

    if (sguan_printfBuff[i] == '-'){
        sign = -1;
        i++;
    }

    while (sguan_printfBuff[i] != '?' && sguan_printfBuff[i] != '\0' && i < 200){
        uint8_t c = sguan_printfBuff[i];
        
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

void Printf_TX_Init(Printf *str){
    str->tail[0] = 0x00;
    str->tail[1] = 0x00;
    str->tail[2] = 0x80;
    str->tail[3] = 0x7f;
}

void Printf_TX_Loop(Printf *str){
    // User_UserTX();
    // User_CorrespondSet((uint8_t *)str, sizeof(Printf));
}

void Printf_RX_Init(void){
    for (uint8_t i = 0; i < DICT_COUNT; i++){
        uint8_t len = strlen(cmd_dict[i].name);
        cmd_dict[i].hash_val = Hash_Compute(cmd_dict[i].name, len);
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
        
        sguan_printfBuff[buff_ptr++] = ch;
        
        // 2. 遇到帧尾 '?' 触发解析
        if (ch == '?') {
            Printf_Adjust();
            buff_ptr = 0; // 重置索引
            continue;
        }
        
        // 3. 熔断保护（防粘包 / 防死滞溢出）
        // 如果积累了 40 个字节还没见到 '?'，直接丢弃清空，防止系统死机
        if (buff_ptr >= 40) {
            memset(sguan_printfBuff, 0, sizeof(sguan_printfBuff));
            buff_ptr = 0;
        }
    }
}

#include "Sguan_Printf.h"

/* SguanFOC配置文件声明 */
#include "UserData_Driver.h"
#include "UnitLib_UserControl.h"
#include <stdio.h>
#include <string.h>

/* 内部私有变量与函数 */
uint8_t sguan_printfbuff[64];
static uint32_t hash_compute(const char *str, uint8_t len);
static float get_data_fast(uint8_t start_idx);
static void printf_adjust(void);


/* ==================== 哈希/字典映射实现 =================== */
// 轻量级字符串哈希函数 (DJB2算法)
static uint32_t hash_compute(const char *str, uint8_t len){
    uint32_t hash = 5381;
    for (uint8_t i = 0; i < len; i++) {
        hash = ((hash << 5) + hash) + str[i]; 
    }
    return hash;
}

#define DICT_COUNT (sizeof(cmd_dict) / sizeof(CmdMap))

/**
 * @description: 极速浮点解析 (单遍扫描 + 异常数据熔断截断)
 */
static float get_data_fast(uint8_t start_idx){
    float result = 0.0f;
    float frac_weight = 0.1f;
    uint8_t i = start_idx;
    int8_t sign = 1;
    uint8_t in_fraction = 0;

    if (sguan_printfbuff[i] == '-'){
        sign = -1;
        i++;
    }

    while (sguan_printfbuff[i] != '?' && sguan_printfbuff[i] != '\0' && i < 200){
        uint8_t c = sguan_printfbuff[i];
        
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

/**
 * @description: [接收]实时参数调整函数 - 真·哈希查表
 */
static void printf_adjust(void){
    uint8_t eq_pos = 0;
    
    // 1. 寻找等号，确定指令 Key 的长度
    while (sguan_printfbuff[eq_pos] != '=' && eq_pos < 50){
        if (sguan_printfbuff[eq_pos] == '\0') return;
        eq_pos++;
    }

    // 2. 提取数值
    float data_val = get_data_fast(eq_pos + 1);

    // 3. 计算当前收到指令的哈希值
    uint32_t rx_hash = hash_compute((const char*)sguan_printfbuff, eq_pos);

    // 4. 哈希查表匹配 (纯整数比较，速度极快)
    for (uint8_t i = 0; i < DICT_COUNT; i++){
        if (cmd_dict[i].hash_val == rx_hash){
            // 严谨起见：发生哈希碰撞时，二次确认字符串内容
            if (strncmp((char*)sguan_printfbuff, cmd_dict[i].name, eq_pos) == 0){
                if (cmd_dict[i].callback != NULL) {
                    cmd_dict[i].callback(data_val);
                }
                break;
            }
        }
    }

    // 5. 清理缓冲区
    memset(sguan_printfbuff, 0, sizeof(sguan_printfbuff));
}

/* ==================== 通用 Loop 函数 =================== */
void printf_tx_init(Printf *str){
    str->tail[0] = 0x00;
    str->tail[1] = 0x00;
    str->tail[2] = 0x80;
    str->tail[3] = 0x7f;
}

void printf_tx_loop(Printf *str){
    // User_UserTX();
    // User_CorrespondSet((uint8_t *)str, sizeof(Printf));
}

void printf_rx_init(void){
    for (uint8_t i = 0; i < DICT_COUNT; i++){
        uint8_t len = strlen(cmd_dict[i].name);
        cmd_dict[i].hash_val = hash_compute(cmd_dict[i].name, len);
    }
}

void printf_rx_loop(uint8_t *data, uint16_t length){
    static uint16_t buff_ptr = 0;
    
    for (uint16_t i = 0; i < length; i++){
        uint8_t ch = data[i];

        // 1. 过滤：无视掉换行、回车、空格等无意义字符，防干扰
        if (ch == '\r' || ch == '\n' || ch == ' '){
            continue;
        }
        
        sguan_printfbuff[buff_ptr++] = ch;
        
        // 2. 遇到帧尾 '?' 触发解析
        if (ch == '?') {
            printf_adjust();
            buff_ptr = 0; // 重置索引
            continue;
        }
        
        // 3. 熔断保护（防粘包 / 防死滞溢出）
        // 如果积累了 40 个字节还没见到 '?'，直接丢弃清空，防止系统死机
        if (buff_ptr >= 40) {
            memset(sguan_printfbuff, 0, sizeof(sguan_printfbuff));
            buff_ptr = 0;
        }
    }
}

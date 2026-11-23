/* ===================================================================
 * 文件:     Sguan_Printf.c
 * 项目:     SguanFOC_v4 —— 开源 FOC 电机控制框架
 * 链接:     https://github.com/Sguan-ZhouQing/SguanFOC_Library
 * 版权:     Copyright 2026 星必尘Sguan <3464647102@qq.com>
 * 许可证:   Apache License, Version 2.0
 * SPDX:     SPDX-License-Identifier: Apache-2.0
 *
 * 许可说明:
 *   [EN] Licensed under the Apache License, Version 2.0. You may use,
 *        copy, modify, and distribute this software, including for
 *        commercial purposes, provided you retain this copyright
 *        notice and the terms of the License.
 *   [中文] 本软件依据 Apache License 2.0 授权：可自由使用、复制、修改、
 *        分发（含商业用途），但必须保留本版权声明与许可证条款。
 *   [DE] Lizenziert unter der Apache License, Version 2.0. Nutzung,
 *        Vervielfaeltigung, Aenderung und Weitergabe (auch kommerziell)
 *        sind erlaubt, sofern dieser Urheberrechtshinweis und die
 *        Lizenzbedingungen erhalten bleiben.
 *   [JA] Apache License, Version 2.0 に基づき許諾されます。使用・複製・
 *        改変・再配布（商用を含む）が可能ですが、本著作権表示と
 *        ライセンス条項を保持する必要があります。
 * 完整许可文本: 参见本目录下的 LICENSE 文件。
 * =================================================================== */
#include "Sguan_Printf.h"

/* SguanFOC配置文件声明 */
#include "UnitLib_UserControl.h"
#include "Zone_Driver.h"

// ================================================================================
// 1.内部私有量与函数
uint8_t sguan_printfbuff[64];
#define __LOCAL_DICT_COUNT (sizeof(cmd_dict) / sizeof(__LocalMap))
static uint32_t __local_hash_compute(const char *str, uint8_t len);
static float __local_get_data(uint8_t start_idx);
static void __local_adjust(void);

// ================================================================================
// 2.局部函数__local基础
// (轻量级字符串哈希DJB2)
static uint32_t __local_hash_compute(const char *str, uint8_t len){
    uint32_t hash = 5381;
    for (uint8_t i = 0; i < len; i++){
        hash = ((hash << 5) + hash) + str[i]; 
    }
    return hash;
}

// (极速浮点解析)
static float __local_get_data(uint8_t start_idx){
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
        if (c == '.'){
            in_fraction = 1;
        } else if (c >= '0' && c <= '9'){
            if (!in_fraction){
                result = result * 10.0f + (c - '0');
            } else{
                result += (c - '0') * frac_weight;
                frac_weight *= 0.1f;
            }
        } else{
            // 遇到非数字or小数点的异常字符(如粘包带来的字母)截断
            break; 
        }
        i++;
    }
    return result * (float)sign;
}

// (实时参数调整函数)
static void __local_adjust(void){
    uint8_t eq_pos = 0;
    while (sguan_printfbuff[eq_pos] != '=' && eq_pos < 50){
        if (sguan_printfbuff[eq_pos] == '\0') return;
        eq_pos++;
    }

    float data_val = __local_get_data(eq_pos + 1);
    uint32_t rx_hash = __local_hash_compute((const char*)sguan_printfbuff, eq_pos);

    for (uint8_t i = 0; i < __LOCAL_DICT_COUNT; i++){
        if (cmd_dict[i].hash_val == rx_hash){
            // 严谨起见：发生哈希碰撞时，二次确认字符串内容
            if (strncmp((char*)sguan_printfbuff, cmd_dict[i].name, eq_pos) == 0){
                if (cmd_dict[i].callback != NULL){
                    cmd_dict[i].callback(data_val);
                }
                break;
            }
        }
    }

    math_memset(sguan_printfbuff, 0, sizeof(sguan_printfbuff));
}

// ================================================================================
// 3.全局函数printf基础
// (printf发送初始化)
void printf_tx_init(Printf *str){
    str->tail[0] = 0x00;
    str->tail[1] = 0x00;
    str->tail[2] = 0x80;
    str->tail[3] = 0x7f;
}

// (printf发送运行)
void printf_tx_loop(Printf *str){
    // User_UserTX();
    // User_CorrespondSet((uint8_t *)str, sizeof(Printf));
}

// (printf接收初始化)
void printf_rx_init(void){
    for (uint8_t i = 0; i < __LOCAL_DICT_COUNT; i++){
        uint8_t len = strlen(cmd_dict[i].name);
        cmd_dict[i].hash_val = __local_hash_compute(cmd_dict[i].name, len);
    }
}

// (printf接收运行)
void printf_rx_loop(uint8_t *data, uint16_t length){
    static uint16_t buff_ptr = 0;
    for (uint16_t i = 0; i < length; i++){
        uint8_t ch = data[i];
        if (ch == '\r' || ch == '\n' || ch == ' '){
            continue;
        }
        
        sguan_printfbuff[buff_ptr++] = ch;
        if (ch == '?'){
            __local_adjust();
            buff_ptr = 0;
            continue;
        }
        
        if (buff_ptr >= 40){
            memset(sguan_printfbuff, 0, sizeof(sguan_printfbuff));
            buff_ptr = 0;
        }
    }
}

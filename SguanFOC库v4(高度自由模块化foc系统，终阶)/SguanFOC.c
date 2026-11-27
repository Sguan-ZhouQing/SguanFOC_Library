/* ===================================================================
 * 文件:     SguanFOC.c
 * 项目:     SguanFOC_v4 —— 开源 FOC 电机控制框架
 * 说明:     1.Sguan固件库_(固件)算法核心
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
 * 完整许可文本: 参见仓库根目录下的 LICENSE-APACHE 文件。
 * =================================================================== */
#include "SguanFOC.h"

/* SguanFOC配置文件声明 */
#include "UnitLib_main.h"

// static void __local_high_loop(SguanFoc *sguan);
// static void __local_low_loop(SguanFoc *sguan);
// static void __local_printf_loop(uint8_t *data, uint16_t length);
// static void __local_main_loop(SguanFoc *sguan);


// 1. 先定义基础宏
#define __LOCAL_CONCAT(a, b) a##b
#define __LOCAL_EXPAND_MOTOR(n) __LOCAL_CONCAT(__LOCAL_LIST_, n)

#define __LOCAL_LIST_1  X(0)
#define __LOCAL_LIST_2  X(0) X(1)
#define __LOCAL_LIST_3  X(0) X(1) X(2)
#define __LOCAL_LIST_4  X(0) X(1) X(2) X(3)
#define __LOCAL_LIST_5  X(0) X(1) X(2) X(3) X(4)
#define __LOCAL_LIST_6  X(0) X(1) X(2) X(3) X(4) X(5)

// 2. 定义 MOTOR_LIST
#define __LOCAL_MOTOR_LIST __LOCAL_EXPAND_MOTOR(CONFIG_MOTOR)

// 3. 定义函数生成宏
#define __LOCAL_MAKE_MOTOR_FUNCS(n) \
    static void high_loop_##n(void); \
    static void low_loop_##n(void); \
    static void main_loop_##n(void); \
    \
    static void high_loop_##n(void) { \
        SguanFoc *p = &sguanfoc[n]; \
        sguanfoc_high_loop(p); \
    } \
    static void low_loop_##n(void) { \
        SguanFoc *p = &sguanfoc[n]; \
        sguanfoc_low_loop(p); \
    } \
    static void main_loop_##n(void) { \
        SguanFoc *p = &sguanfoc[n]; \
        sguanfoc_main_loop(p); \
    }

// 4. 生成所有电机函数
#define X(n) __LOCAL_MAKE_MOTOR_FUNCS(n)
__LOCAL_MOTOR_LIST
#undef X

// 5. 结构体数组初始化
SguanFoc sguanfoc[CONFIG_MOTOR] = {
    #define X(n) { \
        .id = n, \
        .func_high_loop = high_loop_##n, \
        .func_low_loop = low_loop_##n, \
        .func_printf_loop = sguanfoc_printf_loop, \
        .func_main_loop = main_loop_##n \
    },
    __LOCAL_MOTOR_LIST
    #undef X
};

static void sguanfoc_high_loop(SguanFoc *sguan){
    #if CONFIG_MOTOR >= 0x02
    switch (sguan->id_flag){
    case MOTOR_ONE:
        main_high_loop_one(sguan);
        break;
    case MOTOR_TWO:
        main_high_loop_two(sguan);
        break;
    case MOTOR_THREE:
        main_high_loop_three(sguan);
        break;
    case MOTOR_FOUR:
        main_high_loop_four(sguan);
        break;
    case MOTOR_FIVE:
        main_high_loop_five(sguan);
        break;
    case MOTOR_SIX:
        main_high_loop_six(sguan);
        break;
    
    default:
        break;
    }
    #else // CONFIG_MOTOR
    main_high_loop(sguan);
    #endif // CONFIG_MOTOR
}

static void sguanfoc_low_loop(SguanFoc *sguan){
    main_low_loop(sguan);
}

static void sguanfoc_printf_loop(uint8_t *data, uint16_t length){
    static uint8_t count = 0;
    if (count == 0){
        printf_rx_init();
        count = 1;
    }
    printf_rx_loop(data,length);
}

static void sguanfoc_main_loop(SguanFoc *sguan){
    #if CONFIG_MOTOR >= 0x02
    switch (sguan->id_flag){
    case MOTOR_ONE:
        main_main_loop_one(sguan);
        break;
    case MOTOR_TWO:
        main_main_loop_two(sguan);
        break;
    case MOTOR_THREE:
        main_main_loop_three(sguan);
        break;
    case MOTOR_FOUR:
        main_main_loop_four(sguan);
        break;
    case MOTOR_FIVE:
        main_main_loop_five(sguan);
        break;
    case MOTOR_SIX:
        main_main_loop_six(sguan);
        break;
    
    default:
        break;
    }
    #else // CONFIG_MOTOR
    main_main_loop(sguan);
    #endif // CONFIG_MOTOR
}

void __local_high_loop(const uint8_t *cmds, int count){
    for (int i = 0; i < count; i++) {
        uint8_t cmd = cmds[i];
        if (cmd > 5) continue;
        switch (cmd) {
            case 0: /* ... */ break;
            case 1: /* ... */ break;
            case 2: /* ... */ break;
            case 3: /* ... */ break;
            case 4: /* ... */ break;
            case 5: /* ... */ break;
        }
    }
}

#include "SguanFOC.h"

// 外部文件声明
#include "UnitLib_main.h"

static void sguanfoc_high_loop(SguanFoc *sguan);
static void sguanfoc_low_loop(SguanFoc *sguan);
static void sguanfoc_printf_loop(uint8_t *data, uint16_t length);
static void sguanfoc_main_loop(SguanFoc *sguan);


// 1. 先定义基础宏
#define CONCAT(a, b) a##b
#define EXPAND_MOTOR(n) CONCAT(LIST_, n)

#define LIST_1  X(0)
#define LIST_2  X(0) X(1)
#define LIST_3  X(0) X(1) X(2)
#define LIST_4  X(0) X(1) X(2) X(3)
#define LIST_5  X(0) X(1) X(2) X(3) X(4)
#define LIST_6  X(0) X(1) X(2) X(3) X(4) X(5)

// 2. 定义 MOTOR_LIST
#define MOTOR_LIST EXPAND_MOTOR(CONFIG_MOTOR)

// 3. 定义函数生成宏
#define MAKE_MOTOR_FUNCS(n) \
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
#define X(n) MAKE_MOTOR_FUNCS(n)
MOTOR_LIST
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
    MOTOR_LIST
    #undef X
};


static void sguanfoc_high_loop(SguanFoc *sguan){
    // sguan->id

}

static void sguanfoc_low_loop(SguanFoc *sguan){
    // sguan->id

}

static void sguanfoc_printf_loop(uint8_t *data, uint16_t length){

}

static void sguanfoc_main_loop(SguanFoc *sguan){
    // sguan->id

}


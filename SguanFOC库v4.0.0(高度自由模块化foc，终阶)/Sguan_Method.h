#ifndef __SGUAN_METHOD
#define __SGUAN_METHOD

/* SguanFOC配置文件声明 */
#include "Sguan_Config.h"

typedef struct{
    #if CONFIG_Float // 浮点数
    SguanQ             float_ch[CONFIG_Float];
    #endif // CONFIG_Float

    #if CONFIG_Int8 // 8位数据
    int8_t              int8_ch[CONFIG_Int8];
    #endif // CONFIG_Int8

    #if CONFIG_Uint8 // 8位数据
    uint8_t             uint8_ch[CONFIG_Uint8];
    #endif // CONFIG_Uint8

    #if CONFIG_Int16 // 16位数据
    int16_t             int16_ch[CONFIG_Int16];
    #endif // CONFIG_Int16

    #if CONFIG_Uint16 // 16位数据
    uint16_t            uint16_ch[CONFIG_Uint16];
    #endif // CONFIG_Uint16

    #if CONFIG_Int32 // 32位数据
    int32_t             int32_ch[CONFIG_Int32];
    #endif // CONFIG_Int32

    #if CONFIG_Uint32 // 32位数据
    uint32_t            uint32_ch[CONFIG_Uint32];
    #endif // CONFIG_Uint32

    uint8_t response;                           // (环路倍率)内外环控制倍率
}Method;




#endif // SGUAN_METHOD

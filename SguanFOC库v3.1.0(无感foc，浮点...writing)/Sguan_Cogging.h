#ifndef __SGUAN_COGGING_H
#define __SGUAN_COGGING_H

/* SguanFOC配置文件声明 */
#include "Sguan_Config.h"

typedef struct{
    uint32_t Count;
    uint32_t Time_Count;

    float Input_Iq;
    float Output_Pos;

}COGGING_GO_STRUCT;

typedef struct{
    COGGING_GO_STRUCT go;

    uint32_t Cogging_Count;
    uint32_t Cogging_Cycle;
}COGGING_STRUCT;



#endif // SGUAN_COGGING_H

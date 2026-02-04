#ifndef __SGUAN_SENSORLESS_H
#define __SGUAN_SENSORLESS_H

typedef struct{
    float z[3];
}HFI_STRUCT;

typedef struct{
    float temp;
}SMO_STRUCT;

void HFI_Init(HFI_STRUCT *hfi);
void SMO_Init(SMO_STRUCT *smo);


#endif // SGUAN_SENSORLESS_H

#include "Sguan_Transfer.h"


// ---------------------------工程模块Transfer---------------------------



struct Transfer1Data {
    SguanQ i;
    SguanQ o;

    SguanQ data_num[2];
    SguanQ data_den[2];
};


void transfer_transfer1_init(Transfer1 *transfer){
    #if CONFIG_IQmath


    #else // CONFIG_IQmath
    // 传递函数运算系数固定计算
    transfer->data->data_num[0] = 2.0f*transfer->params.num1 + transfer->params.num0*transfer->params.t;
    transfer->data->data_num[1] = -2.0f*transfer->params.num1 + transfer->params.num0*transfer->params.t;

    transfer->data->data_den[0] = 2.0f*transfer->params.den1 + transfer->params.den0*transfer->params.t;
    transfer->data->data_num[1] = -2.0f*transfer->params.den1 + transfer->params.den0*transfer->params.t;

    // 初始化为零
    transfer->data->i = 0.0f;
    transfer->data->o = 0.0f;

    transfer->in.input = 0.0f;
    transfer->out.output = 0.0f;
    #endif // CONFIG_IQmath
}

void transfer_transfer1_loop(Transfer1 *transfer){
    #if CONFIG_IQmath


    #else // CONFIG_IQmath
    // 1.传递函数运算系数动态计算
    if (transfer->params.recalculate_total_flag){
        transfer->data->data_num[0] = 2.0f*transfer->params.num1 + transfer->params.num0*transfer->params.t;
        transfer->data->data_num[1] = -2.0f*transfer->params.num1 + transfer->params.num0*transfer->params.t;

        transfer->data->data_den[0] = 2.0f*transfer->params.den1 + transfer->params.den0*transfer->params.t;
        transfer->data->data_num[1] = -2.0f*transfer->params.den1 + transfer->params.den0*transfer->params.t;
    }

    // 2.运算传递函数
    transfer->out.output = (transfer->in.input*transfer->data->data_num[0] + transfer->data->i*transfer->data->data_num[1] - transfer->data->o*transfer->data->data_den[1])/transfer->data->data_den[0];

    // 3.更新历史数值
    transfer->data->i = transfer->in.input;
    transfer->data->o = transfer->out.output;
    #endif // CONFIG_IQmath
}

// ---------------------------工程模块Transfer---------------------------



struct Transfer2Data {
    SguanQ i[2];
    SguanQ o[2];

    SguanQ data_num[3];
    SguanQ data_den[3];
};


void transfer_transfer2_init(Transfer2 *transfer){
    #if CONFIG_IQmath


    #else // CONFIG_IQmath
    // 传递函数运算系数固定计算
    transfer->data->data_num[0] = 4.0f*transfer->params.num2 + 2.0f*transfer->params.num1*transfer->params.t + transfer->params.num0*transfer->params.t*transfer->params.t;
    transfer->data->data_num[1] = -8.0f*transfer->params.num2 + 2.0f*transfer->params.num0*transfer->params.t*transfer->params.t;
    transfer->data->data_num[2] = 4.0f*transfer->params.num2 - 2.0f*transfer->params.num1*transfer->params.t + transfer->params.num0*transfer->params.t*transfer->params.t;
    
    transfer->data->data_den[0] = 4.0f*transfer->params.den2 + 2.0f*transfer->params.den1*transfer->params.t + transfer->params.den0*transfer->params.t*transfer->params.t;
    transfer->data->data_den[1] = -8.0f*transfer->params.den2 + 2.0f*transfer->params.den0*transfer->params.t*transfer->params.t;
    transfer->data->data_den[2] = 4.0f*transfer->params.den2 - 2.0f*transfer->params.den1*transfer->params.t + transfer->params.den0*transfer->params.t*transfer->params.t;

    // 初始化为零
    transfer->data->i[0] = 0.0f;
    transfer->data->i[1] = 0.0f;
    
    transfer->data->o[1] = 0.0f;
    transfer->data->o[1] = 0.0f;

    transfer->in.input = 0.0f;
    transfer->out.output = 0.0f;
    #endif // CONFIG_IQmath
}

void transfer_transfer2_loop(Transfer2 *transfer){
    #if CONFIG_IQmath


    #else // CONFIG_IQmath
    // 1.传递函数运算系数动态计算
    if (transfer->params.recalculate_total_flag){
        transfer->data->data_num[0] = 4.0f*transfer->params.num2 + 2.0f*transfer->params.num1*transfer->params.t + transfer->params.num0*transfer->params.t*transfer->params.t;
        transfer->data->data_num[1] = -8.0f*transfer->params.num2 + 2.0f*transfer->params.num0*transfer->params.t*transfer->params.t;
        transfer->data->data_num[2] = 4.0f*transfer->params.num2 - 2.0f*transfer->params.num1*transfer->params.t + transfer->params.num0*transfer->params.t*transfer->params.t;
        
        transfer->data->data_den[0] = 4.0f*transfer->params.den2 + 2.0f*transfer->params.den1*transfer->params.t + transfer->params.den0*transfer->params.t*transfer->params.t;
        transfer->data->data_den[1] = -8.0f*transfer->params.den2 + 2.0f*transfer->params.den0*transfer->params.t*transfer->params.t;
        transfer->data->data_den[2] = 4.0f*transfer->params.den2 - 2.0f*transfer->params.den1*transfer->params.t + transfer->params.den0*transfer->params.t*transfer->params.t;
    }

    // 2.运算传递函数
    transfer->out.output = (transfer->in.input*transfer->data->data_num[0] + transfer->data->i[0]*transfer->data->data_num[1] + transfer->data->i[1]*transfer->data->data_num[2] - transfer->data->o[0]*transfer->data->data_den[1] - transfer->data->o[1]*transfer->data->data_den[2])/transfer->data->data_den[0];

    // 3.更新历史数值
    transfer->data->i[1] = transfer->data->i[0];
    transfer->data->i[0] = transfer->in.input;

    transfer->data->o[1] = transfer->data->o[0];
    transfer->data->o[0] = transfer->out.output;
    #endif // CONFIG_IQmath
}

// ---------------------------工程模块Transfer---------------------------



struct Transfer3Data {
    SguanQ i[3];
    SguanQ o[3];

    SguanQ data_num[4];
    SguanQ data_den[4];
};


void transfer_transfer3_init(Transfer3 *transfer){
    #if CONFIG_IQmath


    #else // CONFIG_IQmath
    // 传递函数运算系数固定计算
    transfer->data->data_num[0] = 8.0f*transfer->params.num3 + 4.0f*transfer->params.num2*transfer->params.t + 2.0f*transfer->params.num1*transfer->params.t*transfer->params.t + transfer->params.num0*transfer->params.t*transfer->params.t*transfer->params.t;
    transfer->data->data_num[1] = -24.0f*transfer->params.num3 - 4.0f*transfer->params.num2*transfer->params.t + 2.0f*transfer->params.num1*transfer->params.t*transfer->params.t + 3.0f*transfer->params.num0*transfer->params.t*transfer->params.t*transfer->params.t;
    transfer->data->data_num[2] = 24.0f*transfer->params.num3 - 4.0f*transfer->params.num2*transfer->params.t - 2.0f*transfer->params.num1*transfer->params.t*transfer->params.t + 3.0f*transfer->params.num0*transfer->params.t*transfer->params.t*transfer->params.t;
    transfer->data->data_num[3] = -8.0f*transfer->params.num3 + 4.0f*transfer->params.num2*transfer->params.t - 2.0f*transfer->params.num1*transfer->params.t*transfer->params.t + transfer->params.num0*transfer->params.t*transfer->params.t*transfer->params.t;
    
    transfer->data->data_den[0] = 8.0f*transfer->params.den3 + 4.0f*transfer->params.den2*transfer->params.t + 2.0f*transfer->params.den1*transfer->params.t*transfer->params.t + transfer->params.den0*transfer->params.t*transfer->params.t*transfer->params.t;
    transfer->data->data_den[1] = -24.0f*transfer->params.den3 - 4.0f*transfer->params.den2*transfer->params.t + 2.0f*transfer->params.den1*transfer->params.t*transfer->params.t + 3.0f*transfer->params.den0*transfer->params.t*transfer->params.t*transfer->params.t;
    transfer->data->data_den[2] = 24.0f*transfer->params.den3 - 4.0f*transfer->params.den2*transfer->params.t - 2.0f*transfer->params.den1*transfer->params.t*transfer->params.t + 3.0f*transfer->params.den0*transfer->params.t*transfer->params.t*transfer->params.t;
    transfer->data->data_den[3] = -8.0f*transfer->params.den3 + 4.0f*transfer->params.den2*transfer->params.t - 2.0f*transfer->params.den1*transfer->params.t*transfer->params.t + transfer->params.den0*transfer->params.t*transfer->params.t*transfer->params.t;
    
    // 初始化为零
    transfer->data->i[0] = 0.0f;
    transfer->data->i[1] = 0.0f;
    transfer->data->i[2] = 0.0f;
    
    transfer->data->o[0] = 0.0f;
    transfer->data->o[1] = 0.0f;
    transfer->data->o[2] = 0.0f;

    transfer->in.input = 0.0f;
    transfer->out.output = 0.0f;
    #endif // CONFIG_IQmath
}

void transfer_transfer3_loop(Transfer3 *transfer){
    #if CONFIG_IQmath


    #else // CONFIG_IQmath
    // 1.传递函数运算系数动态计算
    if (transfer->params.recalculate_total_flag){
        transfer->data->data_num[0] = 8.0f*transfer->params.num3 + 4.0f*transfer->params.num2*transfer->params.t + 2.0f*transfer->params.num1*transfer->params.t*transfer->params.t + transfer->params.num0*transfer->params.t*transfer->params.t*transfer->params.t;
        transfer->data->data_num[1] = -24.0f*transfer->params.num3 - 4.0f*transfer->params.num2*transfer->params.t + 2.0f*transfer->params.num1*transfer->params.t*transfer->params.t + 3.0f*transfer->params.num0*transfer->params.t*transfer->params.t*transfer->params.t;
        transfer->data->data_num[2] = 24.0f*transfer->params.num3 - 4.0f*transfer->params.num2*transfer->params.t - 2.0f*transfer->params.num1*transfer->params.t*transfer->params.t + 3.0f*transfer->params.num0*transfer->params.t*transfer->params.t*transfer->params.t;
        transfer->data->data_num[3] = -8.0f*transfer->params.num3 + 4.0f*transfer->params.num2*transfer->params.t - 2.0f*transfer->params.num1*transfer->params.t*transfer->params.t + transfer->params.num0*transfer->params.t*transfer->params.t*transfer->params.t;
        
        transfer->data->data_den[0] = 8.0f*transfer->params.den3 + 4.0f*transfer->params.den2*transfer->params.t + 2.0f*transfer->params.den1*transfer->params.t*transfer->params.t + transfer->params.den0*transfer->params.t*transfer->params.t*transfer->params.t;
        transfer->data->data_den[1] = -24.0f*transfer->params.den3 - 4.0f*transfer->params.den2*transfer->params.t + 2.0f*transfer->params.den1*transfer->params.t*transfer->params.t + 3.0f*transfer->params.den0*transfer->params.t*transfer->params.t*transfer->params.t;
        transfer->data->data_den[2] = 24.0f*transfer->params.den3 - 4.0f*transfer->params.den2*transfer->params.t - 2.0f*transfer->params.den1*transfer->params.t*transfer->params.t + 3.0f*transfer->params.den0*transfer->params.t*transfer->params.t*transfer->params.t;
        transfer->data->data_den[3] = -8.0f*transfer->params.den3 + 4.0f*transfer->params.den2*transfer->params.t - 2.0f*transfer->params.den1*transfer->params.t*transfer->params.t + transfer->params.den0*transfer->params.t*transfer->params.t*transfer->params.t;
    }

    // 2.运算传递函数
    transfer->out.output = (transfer->in.input*transfer->data->data_num[0] + transfer->data->i[0]*transfer->data->data_num[1] + transfer->data->i[1]*transfer->data->data_num[2] + transfer->data->i[2]*transfer->data->data_num[3] - transfer->data->o[0]*transfer->data->data_den[1] - transfer->data->o[1]*transfer->data->data_den[2] - transfer->data->o[2]*transfer->data->data_den[3])/transfer->data->data_den[0];

    // 3.更新历史数值
    transfer->data->i[2] = transfer->data->i[1];
    transfer->data->i[1] = transfer->data->i[0];
    transfer->data->i[0] = transfer->in.input;

    transfer->data->o[2] = transfer->data->o[1];
    transfer->data->o[1] = transfer->data->o[0];
    transfer->data->o[0] = transfer->out.output;
    #endif // CONFIG_IQmath
}

// ---------------------------工程模块Transfer---------------------------



struct Transfer4Data {
    SguanQ i[4];
    SguanQ o[4];

    SguanQ data_num[5];
    SguanQ data_den[5];
};


void transfer_transfer4_init(Transfer4 *transfer){
    #if CONFIG_IQmath


    #else // CONFIG_IQmath
    // 传递函数运算系数固定计算
    transfer->data->data_num[0] = 16.0f*transfer->params.num4 + 8.0f*transfer->params.num3*transfer->params.t + 4.0f*transfer->params.num2*transfer->params.t*transfer->params.t + 2.0f*transfer->params.num1*transfer->params.t*transfer->params.t*transfer->params.t + transfer->params.num0*transfer->params.t*transfer->params.t*transfer->params.t*transfer->params.t;
    transfer->data->data_num[1] = -64.0f*transfer->params.num4 - 16.0f*transfer->params.num3*transfer->params.t + 4.0f*transfer->params.num1*transfer->params.t*transfer->params.t*transfer->params.t + 4.0f*transfer->params.num0*transfer->params.t*transfer->params.t*transfer->params.t*transfer->params.t;
    transfer->data->data_num[2] = 96.0f*transfer->params.num4 - 8.0f*transfer->params.num2*transfer->params.t*transfer->params.t + 6.0f*transfer->params.num0*transfer->params.t*transfer->params.t*transfer->params.t*transfer->params.t;
    transfer->data->data_num[3] = -64.0f*transfer->params.num4 + 16.0f*transfer->params.num3*transfer->params.t - 4.0f*transfer->params.num1*transfer->params.t*transfer->params.t*transfer->params.t + 4.0f*transfer->params.num0*transfer->params.t*transfer->params.t*transfer->params.t*transfer->params.t;
    transfer->data->data_num[4] = 16.0f*transfer->params.num4 - 8.0f*transfer->params.num3*transfer->params.t + 4.0f*transfer->params.num2*transfer->params.t*transfer->params.t - 2.0f*transfer->params.num1*transfer->params.t*transfer->params.t*transfer->params.t + transfer->params.num0*transfer->params.t*transfer->params.t*transfer->params.t*transfer->params.t;
    
    transfer->data->data_den[0] = 16.0f*transfer->params.den4 + 8.0f*transfer->params.den3*transfer->params.t + 4.0f*transfer->params.den2*transfer->params.t*transfer->params.t + 2.0f*transfer->params.den1*transfer->params.t*transfer->params.t*transfer->params.t + transfer->params.den0*transfer->params.t*transfer->params.t*transfer->params.t*transfer->params.t;
    transfer->data->data_den[1] = -64.0f*transfer->params.den4 - 16.0f*transfer->params.den3*transfer->params.t + 4.0f*transfer->params.den1*transfer->params.t*transfer->params.t*transfer->params.t + 4.0f*transfer->params.den0*transfer->params.t*transfer->params.t*transfer->params.t*transfer->params.t;
    transfer->data->data_den[2] = 96.0f*transfer->params.den4 - 8.0f*transfer->params.den2*transfer->params.t*transfer->params.t + 6.0f*transfer->params.den0*transfer->params.t*transfer->params.t*transfer->params.t*transfer->params.t;
    transfer->data->data_den[3] = -64.0f*transfer->params.den4 + 16.0f*transfer->params.den3*transfer->params.t - 4.0f*transfer->params.den1*transfer->params.t*transfer->params.t*transfer->params.t + 4.0f*transfer->params.den0*transfer->params.t*transfer->params.t*transfer->params.t*transfer->params.t;
    transfer->data->data_den[4] = 16.0f*transfer->params.den4 - 8.0f*transfer->params.den3*transfer->params.t + 4.0f*transfer->params.den2*transfer->params.t*transfer->params.t - 2.0f*transfer->params.den1*transfer->params.t*transfer->params.t*transfer->params.t + transfer->params.den0*transfer->params.t*transfer->params.t*transfer->params.t*transfer->params.t;

    // 初始化为零
    transfer->data->i[0] = 0.0f;
    transfer->data->i[1] = 0.0f;
    transfer->data->i[2] = 0.0f;
    transfer->data->i[3] = 0.0f;

    transfer->data->o[0] = 0.0f;
    transfer->data->o[1] = 0.0f;
    transfer->data->o[2] = 0.0f;
    transfer->data->o[3] = 0.0f;

    transfer->in.input = 0.0f;
    transfer->out.output = 0.0f;
    #endif // CONFIG_IQmath
}

void transfer_transfer4_loop(Transfer4 *transfer){
    #if CONFIG_IQmath


    #else // CONFIG_IQmath
    // 1.传递函数运算系数动态计算
    if (transfer->params.recalculate_total_flag){
        transfer->data->data_num[0] = 16.0f*transfer->params.num4 + 8.0f*transfer->params.num3*transfer->params.t + 4.0f*transfer->params.num2*transfer->params.t*transfer->params.t + 2.0f*transfer->params.num1*transfer->params.t*transfer->params.t*transfer->params.t + transfer->params.num0*transfer->params.t*transfer->params.t*transfer->params.t*transfer->params.t;
        transfer->data->data_num[1] = -64.0f*transfer->params.num4 - 16.0f*transfer->params.num3*transfer->params.t + 4.0f*transfer->params.num1*transfer->params.t*transfer->params.t*transfer->params.t + 4.0f*transfer->params.num0*transfer->params.t*transfer->params.t*transfer->params.t*transfer->params.t;
        transfer->data->data_num[2] = 96.0f*transfer->params.num4 - 8.0f*transfer->params.num2*transfer->params.t*transfer->params.t + 6.0f*transfer->params.num0*transfer->params.t*transfer->params.t*transfer->params.t*transfer->params.t;
        transfer->data->data_num[3] = -64.0f*transfer->params.num4 + 16.0f*transfer->params.num3*transfer->params.t - 4.0f*transfer->params.num1*transfer->params.t*transfer->params.t*transfer->params.t + 4.0f*transfer->params.num0*transfer->params.t*transfer->params.t*transfer->params.t*transfer->params.t;
        transfer->data->data_num[4] = 16.0f*transfer->params.num4 - 8.0f*transfer->params.num3*transfer->params.t + 4.0f*transfer->params.num2*transfer->params.t*transfer->params.t - 2.0f*transfer->params.num1*transfer->params.t*transfer->params.t*transfer->params.t + transfer->params.num0*transfer->params.t*transfer->params.t*transfer->params.t*transfer->params.t;
        
        transfer->data->data_den[0] = 16.0f*transfer->params.den4 + 8.0f*transfer->params.den3*transfer->params.t + 4.0f*transfer->params.den2*transfer->params.t*transfer->params.t + 2.0f*transfer->params.den1*transfer->params.t*transfer->params.t*transfer->params.t + transfer->params.den0*transfer->params.t*transfer->params.t*transfer->params.t*transfer->params.t;
        transfer->data->data_den[1] = -64.0f*transfer->params.den4 - 16.0f*transfer->params.den3*transfer->params.t + 4.0f*transfer->params.den1*transfer->params.t*transfer->params.t*transfer->params.t + 4.0f*transfer->params.den0*transfer->params.t*transfer->params.t*transfer->params.t*transfer->params.t;
        transfer->data->data_den[2] = 96.0f*transfer->params.den4 - 8.0f*transfer->params.den2*transfer->params.t*transfer->params.t + 6.0f*transfer->params.den0*transfer->params.t*transfer->params.t*transfer->params.t*transfer->params.t;
        transfer->data->data_den[3] = -64.0f*transfer->params.den4 + 16.0f*transfer->params.den3*transfer->params.t - 4.0f*transfer->params.den1*transfer->params.t*transfer->params.t*transfer->params.t + 4.0f*transfer->params.den0*transfer->params.t*transfer->params.t*transfer->params.t*transfer->params.t;
        transfer->data->data_den[4] = 16.0f*transfer->params.den4 - 8.0f*transfer->params.den3*transfer->params.t + 4.0f*transfer->params.den2*transfer->params.t*transfer->params.t - 2.0f*transfer->params.den1*transfer->params.t*transfer->params.t*transfer->params.t + transfer->params.den0*transfer->params.t*transfer->params.t*transfer->params.t*transfer->params.t;
    }
    
    // 2.运算传递函数
    transfer->out.output = (transfer->in.input*transfer->data->data_num[0] + transfer->data->i[0]*transfer->data->data_num[1] + transfer->data->i[1]*transfer->data->data_num[2] + transfer->data->i[2]*transfer->data->data_num[3] + transfer->data->i[3]*transfer->data->data_num[4] - transfer->data->o[0]*transfer->data->data_den[1] - transfer->data->o[1]*transfer->data->data_den[2] - transfer->data->o[2]*transfer->data->data_den[3] - transfer->data->o[3]*transfer->data->data_den[4])/transfer->data->data_den[0];

    // 3.更新历史数值
    transfer->data->i[3] = transfer->data->i[2];
    transfer->data->i[2] = transfer->data->i[1];
    transfer->data->i[1] = transfer->data->i[0];
    transfer->data->i[0] = transfer->in.input;

    transfer->data->o[3] = transfer->data->o[2];
    transfer->data->o[2] = transfer->data->o[1];
    transfer->data->o[1] = transfer->data->o[0];
    transfer->data->o[0] = transfer->out.output;
    #endif // CONFIG_IQmath
}

// ---------------------------工程模块Transfer---------------------------



struct Transfer5Data {
    SguanQ i[5];
    SguanQ o[5];

    SguanQ data_num[6];
    SguanQ data_den[6];
};


void transfer_transfer5_init(Transfer5 *transfer){
    #if CONFIG_IQmath


    #else // CONFIG_IQmath
    // 传递函数运算系数固定计算
    transfer->data->data_num[0] = 32.0f*transfer->params.num5 + 16.0f*transfer->params.num4*transfer->params.t + 8.0f*transfer->params.num3*transfer->params.t*transfer->params.t + 4.0f*transfer->params.num2*transfer->params.t*transfer->params.t*transfer->params.t + 2.0f*transfer->params.num1*transfer->params.t*transfer->params.t*transfer->params.t*transfer->params.t + transfer->params.num0*transfer->params.t*transfer->params.t*transfer->params.t*transfer->params.t*transfer->params.t;
    transfer->data->data_num[1] = -160.0f*transfer->params.num5 - 48.0f*transfer->params.num4*transfer->params.t - 8.0f*transfer->params.num3*transfer->params.t*transfer->params.t + 4.0f*transfer->params.num2*transfer->params.t*transfer->params.t*transfer->params.t + 6.0f*transfer->params.num1*transfer->params.t*transfer->params.t*transfer->params.t*transfer->params.t + 5.0f*transfer->params.num0*transfer->params.t*transfer->params.t*transfer->params.t*transfer->params.t*transfer->params.t;
    transfer->data->data_num[2] = 320.0f*transfer->params.num5 + 32.0f*transfer->params.num4*transfer->params.t - 16.0f*transfer->params.num3*transfer->params.t*transfer->params.t - 8.0f*transfer->params.num2*transfer->params.t*transfer->params.t*transfer->params.t + 4.0f*transfer->params.num1*transfer->params.t*transfer->params.t*transfer->params.t*transfer->params.t + 10.0f*transfer->params.num0*transfer->params.t*transfer->params.t*transfer->params.t*transfer->params.t*transfer->params.t;
    transfer->data->data_num[3] = -320.0f*transfer->params.num5 + 32.0f*transfer->params.num4*transfer->params.t + 16.0f*transfer->params.num3*transfer->params.t*transfer->params.t - 8.0f*transfer->params.num2*transfer->params.t*transfer->params.t*transfer->params.t - 4.0f*transfer->params.num1*transfer->params.t*transfer->params.t*transfer->params.t*transfer->params.t + 10.0f*transfer->params.num0*transfer->params.t*transfer->params.t*transfer->params.t*transfer->params.t*transfer->params.t;
    transfer->data->data_num[4] = 160.0f*transfer->params.num5 - 48.0f*transfer->params.num4*transfer->params.t + 8.0f*transfer->params.num3*transfer->params.t*transfer->params.t + 4.0f*transfer->params.num2*transfer->params.t*transfer->params.t*transfer->params.t - 6.0f*transfer->params.num1*transfer->params.t*transfer->params.t*transfer->params.t*transfer->params.t + 5.0f*transfer->params.num0*transfer->params.t*transfer->params.t*transfer->params.t*transfer->params.t*transfer->params.t;
    transfer->data->data_num[5] = -32.0f*transfer->params.num5 + 16.0f*transfer->params.num4*transfer->params.t - 8.0f*transfer->params.num3*transfer->params.t*transfer->params.t + 4.0f*transfer->params.num2*transfer->params.t*transfer->params.t*transfer->params.t - 2.0f*transfer->params.num1*transfer->params.t*transfer->params.t*transfer->params.t*transfer->params.t + transfer->params.num0*transfer->params.t*transfer->params.t*transfer->params.t*transfer->params.t*transfer->params.t;
    
    transfer->data->data_den[0] = 32.0f*transfer->params.den5 + 16.0f*transfer->params.den4*transfer->params.t + 8.0f*transfer->params.den3*transfer->params.t*transfer->params.t + 4.0f*transfer->params.den2*transfer->params.t*transfer->params.t*transfer->params.t + 2.0f*transfer->params.den1*transfer->params.t*transfer->params.t*transfer->params.t*transfer->params.t + transfer->params.den0*transfer->params.t*transfer->params.t*transfer->params.t*transfer->params.t*transfer->params.t;
    transfer->data->data_den[1] = -160.0f*transfer->params.den5 - 48.0f*transfer->params.den4*transfer->params.t - 8.0f*transfer->params.den3*transfer->params.t*transfer->params.t + 4.0f*transfer->params.den2*transfer->params.t*transfer->params.t*transfer->params.t + 6.0f*transfer->params.den1*transfer->params.t*transfer->params.t*transfer->params.t*transfer->params.t + 5.0f*transfer->params.den0*transfer->params.t*transfer->params.t*transfer->params.t*transfer->params.t*transfer->params.t;
    transfer->data->data_den[2] = 320.0f*transfer->params.den5 + 32.0f*transfer->params.den4*transfer->params.t - 16.0f*transfer->params.den3*transfer->params.t*transfer->params.t - 8.0f*transfer->params.den2*transfer->params.t*transfer->params.t*transfer->params.t + 4.0f*transfer->params.den1*transfer->params.t*transfer->params.t*transfer->params.t*transfer->params.t + 10.0f*transfer->params.den0*transfer->params.t*transfer->params.t*transfer->params.t*transfer->params.t*transfer->params.t;
    transfer->data->data_den[3] = -320.0f*transfer->params.den5 + 32.0f*transfer->params.den4*transfer->params.t + 16.0f*transfer->params.den3*transfer->params.t*transfer->params.t - 8.0f*transfer->params.den2*transfer->params.t*transfer->params.t*transfer->params.t - 4.0f*transfer->params.den1*transfer->params.t*transfer->params.t*transfer->params.t*transfer->params.t + 10.0f*transfer->params.den0*transfer->params.t*transfer->params.t*transfer->params.t*transfer->params.t*transfer->params.t;
    transfer->data->data_den[4] = 160.0f*transfer->params.den5 - 48.0f*transfer->params.den4*transfer->params.t + 8.0f*transfer->params.den3*transfer->params.t*transfer->params.t + 4.0f*transfer->params.den2*transfer->params.t*transfer->params.t*transfer->params.t - 6.0f*transfer->params.den1*transfer->params.t*transfer->params.t*transfer->params.t*transfer->params.t + 5.0f*transfer->params.den0*transfer->params.t*transfer->params.t*transfer->params.t*transfer->params.t*transfer->params.t;
    transfer->data->data_den[5] = -32.0f*transfer->params.den5 + 16.0f*transfer->params.den4*transfer->params.t - 8.0f*transfer->params.den3*transfer->params.t*transfer->params.t + 4.0f*transfer->params.den2*transfer->params.t*transfer->params.t*transfer->params.t - 2.0f*transfer->params.den1*transfer->params.t*transfer->params.t*transfer->params.t*transfer->params.t + transfer->params.den0*transfer->params.t*transfer->params.t*transfer->params.t*transfer->params.t*transfer->params.t;
    
    // 初始化为零
    transfer->data->i[0] = 0.0f;
    transfer->data->i[1] = 0.0f;
    transfer->data->i[2] = 0.0f;
    transfer->data->i[3] = 0.0f;
    transfer->data->i[4] = 0.0f;

    transfer->data->o[0] = 0.0f;
    transfer->data->o[1] = 0.0f;
    transfer->data->o[2] = 0.0f;
    transfer->data->o[3] = 0.0f;
    transfer->data->o[4] = 0.0f;

    transfer->in.input = 0.0f;
    transfer->out.output = 0.0f;
    #endif // CONFIG_IQmath
}

void transfer_transfer5_loop(Transfer5 *transfer){
    #if CONFIG_IQmath


    #else // CONFIG_IQmath
    // 1.传递函数运算系数动态计算
    if (transfer->params.recalculate_total_flag){
        transfer->data->data_num[0] = 32.0f*transfer->params.num5 + 16.0f*transfer->params.num4*transfer->params.t + 8.0f*transfer->params.num3*transfer->params.t*transfer->params.t + 4.0f*transfer->params.num2*transfer->params.t*transfer->params.t*transfer->params.t + 2.0f*transfer->params.num1*transfer->params.t*transfer->params.t*transfer->params.t*transfer->params.t + transfer->params.num0*transfer->params.t*transfer->params.t*transfer->params.t*transfer->params.t*transfer->params.t;
        transfer->data->data_num[1] = -160.0f*transfer->params.num5 - 48.0f*transfer->params.num4*transfer->params.t - 8.0f*transfer->params.num3*transfer->params.t*transfer->params.t + 4.0f*transfer->params.num2*transfer->params.t*transfer->params.t*transfer->params.t + 6.0f*transfer->params.num1*transfer->params.t*transfer->params.t*transfer->params.t*transfer->params.t + 5.0f*transfer->params.num0*transfer->params.t*transfer->params.t*transfer->params.t*transfer->params.t*transfer->params.t;
        transfer->data->data_num[2] = 320.0f*transfer->params.num5 + 32.0f*transfer->params.num4*transfer->params.t - 16.0f*transfer->params.num3*transfer->params.t*transfer->params.t - 8.0f*transfer->params.num2*transfer->params.t*transfer->params.t*transfer->params.t + 4.0f*transfer->params.num1*transfer->params.t*transfer->params.t*transfer->params.t*transfer->params.t + 10.0f*transfer->params.num0*transfer->params.t*transfer->params.t*transfer->params.t*transfer->params.t*transfer->params.t;
        transfer->data->data_num[3] = -320.0f*transfer->params.num5 + 32.0f*transfer->params.num4*transfer->params.t + 16.0f*transfer->params.num3*transfer->params.t*transfer->params.t - 8.0f*transfer->params.num2*transfer->params.t*transfer->params.t*transfer->params.t - 4.0f*transfer->params.num1*transfer->params.t*transfer->params.t*transfer->params.t*transfer->params.t + 10.0f*transfer->params.num0*transfer->params.t*transfer->params.t*transfer->params.t*transfer->params.t*transfer->params.t;
        transfer->data->data_num[4] = 160.0f*transfer->params.num5 - 48.0f*transfer->params.num4*transfer->params.t + 8.0f*transfer->params.num3*transfer->params.t*transfer->params.t + 4.0f*transfer->params.num2*transfer->params.t*transfer->params.t*transfer->params.t - 6.0f*transfer->params.num1*transfer->params.t*transfer->params.t*transfer->params.t*transfer->params.t + 5.0f*transfer->params.num0*transfer->params.t*transfer->params.t*transfer->params.t*transfer->params.t*transfer->params.t;
        transfer->data->data_num[5] = -32.0f*transfer->params.num5 + 16.0f*transfer->params.num4*transfer->params.t - 8.0f*transfer->params.num3*transfer->params.t*transfer->params.t + 4.0f*transfer->params.num2*transfer->params.t*transfer->params.t*transfer->params.t - 2.0f*transfer->params.num1*transfer->params.t*transfer->params.t*transfer->params.t*transfer->params.t + transfer->params.num0*transfer->params.t*transfer->params.t*transfer->params.t*transfer->params.t*transfer->params.t;
        
        transfer->data->data_den[0] = 32.0f*transfer->params.den5 + 16.0f*transfer->params.den4*transfer->params.t + 8.0f*transfer->params.den3*transfer->params.t*transfer->params.t + 4.0f*transfer->params.den2*transfer->params.t*transfer->params.t*transfer->params.t + 2.0f*transfer->params.den1*transfer->params.t*transfer->params.t*transfer->params.t*transfer->params.t + transfer->params.den0*transfer->params.t*transfer->params.t*transfer->params.t*transfer->params.t*transfer->params.t;
        transfer->data->data_den[1] = -160.0f*transfer->params.den5 - 48.0f*transfer->params.den4*transfer->params.t - 8.0f*transfer->params.den3*transfer->params.t*transfer->params.t + 4.0f*transfer->params.den2*transfer->params.t*transfer->params.t*transfer->params.t + 6.0f*transfer->params.den1*transfer->params.t*transfer->params.t*transfer->params.t*transfer->params.t + 5.0f*transfer->params.den0*transfer->params.t*transfer->params.t*transfer->params.t*transfer->params.t*transfer->params.t;
        transfer->data->data_den[2] = 320.0f*transfer->params.den5 + 32.0f*transfer->params.den4*transfer->params.t - 16.0f*transfer->params.den3*transfer->params.t*transfer->params.t - 8.0f*transfer->params.den2*transfer->params.t*transfer->params.t*transfer->params.t + 4.0f*transfer->params.den1*transfer->params.t*transfer->params.t*transfer->params.t*transfer->params.t + 10.0f*transfer->params.den0*transfer->params.t*transfer->params.t*transfer->params.t*transfer->params.t*transfer->params.t;
        transfer->data->data_den[3] = -320.0f*transfer->params.den5 + 32.0f*transfer->params.den4*transfer->params.t + 16.0f*transfer->params.den3*transfer->params.t*transfer->params.t - 8.0f*transfer->params.den2*transfer->params.t*transfer->params.t*transfer->params.t - 4.0f*transfer->params.den1*transfer->params.t*transfer->params.t*transfer->params.t*transfer->params.t + 10.0f*transfer->params.den0*transfer->params.t*transfer->params.t*transfer->params.t*transfer->params.t*transfer->params.t;
        transfer->data->data_den[4] = 160.0f*transfer->params.den5 - 48.0f*transfer->params.den4*transfer->params.t + 8.0f*transfer->params.den3*transfer->params.t*transfer->params.t + 4.0f*transfer->params.den2*transfer->params.t*transfer->params.t*transfer->params.t - 6.0f*transfer->params.den1*transfer->params.t*transfer->params.t*transfer->params.t*transfer->params.t + 5.0f*transfer->params.den0*transfer->params.t*transfer->params.t*transfer->params.t*transfer->params.t*transfer->params.t;
        transfer->data->data_den[5] = -32.0f*transfer->params.den5 + 16.0f*transfer->params.den4*transfer->params.t - 8.0f*transfer->params.den3*transfer->params.t*transfer->params.t + 4.0f*transfer->params.den2*transfer->params.t*transfer->params.t*transfer->params.t - 2.0f*transfer->params.den1*transfer->params.t*transfer->params.t*transfer->params.t*transfer->params.t + transfer->params.den0*transfer->params.t*transfer->params.t*transfer->params.t*transfer->params.t*transfer->params.t;
    }
    
    // 2.运算传递函数
    transfer->out.output = (transfer->in.input*transfer->data->data_num[0] + transfer->data->i[0]*transfer->data->data_num[1] + transfer->data->i[1]*transfer->data->data_num[2] + transfer->data->i[2]*transfer->data->data_num[3] + transfer->data->i[3]*transfer->data->data_num[4] + transfer->data->i[4]*transfer->data->data_num[5] - transfer->data->o[0]*transfer->data->data_den[1] - transfer->data->o[1]*transfer->data->data_den[2] - transfer->data->o[2]*transfer->data->data_den[3] - transfer->data->o[3]*transfer->data->data_den[4] - transfer->data->o[4]*transfer->data->data_den[5])/transfer->data->data_den[0];

    // 3.更新历史数值
    transfer->data->i[4] = transfer->data->i[3];
    transfer->data->i[3] = transfer->data->i[2];
    transfer->data->i[2] = transfer->data->i[1];
    transfer->data->i[1] = transfer->data->i[0];
    transfer->data->i[0] = transfer->in.input;

    transfer->data->o[4] = transfer->data->o[3];
    transfer->data->o[3] = transfer->data->o[2];
    transfer->data->o[2] = transfer->data->o[1];
    transfer->data->o[1] = transfer->data->o[0];
    transfer->data->o[0] = transfer->out.output;
    #endif // CONFIG_IQmath
}

// ---------------------------工程模块Transfer---------------------------



struct IntegratorData {
    SguanQ i;

    SguanQ data_num;
};


void transfer_integrator_init(Integrator *integrator){
    #if CONFIG_IQmath


    #else // CONFIG_IQmath
    // 传递函数运算系数固定计算
    integrator->data->data_num = integrator->params.t/2.0f;

    // 初始化为零
    integrator->data->i = 0.0f;

    integrator->in.input = 0.0f;
    integrator->out.output = 0.0f;
    #endif // CONFIG_IQmath
}

void transfer_integrator_loop(Integrator *integrator){
    #if CONFIG_IQmath


    #else // CONFIG_IQmath
    // 1.运算传递函数
    integrator->out.output += (integrator->in.input + integrator->data->i)*integrator->data->data_num;

    // 2.更新历史数值
    integrator->data->i = integrator->in.input;
    #endif // CONFIG_IQmath
}

// ---------------------------工程模块Transfer---------------------------



struct DerivativeData {
    SguanQ i;
    SguanQ o;

    SguanQ data_num[2];
    SguanQ data_den[2];
};


void transfer_derivative_init(Derivative *derivative){
    #if CONFIG_IQmath


    #else // CONFIG_IQmath
    // 传递函数运算系数固定计算
    derivative->data->data_num[0] = 2.0f*derivative->params.wc;
    derivative->data->data_num[1] = -2.0f*derivative->params.wc;

    derivative->data->data_den[0] = 2.0f + derivative->params.t*derivative->params.wc;
    derivative->data->data_den[1] = -2.0f + derivative->params.t*derivative->params.wc;

    // 初始化为零
    derivative->data->i = 0.0f;
    derivative->data->o = 0.0f;

    derivative->in.input = 0.0f;
    derivative->out.output = 0.0f;
    #endif // CONFIG_IQmath
}

void transfer_derivative_loop(Derivative *derivative){
    #if CONFIG_IQmath


    #else // CONFIG_IQmath
    // 1.传递函数运算系数动态计算
    if (derivative->params.recalculate_total_flag){
        derivative->data->data_num[0] = 2.0f*derivative->params.wc;
        derivative->data->data_num[1] = -2.0f*derivative->params.wc;
        
        derivative->data->data_den[0] = 2.0f + derivative->params.t*derivative->params.wc;
        derivative->data->data_den[1] = -2.0f + derivative->params.t*derivative->params.wc;
    }

    // 2.运算传递函数
    derivative->out.output = (derivative->in.input*derivative->data->data_num[0] + derivative->data->i*derivative->data->data_num[1] - derivative->data->o*derivative->data->data_den[1])/derivative->data->data_den[0];

    // 3.更新历史数值
    derivative->data->i = derivative->in.input;
    derivative->data->o = derivative->out.output;
    #endif // CONFIG_IQmath
}

// ---------------------------工程模块Transfer---------------------------
struct DftData {
    SguanQ i;
    SguanQ o;

    SguanQ data_num[2];
    SguanQ data_den[2];
};


void transfer_dft_init(Dft *dft){

}


void transfer_dft_loop(Dft *dft){
    
}

// ---------------------------工程模块Transfer---------------------------



struct HallData {
    SguanQ num;
    SguanQ den;
};


void transfer_hall_init(Hall *hall){
    #if CONFIG_IQmath


    #else // CONFIG_IQmath
    // 传递函数运算系数固定计算



    // 初始化为零

    
    #endif // CONFIG_IQmath
}

void transfer_hall_loop(Hall *hall){
    #if CONFIG_IQmath


    #else // CONFIG_IQmath




    #endif // CONFIG_IQmath
}

// ---------------------------工程模块Transfer---------------------------



struct Ladrc1Data {
    SguanQ num;
    SguanQ den;
};


void transfer_ladrc1_init(Ladrc1 *ladrc){
    #if CONFIG_IQmath


    #else // CONFIG_IQmath
    // 传递函数运算系数固定计算



    // 初始化为零

    
    #endif // CONFIG_IQmath
}

void transfer_ladrc1_loop(Ladrc1 *ladrc){
    #if CONFIG_IQmath


    #else // CONFIG_IQmath


    

    #endif // CONFIG_IQmath
}

// ---------------------------工程模块Transfer---------------------------



struct Ladrc2Data {
    SguanQ num;
    SguanQ den;
};


void transfer_ladrc2_init(Ladrc2 *ladrc){
    #if CONFIG_IQmath


    #else // CONFIG_IQmath


    
    #endif // CONFIG_IQmath
}

void transfer_ladrc2_loop(Ladrc2 *ladrc){
    #if CONFIG_IQmath


    #else // CONFIG_IQmath


    

    #endif // CONFIG_IQmath
}

// ---------------------------工程模块Transfer---------------------------



struct SmcData {
    SguanQ num;
    SguanQ den;
};


void transfer_smc_init(Smc *smc){
    #if CONFIG_IQmath


    #else // CONFIG_IQmath


    
    #endif // CONFIG_IQmath
}

void transfer_smc_loop(Smc *smc){
    #if CONFIG_IQmath


    #else // CONFIG_IQmath


    

    #endif // CONFIG_IQmath
}

// ---------------------------工程模块Transfer---------------------------



struct DpccData {
    SguanQ num;
    SguanQ den;
};


void transfer_dpcc_init(Dpcc *dpcc){
    #if CONFIG_IQmath


    #else // CONFIG_IQmath


    
    #endif // CONFIG_IQmath
}

void transfer_dpcc_loop(Dpcc *dpcc){
    #if CONFIG_IQmath


    #else // CONFIG_IQmath


    

    #endif // CONFIG_IQmath
}

// ---------------------------工程模块Transfer---------------------------



struct PirData {
    SguanQ num;
    SguanQ den;
};


void transfer_pir_init(Pir *pir){
    #if CONFIG_IQmath


    #else // CONFIG_IQmath


    
    #endif // CONFIG_IQmath
}

void transfer_pir_loop(Pir *pir){
    #if CONFIG_IQmath


    #else // CONFIG_IQmath


    

    #endif // CONFIG_IQmath
}

// ---------------------------工程模块Transfer---------------------------



struct PidData {
    SguanQ num;
    SguanQ den;
};


void transfer_pid_init(Pid *pid){
    #if CONFIG_IQmath


    #else // CONFIG_IQmath


    
    #endif // CONFIG_IQmath
}

void transfer_pid_loop(Pid *pid){
    #if CONFIG_IQmath


    #else // CONFIG_IQmath


    

    #endif // CONFIG_IQmath
}

// ---------------------------工程模块Transfer---------------------------



struct PllData {
    SguanQ num;
    SguanQ den;
};


void transfer_pll_init(Pll *pll){
    #if CONFIG_IQmath


    #else // CONFIG_IQmath


    
    #endif // CONFIG_IQmath
}

void transfer_pll_loop(Pll *pll){
    #if CONFIG_IQmath


    #else // CONFIG_IQmath


    

    #endif // CONFIG_IQmath
}

// ---------------------------工程模块Transfer---------------------------



struct Lpf1Data {
    SguanQ i;
    SguanQ o;

    SguanQ data_num;
    SguanQ data_den[2];
};


void transfer_lpf1_init(Lpf1 *lpf){
    #if CONFIG_IQmath


    #else // CONFIG_IQmath
    // 传递函数运算系数固定计算
    lpf->data->data_num = lpf->params.t*lpf->params.wc;

    lpf->data->data_den[0] = 2.0f + lpf->params.t*lpf->params.wc;
    lpf->data->data_den[1] = -2.0f + lpf->params.t*lpf->params.wc;

    // 初始化为零
    lpf->data->i = 0.0f;
    lpf->data->o = 0.0f;

    lpf->in.input = 0.0f;
    lpf->out.output = 0.0f;
    #endif // CONFIG_IQmath
}

void transfer_lpf1_loop(Lpf1 *lpf){
    #if CONFIG_IQmath


    #else // CONFIG_IQmath
    // 1.传递函数运算系数动态计算
    if (lpf->params.recalculate_total_flag){
        lpf->data->data_num = lpf->params.t*lpf->params.wc;
    
        lpf->data->data_den[0] = 2.0f + lpf->params.t*lpf->params.wc;
        lpf->data->data_den[1] = -2.0f + lpf->params.t*lpf->params.wc;
    }

    // 2.运算传递函数
    lpf->out.output = (lpf->in.input+lpf->data->i - lpf->data->o*lpf->data->data_den[1])/lpf->data->data_den[0];

    // 3.更新历史数值
    lpf->data->i = lpf->in.input;
    lpf->data->o = lpf->out.output;
    #endif // CONFIG_IQmath
}

// ---------------------------工程模块Transfer---------------------------



struct Lpf2Data {
    SguanQ i[2];
    SguanQ o[2];
    
    // (data_num)0->现在和LLast
    // (data_num)1->仅Last的系数
    // (传递函数分子系数)
    SguanQ data_num[2];
    SguanQ data_den[3];
};


void transfer_lpf2_init(Lpf2 *lpf){
    #if CONFIG_IQmath


    #else // CONFIG_IQmath
    // 传递函数运算系数固定计算
    lpf->data->data_num[0] = lpf->params.t*lpf->params.wc*lpf->params.t*lpf->params.wc;
    lpf->data->data_num[1] = 2.0f*lpf->params.t*lpf->params.wc*lpf->params.t*lpf->params.wc;

    lpf->data->data_den[0] = 4.0f + MATH_Value_2_SQRT2*lpf->params.t*lpf->params.wc + lpf->params.t*lpf->params.wc*lpf->params.t*lpf->params.wc;
    lpf->data->data_den[1] = -8.0f + 2.0f*lpf->params.t*lpf->params.wc*lpf->params.t*lpf->params.wc;
    lpf->data->data_den[2] = 4.0f - MATH_Value_2_SQRT2*lpf->params.t*lpf->params.wc + lpf->params.t*lpf->params.wc*lpf->params.t*lpf->params.wc;

    // 初始化为零
    lpf->data->i[0] = 0.0f;
    lpf->data->i[1] = 0.0f;
    lpf->data->o[0] = 0.0f;
    lpf->data->o[1] = 0.0f;
    
    lpf->in.input = 0.0f;
    lpf->out.output = 0.0f;
    #endif // CONFIG_IQmath
}

void transfer_lpf2_loop(Lpf2 *lpf){
    #if CONFIG_IQmath


    #else // CONFIG_IQmath
    // 1.传递函数运算系数动态计算
    if (lpf->params.recalculate_total_flag){
        lpf->data->data_num[0] = lpf->params.t*lpf->params.wc*lpf->params.t*lpf->params.wc;
        lpf->data->data_num[1] = 2.0f*lpf->params.t*lpf->params.wc*lpf->params.t*lpf->params.wc;

        lpf->data->data_den[0] = 4.0f + MATH_Value_2_SQRT2*lpf->params.t*lpf->params.wc + lpf->params.t*lpf->params.wc*lpf->params.t*lpf->params.wc;
        lpf->data->data_den[1] = -8.0f + 2.0f*lpf->params.t*lpf->params.wc*lpf->params.t*lpf->params.wc;
        lpf->data->data_den[2] = 4.0f - MATH_Value_2_SQRT2*lpf->params.t*lpf->params.wc + lpf->params.t*lpf->params.wc*lpf->params.t*lpf->params.wc;
    }

    // 2.运算传递函数
    lpf->out.output = ((lpf->in.input + lpf->data->i[1])*lpf->data->data_num[0] + lpf->data->i[0]*lpf->data->data_num[1] - lpf->data->o[0]*lpf->data->data_den[1] - lpf->data->o[1]*lpf->data->data_den[2])/lpf->data->data_den[0];

    // 3.更新历史数值
    lpf->data->i[1] = lpf->data->i[0];
    lpf->data->i[0] = lpf->in.input;
    
    lpf->data->o[1] = lpf->data->o[0];
    lpf->data->o[0] = lpf->out.output;
    #endif // CONFIG_IQmath
}

// ---------------------------工程模块Transfer---------------------------



struct Hpf1Data {
    SguanQ i;
    SguanQ o;

    SguanQ data_num[2];
    SguanQ data_den[2];
};


void transfer_hpf1_init(Hpf1 *hpf){
    #if CONFIG_IQmath


    #else // CONFIG_IQmath
    // 传递函数运算系数固定计算
    hpf->data->data_num[0] = 2.0f;
    hpf->data->data_num[1] = -2.0f;

    hpf->data->data_den[0] = 2.0f + hpf->params.t*hpf->params.wc;
    hpf->data->data_den[1] = -2.0f + hpf->params.t*hpf->params.wc;

    // 初始化为零
    hpf->data->i = 0.0f;
    hpf->data->o = 0.0f;

    hpf->in.input = 0.0f;
    hpf->out.output = 0.0f;
    #endif // CONFIG_IQmath
}

void transfer_hpf1_loop(Hpf1 *hpf){
    #if CONFIG_IQmath


    #else // CONFIG_IQmath
    // 1.传递函数运算系数动态计算
    if (hpf->params.recalculate_total_flag){
        hpf->data->data_num[0] = 2.0f;
        hpf->data->data_num[1] = -2.0f;

        hpf->data->data_den[0] = 2.0f + hpf->params.t*hpf->params.wc;
        hpf->data->data_den[1] = -2.0f + hpf->params.t*hpf->params.wc;
    }

    // 2.运算传递函数
    hpf->out.output = (hpf->in.input*hpf->data->data_num[0] + hpf->data->i*hpf->data->data_num[1] - hpf->data->o*hpf->data->data_den[1])/hpf->data->data_den[0];

    // 3.更新历史数值
    hpf->data->i = hpf->in.input;
    hpf->data->o = hpf->out.output;
    #endif // CONFIG_IQmath
}

// ---------------------------工程模块Transfer---------------------------



struct Hpf2Data {
    SguanQ i[2];
    SguanQ o[2];

    // (data_num)0->现在和LLast
    // (data_num)1->仅Last的系数
    // (传递函数分子系数)
    SguanQ data_num[2];
    SguanQ data_den[3];
};


void transfer_hpf2_init(Hpf2 *hpf){
    #if CONFIG_IQmath


    #else // CONFIG_IQmath
    // 传递函数运算系数固定计算
    hpf->data->data_num[0] = 4.0f;
    hpf->data->data_num[1] = -8.0f;

    hpf->data->data_den[0] = 4.0f + MATH_Value_2_SQRT2*hpf->params.t*hpf->params.wc + hpf->params.t*hpf->params.wc*hpf->params.t*hpf->params.wc;
    hpf->data->data_den[1] = -8.0f + 2.0f*hpf->params.t*hpf->params.wc*hpf->params.t*hpf->params.wc;
    hpf->data->data_den[2] = 4.0f - MATH_Value_2_SQRT2*hpf->params.t*hpf->params.wc + hpf->params.t*hpf->params.wc*hpf->params.t*hpf->params.wc;

    // 初始化为零
    hpf->data->i[0] = 0.0f;
    hpf->data->i[1] = 0.0f;

    hpf->data->o[0] = 0.0f;
    hpf->data->o[1] = 0.0f;

    hpf->in.input = 0.0f;
    hpf->out.output = 0.0f;
    #endif // CONFIG_IQmath
}

void transfer_hpf2_loop(Hpf2 *hpf){
    #if CONFIG_IQmath


    #else // CONFIG_IQmath
    // 1.传递函数运算系数动态计算
    if (hpf->params.recalculate_total_flag){
        hpf->data->data_num[0] = 4.0f;
        hpf->data->data_num[1] = -8.0f;

        hpf->data->data_den[0] = 4.0f + MATH_Value_2_SQRT2*hpf->params.t*hpf->params.wc + hpf->params.t*hpf->params.wc*hpf->params.t*hpf->params.wc;
        hpf->data->data_den[1] = -8.0f + 2.0f*hpf->params.t*hpf->params.wc*hpf->params.t*hpf->params.wc;
        hpf->data->data_den[2] = 4.0f - MATH_Value_2_SQRT2*hpf->params.t*hpf->params.wc + hpf->params.t*hpf->params.wc*hpf->params.t*hpf->params.wc;
    }

    // 2.运算传递函数
    hpf->out.output = ((hpf->in.input + hpf->data->i[1])*hpf->data->data_num[0] + hpf->data->i[0]*hpf->data->data_num[1] - hpf->data->o[0]*hpf->data->data_den[1] - hpf->data->o[1]*hpf->data->data_den[2])/hpf->data->data_den[0];

    // 3.更新历史数值
    hpf->data->i[1] = hpf->data->i[0];
    hpf->data->i[0] = hpf->in.input;
    
    hpf->data->o[1] = hpf->data->o[0];
    hpf->data->o[0] = hpf->out.output;
    #endif // CONFIG_IQmath
}

// ---------------------------工程模块Transfer---------------------------



struct Bpf1Data {
    SguanQ i[2];
    SguanQ o[2];

    // (data_num)0->现在的系数
    // (data_num)1->LLast的系数
    // (传递函数分子系数)
    SguanQ data_num[2];
    SguanQ data_den[3];
};


void transfer_bpf1_init(Bpf1 *bpf){
    #if CONFIG_IQmath


    #else // CONFIG_IQmath
    // 传递函数运算系数固定计算
    bpf->data->data_num[0] = 2.0f*bpf->params.t*bpf->params.wc_low;
    bpf->data->data_num[1] = -2.0f*bpf->params.t*bpf->params.wc_low;

    bpf->data->data_den[0] = 4.0f + 2.0f*bpf->params.t*bpf->params.wc_high + 2.0f*bpf->params.t*bpf->params.wc_low + bpf->params.t*bpf->params.t*bpf->params.wc_high*bpf->params.wc_low;
    bpf->data->data_den[1] = -8.0f + 2.0f*bpf->params.t*bpf->params.t*bpf->params.wc_high*bpf->params.wc_low;
    bpf->data->data_den[2] = 4.0f - 2.0f*bpf->params.t*bpf->params.wc_high - 2.0f*bpf->params.t*bpf->params.wc_low + bpf->params.t*bpf->params.t*bpf->params.wc_high*bpf->params.wc_low;

    // 初始化为零
    bpf->data->i[0] = 0.0f;
    bpf->data->i[1] = 0.0f;

    bpf->data->o[0] = 0.0f;
    bpf->data->o[1] = 0.0f;
    
    bpf->in.input = 0.0f;
    bpf->out.output = 0.0f;
    #endif // CONFIG_IQmath
}

void transfer_bpf1_loop(Bpf1 *bpf){
    #if CONFIG_IQmath


    #else // CONFIG_IQmath
    // 1.传递函数运算系数动态计算
    if (bpf->params.recalculate_total_flag){
        bpf->data->data_num[0] = 2.0f*bpf->params.t*bpf->params.wc_low;
        bpf->data->data_num[1] = -2.0f*bpf->params.t*bpf->params.wc_low;

        bpf->data->data_den[0] = 4.0f + 2.0f*bpf->params.t*bpf->params.wc_high + 2.0f*bpf->params.t*bpf->params.wc_low + bpf->params.t*bpf->params.t*bpf->params.wc_high*bpf->params.wc_low;
        bpf->data->data_den[1] = -8.0f + 2.0f*bpf->params.t*bpf->params.t*bpf->params.wc_high*bpf->params.wc_low;
        bpf->data->data_den[2] = 4.0f - 2.0f*bpf->params.t*bpf->params.wc_high - 2.0f*bpf->params.t*bpf->params.wc_low + bpf->params.t*bpf->params.t*bpf->params.wc_high*bpf->params.wc_low;
    }

    // 2.运算传递函数
    bpf->out.output = (bpf->in.input*bpf->data->data_num[0] + bpf->data->i[1]*bpf->data->data_num[1] - bpf->data->o[0]*bpf->data->data_den[1] - bpf->data->o[1]*bpf->data->data_den[2])/bpf->data->data_den[0];

    // 3.更新历史数值
    bpf->data->i[1] = bpf->data->i[0];
    bpf->data->i[0] = bpf->in.input;

    bpf->data->o[1] = bpf->data->o[0];
    bpf->data->o[0] = bpf->out.output;
    #endif // CONFIG_IQmath
}

// ---------------------------工程模块Transfer---------------------------



struct Bpf2Data {
    SguanQ i[2];
    SguanQ o[2];

    // (data_num)0->现在的系数
    // (data_num)1->LLast的系数
    // (传递函数分子系数)
    SguanQ data_num[2];
    SguanQ data_den[3];
};


void transfer_bpf2_init(Bpf2 *bpf){
    #if CONFIG_IQmath


    #else // CONFIG_IQmath
    // 传递函数运算系数固定计算
    bpf->data->data_num[0] = 4.0f*bpf->params.t*bpf->params.wo*bpf->params.zeta;
    bpf->data->data_num[1] = -4.0f*bpf->params.t*bpf->params.wo*bpf->params.zeta;

    bpf->data->data_den[0] = 4.0f + 4.0f*bpf->params.t*bpf->params.wo*bpf->params.zeta + bpf->params.t*bpf->params.wo*bpf->params.t*bpf->params.wo;
    bpf->data->data_den[1] = -8.0f + 2.0f*bpf->params.t*bpf->params.wo*bpf->params.t*bpf->params.wo;
    bpf->data->data_den[2] = 4.0f - 4.0f*bpf->params.t*bpf->params.wo*bpf->params.zeta + bpf->params.t*bpf->params.wo*bpf->params.t*bpf->params.wo;

    // 初始化为零
    bpf->data->i[0] = 0.0f;
    bpf->data->i[1] = 0.0f;

    bpf->data->o[0] = 0.0f;
    bpf->data->o[1] = 0.0f;
    
    bpf->in.input = 0.0f;
    bpf->out.output = 0.0f;
    #endif // CONFIG_IQmath
}

void transfer_bpf2_loop(Bpf2 *bpf){
    #if CONFIG_IQmath


    #else // CONFIG_IQmath
    // 1.传递函数运算系数动态计算
    if (bpf->params.recalculate_total_flag){
        bpf->data->data_num[0] = 4.0f*bpf->params.t*bpf->params.wo*bpf->params.zeta;
        bpf->data->data_num[1] = -4.0f*bpf->params.t*bpf->params.wo*bpf->params.zeta;

        bpf->data->data_den[0] = 4.0f + 4.0f*bpf->params.t*bpf->params.wo*bpf->params.zeta + bpf->params.t*bpf->params.wo*bpf->params.t*bpf->params.wo;
        bpf->data->data_den[1] = -8.0f + 2.0f*bpf->params.t*bpf->params.wo*bpf->params.t*bpf->params.wo;
        bpf->data->data_den[2] = 4.0f - 4.0f*bpf->params.t*bpf->params.wo*bpf->params.zeta + bpf->params.t*bpf->params.wo*bpf->params.t*bpf->params.wo;
    }

    // 2.运算传递函数
    bpf->out.output = (bpf->in.input*bpf->data->data_num[0] + bpf->data->i[1]*bpf->data->data_num[1] - bpf->data->o[0]*bpf->data->data_den[1] - bpf->data->o[1]*bpf->data->data_den[2])/bpf->data->data_den[0];

    // 3.更新历史数值
    bpf->data->i[1] = bpf->data->i[0];
    bpf->data->i[0] = bpf->in.input;

    bpf->data->o[1] = bpf->data->o[0];
    bpf->data->o[0] = bpf->out.output;
    #endif // CONFIG_IQmath
}

// ---------------------------工程模块Transfer---------------------------



struct NfData {
    SguanQ i[2];
    SguanQ o[2];

    // (data_num)0->现在和LLast
    // (data_num)1->仅Last的系数
    // (传递函数分子系数)
    SguanQ data_num[2];
    SguanQ data_den[3];
};


void transfer_nf_init(Nf *nf){
    #if CONFIG_IQmath


    #else // CONFIG_IQmath
    // 传递函数运算系数固定计算
    nf->data->data_num[0] = 4.0f + nf->params.t*nf->params.wo*nf->params.t*nf->params.wo;
    nf->data->data_num[1] = -8.0f + 2.0f*nf->params.t*nf->params.wo*nf->params.t*nf->params.wo;

    nf->data->data_den[0] = 4.0f + 4.0f*nf->params.t*nf->params.wo*nf->params.zeta + nf->params.t*nf->params.wo*nf->params.t*nf->params.wo;
    nf->data->data_den[1] = -8.0f + 2.0f*nf->params.t*nf->params.wo*nf->params.t*nf->params.wo;
    nf->data->data_den[2] = 4.0f - 4.0f*nf->params.t*nf->params.wo*nf->params.zeta + nf->params.t*nf->params.wo*nf->params.t*nf->params.wo;

    // 初始化为零
    nf->data->i[0] = 0.0f;
    nf->data->i[1] = 0.0f;

    nf->data->o[0] = 0.0f;
    nf->data->o[1] = 0.0f;

    nf->in.input = 0.0f;
    nf->out.output = 0.0f;
    #endif // CONFIG_IQmath
}

void transfer_nf_loop(Nf *nf){
    #if CONFIG_IQmath


    #else // CONFIG_IQmath
    // 1.传递函数运算系数动态计算
    if (nf->params.recalculate_total_flag){
        nf->data->data_num[0] = 4.0f + nf->params.t*nf->params.wo*nf->params.t*nf->params.wo;
        nf->data->data_num[1] = -8.0f + 2.0f*nf->params.t*nf->params.wo*nf->params.t*nf->params.wo;

        nf->data->data_den[0] = 4.0f + 4.0f*nf->params.t*nf->params.wo*nf->params.zeta + nf->params.t*nf->params.wo*nf->params.t*nf->params.wo;
        nf->data->data_den[1] = -8.0f + 2.0f*nf->params.t*nf->params.wo*nf->params.t*nf->params.wo;
        nf->data->data_den[2] = 4.0f - 4.0f*nf->params.t*nf->params.wo*nf->params.zeta + nf->params.t*nf->params.wo*nf->params.t*nf->params.wo;
    }

    // 2.运算传递函数
    nf->out.output = ((nf->in.input + nf->data->i[1])*nf->data->data_num[0] + nf->data->i[0]*nf->data->data_num[1] - nf->data->o[0]*nf->data->data_den[1] - nf->data->o[1]*nf->data->data_den[2])/nf->data->data_den[0];

    // 3.更新历史数值
    nf->data->i[1] = nf->data->i[0];
    nf->data->i[0] = nf->in.input;
    
    nf->data->o[1] = nf->data->o[0];
    nf->data->o[0] = nf->out.output;
    #endif // CONFIG_IQmath
}

// ---------------------------工程模块Transfer---------------------------



struct TpnfData {
    SguanQ i[2];
    SguanQ o[2];

    SguanQ data_num[3];
    SguanQ data_den[3];
};


void transfer_tpnf_init(Tpnf *tpnf){
    #if CONFIG_IQmath


    #else // CONFIG_IQmath
    // 传递函数运算系数固定计算
    tpnf->data->data_num[0] = 4.0f + 4.0f*tpnf->params.k2*tpnf->params.t*tpnf->params.wo + tpnf->params.t*tpnf->params.wo*tpnf->params.t*tpnf->params.wo;
    tpnf->data->data_num[1] = -8.0f + 2.0f*tpnf->params.t*tpnf->params.wo*tpnf->params.t*tpnf->params.wo;
    tpnf->data->data_num[2] = 4.0f - 4.0f*tpnf->params.k2*tpnf->params.t*tpnf->params.wo + tpnf->params.t*tpnf->params.wo*tpnf->params.t*tpnf->params.wo;

    tpnf->data->data_den[0] = 4.0f + 4.0f*tpnf->params.k1*tpnf->params.t*tpnf->params.wo + tpnf->params.t*tpnf->params.wo*tpnf->params.t*tpnf->params.wo;
    tpnf->data->data_den[1] = -8.0f + 2.0f*tpnf->params.t*tpnf->params.wo*tpnf->params.t*tpnf->params.wo;
    tpnf->data->data_den[2] = 4.0f - 4.0f*tpnf->params.k1*tpnf->params.t*tpnf->params.wo + tpnf->params.t*tpnf->params.wo*tpnf->params.t*tpnf->params.wo;

    // 初始化为零
    tpnf->data->i[0] = 0.0f;
    tpnf->data->i[1] = 0.0f;

    tpnf->data->o[0] = 0.0f;
    tpnf->data->o[1] = 0.0f;

    tpnf->in.input = 0.0f;
    tpnf->out.output = 0.0f;
    #endif // CONFIG_IQmath
}

void transfer_tpnf_loop(Tpnf *tpnf){
    #if CONFIG_IQmath


    #else // CONFIG_IQmath
    // 1.传递函数运算系数动态计算
    if (tpnf->params.recalculate_total_flag){
        tpnf->data->data_num[0] = 4.0f + 4.0f*tpnf->params.k2*tpnf->params.t*tpnf->params.wo + tpnf->params.t*tpnf->params.wo*tpnf->params.t*tpnf->params.wo;
        tpnf->data->data_num[1] = -8.0f + 2.0f*tpnf->params.t*tpnf->params.wo*tpnf->params.t*tpnf->params.wo;
        tpnf->data->data_num[2] = 4.0f - 4.0f*tpnf->params.k2*tpnf->params.t*tpnf->params.wo + tpnf->params.t*tpnf->params.wo*tpnf->params.t*tpnf->params.wo;

        tpnf->data->data_den[0] = 4.0f + 4.0f*tpnf->params.k1*tpnf->params.t*tpnf->params.wo + tpnf->params.t*tpnf->params.wo*tpnf->params.t*tpnf->params.wo;
        tpnf->data->data_den[1] = -8.0f + 2.0f*tpnf->params.t*tpnf->params.wo*tpnf->params.t*tpnf->params.wo;
        tpnf->data->data_den[2] = 4.0f - 4.0f*tpnf->params.k1*tpnf->params.t*tpnf->params.wo + tpnf->params.t*tpnf->params.wo*tpnf->params.t*tpnf->params.wo;
    }

    // 2.运算传递函数
    tpnf->out.output = (tpnf->in.input*tpnf->data->data_num[0] + tpnf->data->i[0]*tpnf->data->data_num[1] + tpnf->data->i[1]*tpnf->data->data_num[2] - tpnf->data->o[0]*tpnf->data->data_den[1] - tpnf->data->o[1]*tpnf->data->data_den[2])/tpnf->data->data_den[0];

    // 3.更新历史数值
    tpnf->data->i[1] = tpnf->data->i[0];
    tpnf->data->i[0] = tpnf->in.input;
    
    tpnf->data->o[1] = tpnf->data->o[0];
    tpnf->data->o[0] = tpnf->out.output;
    #endif // CONFIG_IQmath
}

// ---------------------------工程模块Transfer---------------------------



struct DobData {
    SguanQ num;
    SguanQ den;
};


void transfer_dob_init(Dob *dob){
    #if CONFIG_IQmath


    #else // CONFIG_IQmath


    
    #endif // CONFIG_IQmath
}

void transfer_dob_loop(Dob *dob){
    #if CONFIG_IQmath


    #else // CONFIG_IQmath


    

    #endif // CONFIG_IQmath
}

// ---------------------------工程模块Transfer---------------------------



struct RlsData {
    SguanQ num;
    SguanQ den;
};


void transfer_rls_init(Rls *rls){
    #if CONFIG_IQmath


    #else // CONFIG_IQmath


    
    #endif // CONFIG_IQmath
}

void transfer_rls_loop(Rls *rls){
    #if CONFIG_IQmath


    #else // CONFIG_IQmath


    

    #endif // CONFIG_IQmath
}

// ---------------------------工程模块Transfer---------------------------



struct SmoData {
    SguanQ num;
    SguanQ den;
};


void transfer_smo_init(Smo *smo){
    #if CONFIG_IQmath


    #else // CONFIG_IQmath


    
    #endif // CONFIG_IQmath
}

void transfer_smo_loop(Smo *smo){
    #if CONFIG_IQmath


    #else // CONFIG_IQmath


    

    #endif // CONFIG_IQmath
}

// ---------------------------工程模块Transfer---------------------------



struct NlfoData {
    SguanQ num;
    SguanQ den;
};


void transfer_nlfo_init(Nlfo *nlfo){
    #if CONFIG_IQmath


    #else // CONFIG_IQmath


    
    #endif // CONFIG_IQmath
}

void transfer_nlfo_loop(Nlfo *nlfo){
    #if CONFIG_IQmath


    #else // CONFIG_IQmath


    

    #endif // CONFIG_IQmath
}
// ---------------------------工程模块Transfer---------------------------



struct VcfoData {
    SguanQ num;
    SguanQ den;
};


void transfer_vcfo_init(Vcfo *vcfo){
    #if CONFIG_IQmath


    #else // CONFIG_IQmath


    
    #endif // CONFIG_IQmath
}

void transfer_vcfo_loop(Vcfo *vcfo){
    #if CONFIG_IQmath


    #else // CONFIG_IQmath


    

    #endif // CONFIG_IQmath
}

// ---------------------------工程模块Transfer---------------------------



struct HfiData {
    SguanQ num;
    SguanQ den;
};


void transfer_hfi_init(Hfi *hfi){
    #if CONFIG_IQmath


    #else // CONFIG_IQmath


    
    #endif // CONFIG_IQmath
}

void transfer_hfi_loop(Hfi *hfi){
    #if CONFIG_IQmath


    #else // CONFIG_IQmath


    

    #endif // CONFIG_IQmath
}

// ---------------------------工程模块Transfer---------------------------



struct RoloData {
    SguanQ num;
    SguanQ den;
};


void transfer_rolo_init(Rolo *rolo){
    #if CONFIG_IQmath


    #else // CONFIG_IQmath


    
    #endif // CONFIG_IQmath
}

void transfer_rolo_loop(Rolo *rolo){
    #if CONFIG_IQmath


    #else // CONFIG_IQmath


    

    #endif // CONFIG_IQmath
}

// ---------------------------工程模块Transfer---------------------------



struct MarsData {
    SguanQ num;
    SguanQ den;
};


void transfer_mars_init(Mars *mars){
    #if CONFIG_IQmath


    #else // CONFIG_IQmath


    
    #endif // CONFIG_IQmath
}

void transfer_mars_loop(Mars *mars){
    #if CONFIG_IQmath


    #else // CONFIG_IQmath


    

    #endif // CONFIG_IQmath
}

// ---------------------------工程模块Transfer---------------------------



struct EkfData {
    SguanQ num;
    SguanQ den;
};


void transfer_ekf_init(Ekf *ekf){
    #if CONFIG_IQmath


    #else // CONFIG_IQmath


    
    #endif // CONFIG_IQmath
}

void transfer_ekf_loop(Ekf *ekf){
    #if CONFIG_IQmath


    #else // CONFIG_IQmath


    

    #endif // CONFIG_IQmath
}

// ---------------------------工程模块Transfer---------------------------



void transfer_delay1_init(Delay1 *delay){
    #if CONFIG_IQmath


    #else // CONFIG_IQmath
    // 初始化为零
    delay->in.input = 0.0f;
    delay->out.output = 0.0f;
    #endif // CONFIG_IQmath
}

void transfer_delay1_loop(Delay1 *delay){
    #if CONFIG_IQmath


    #else // CONFIG_IQmath
    // 1.创建局部静态变量
    static SguanQ delay_num;

    // 2.运算结果
    delay->out.output = delay_num;

    // 3.更新历史数值
    delay_num = delay->in.input;
    #endif // CONFIG_IQmath
}

// ---------------------------工程模块Transfer---------------------------



void transfer_delay2_init(Delay2 *delay){
    #if CONFIG_IQmath


    #else // CONFIG_IQmath
    // 初始化为零
    delay->in.input = 0.0f;
    delay->out.output = 0.0f;
    #endif // CONFIG_IQmath
}

void transfer_delay2_loop(Delay2 *delay){
    #if CONFIG_IQmath


    #else // CONFIG_IQmath
    // 1.创建局部静态变量
    static SguanQ delay_num[2];

    // 2.运算结果
    delay->out.output = delay_num[1];

    // 3.更新历史数值
    delay_num[1] = delay_num[0];
    delay_num[0] = delay->in.input;
    #endif // CONFIG_IQmath
}

// ---------------------------工程模块Transfer---------------------------



void transfer_delay3_init(Delay3 *delay){
    #if CONFIG_IQmath


    #else // CONFIG_IQmath
    // 初始化为零
    delay->in.input = 0.0f;
    delay->out.output = 0.0f;
    #endif // CONFIG_IQmath
}

void transfer_delay3_loop(Delay3 *delay){
    #if CONFIG_IQmath


    #else // CONFIG_IQmath
    // 1.创建局部静态变量
    static SguanQ delay_num[3];

    // 2.运算结果
    delay->out.output = delay_num[2];

    // 3.更新历史数值
    delay_num[2] = delay_num[1];
    delay_num[1] = delay_num[0];
    delay_num[0] = delay->in.input;  
    #endif // CONFIG_IQmath
}

// ---------------------------工程模块Transfer---------------------------



void transfer_sine_loop(Sine *sine){
    #if CONFIG_IQmath


    #else // CONFIG_IQmath
    sine->out.output = Math_sin(sine->in.input);
    #endif // CONFIG_IQmath
}

// ---------------------------工程模块Transfer---------------------------



void transfer_cosine_loop(Cosine *cosine){
    #if CONFIG_IQmath


    #else // CONFIG_IQmath
    cosine->out.output = Math_cos(cosine->in.input);
    #endif // CONFIG_IQmath
}

// ---------------------------工程模块Transfer---------------------------



void transfer_sincos_loop(SinCos *sincos){
    #if CONFIG_IQmath


    #else // CONFIG_IQmath
    Math_sin_cos(sincos->in.input, 
        &sincos->out.sine, 
        &sincos->out.cosine);
    #endif // CONFIG_IQmath
}

// ---------------------------工程模块Transfer---------------------------



void transfer_sign_loop(Sign *sign){
    #if CONFIG_IQmath


    #else // CONFIG_IQmath


    

    #endif // CONFIG_IQmath
}

// ---------------------------工程模块Transfer---------------------------



void transfer_clarke_loop(Clarke *clarke){
    #if CONFIG_IQmath


    #else // CONFIG_IQmath


    

    #endif // CONFIG_IQmath
}

// ---------------------------工程模块Transfer---------------------------



void transfer_park_loop(Park *park){
    #if CONFIG_IQmath


    #else // CONFIG_IQmath


    

    #endif // CONFIG_IQmath
}

// ---------------------------工程模块Transfer---------------------------



void transfer_ipark_loop(Ipark *ipark){
    #if CONFIG_IQmath


    #else // CONFIG_IQmath


    

    #endif // CONFIG_IQmath
}

// ---------------------------工程模块Transfer---------------------------



void transfer_spwm0_loop(Spwm0 *spwm){
    #if CONFIG_IQmath


    #else // CONFIG_IQmath


    

    #endif // CONFIG_IQmath
}

// ---------------------------工程模块Transfer---------------------------



void transfer_spwm_loop(Spwm *spwm){
    #if CONFIG_IQmath


    #else // CONFIG_IQmath


    

    #endif // CONFIG_IQmath
}

// ---------------------------工程模块Transfer---------------------------



void transfer_svpwm_loop(Svpwm *svpwm){
    #if CONFIG_IQmath


    #else // CONFIG_IQmath


    

    #endif // CONFIG_IQmath
}

// ---------------------------工程模块Transfer---------------------------



void transfer_swpwm_loop(Swpwm *swpwm){

}

// ---------------------------工程模块Transfer---------------------------





void transfer_singlers_loop(SingleRs *singlers){
    #if CONFIG_IQmath


    #else // CONFIG_IQmath


    

    #endif // CONFIG_IQmath
}

// ---------------------------工程模块Transfer---------------------------
void transfer_reinit_integrator(void *p){
    #if CONFIG_IQmath


    #else // CONFIG_IQmath



    #endif // CONFIG_IQmath
}

// ==================== 实例池：块实例统一住在这里，用户通过 _get() 拿指针 ====================
#if CONFIG_TRANSFER1
static Transfer1 transfer1_pool[CONFIG_MOTOR][CONFIG_TRANSFER1];
static Transfer1Data transfer1_data_pool[CONFIG_MOTOR][CONFIG_TRANSFER1];
Transfer1 *transfer_transfer1_get(uint8_t motor, int ch) {
    if (motor >= CONFIG_MOTOR || ch >= CONFIG_TRANSFER1) return 0;
    Transfer1 *self = &transfer1_pool[motor][ch];
    self->data = &transfer1_data_pool[motor][ch];
    return self;
}
#endif
#if CONFIG_TRANSFER2
static Transfer2 transfer2_pool[CONFIG_MOTOR][CONFIG_TRANSFER2];
static Transfer2Data transfer2_data_pool[CONFIG_MOTOR][CONFIG_TRANSFER2];
Transfer2 *transfer_transfer2_get(uint8_t motor, int ch) {
    if (motor >= CONFIG_MOTOR || ch >= CONFIG_TRANSFER2) return 0;
    Transfer2 *self = &transfer2_pool[motor][ch];
    self->data = &transfer2_data_pool[motor][ch];
    return self;
}
#endif
#if CONFIG_TRANSFER3
static Transfer3 transfer3_pool[CONFIG_MOTOR][CONFIG_TRANSFER3];
static Transfer3Data transfer3_data_pool[CONFIG_MOTOR][CONFIG_TRANSFER3];
Transfer3 *transfer_transfer3_get(uint8_t motor, int ch) {
    if (motor >= CONFIG_MOTOR || ch >= CONFIG_TRANSFER3) return 0;
    Transfer3 *self = &transfer3_pool[motor][ch];
    self->data = &transfer3_data_pool[motor][ch];
    return self;
}
#endif
#if CONFIG_TRANSFER4
static Transfer4 transfer4_pool[CONFIG_MOTOR][CONFIG_TRANSFER4];
static Transfer4Data transfer4_data_pool[CONFIG_MOTOR][CONFIG_TRANSFER4];
Transfer4 *transfer_transfer4_get(uint8_t motor, int ch) {
    if (motor >= CONFIG_MOTOR || ch >= CONFIG_TRANSFER4) return 0;
    Transfer4 *self = &transfer4_pool[motor][ch];
    self->data = &transfer4_data_pool[motor][ch];
    return self;
}
#endif
#if CONFIG_TRANSFER5
static Transfer5 transfer5_pool[CONFIG_MOTOR][CONFIG_TRANSFER5];
static Transfer5Data transfer5_data_pool[CONFIG_MOTOR][CONFIG_TRANSFER5];
Transfer5 *transfer_transfer5_get(uint8_t motor, int ch) {
    if (motor >= CONFIG_MOTOR || ch >= CONFIG_TRANSFER5) return 0;
    Transfer5 *self = &transfer5_pool[motor][ch];
    self->data = &transfer5_data_pool[motor][ch];
    return self;
}
#endif
#if CONFIG_INTEGRATOR
static Integrator integrator_pool[CONFIG_MOTOR][CONFIG_INTEGRATOR];
static IntegratorData integrator_data_pool[CONFIG_MOTOR][CONFIG_INTEGRATOR];
Integrator *transfer_integrator_get(uint8_t motor, int ch) {
    if (motor >= CONFIG_MOTOR || ch >= CONFIG_INTEGRATOR) return 0;
    Integrator *self = &integrator_pool[motor][ch];
    self->data = &integrator_data_pool[motor][ch];
    return self;
}
#endif
#if CONFIG_DERIVATIVE
static Derivative derivative_pool[CONFIG_MOTOR][CONFIG_DERIVATIVE];
static DerivativeData derivative_data_pool[CONFIG_MOTOR][CONFIG_DERIVATIVE];
Derivative *transfer_derivative_get(uint8_t motor, int ch) {
    if (motor >= CONFIG_MOTOR || ch >= CONFIG_DERIVATIVE) return 0;
    Derivative *self = &derivative_pool[motor][ch];
    self->data = &derivative_data_pool[motor][ch];
    return self;
}
#endif
#if CONFIG_DERIVATIVE
static Dft dft_pool[CONFIG_MOTOR][CONFIG_DFT];
static DftData dft_data_pool[CONFIG_MOTOR][CONFIG_DFT];
Dft *transfer_dft_get(uint8_t motor, int ch) {
    if (motor >= CONFIG_MOTOR || ch >= CONFIG_DFT) return 0;
    Dft *self = &dft_pool[motor][ch];
    self->data = &dft_data_pool[motor][ch];
    return self;
}
#endif
#if CONFIG_HALL
static Hall hall_pool[CONFIG_MOTOR][CONFIG_HALL];
static HallData hall_data_pool[CONFIG_MOTOR][CONFIG_HALL];
Hall *transfer_hall_get(uint8_t motor, int ch) {
    if (motor >= CONFIG_MOTOR || ch >= CONFIG_HALL) return 0;
    Hall *self = &hall_pool[motor][ch];
    self->data = &hall_data_pool[motor][ch];
    return self;
}
#endif
#if CONFIG_LADRC1
static Ladrc1 ladrc1_pool[CONFIG_MOTOR][CONFIG_LADRC1];
static Ladrc1Data ladrc1_data_pool[CONFIG_MOTOR][CONFIG_LADRC1];
Ladrc1 *transfer_ladrc1_get(uint8_t motor, int ch) {
    if (motor >= CONFIG_MOTOR || ch >= CONFIG_LADRC1) return 0;
    Ladrc1 *self = &ladrc1_pool[motor][ch];
    self->data = &ladrc1_data_pool[motor][ch];
    return self;
}
#endif
#if CONFIG_LADRC2
static Ladrc2 ladrc2_pool[CONFIG_MOTOR][CONFIG_LADRC2];
static Ladrc2Data ladrc2_data_pool[CONFIG_MOTOR][CONFIG_LADRC2];
Ladrc2 *transfer_ladrc2_get(uint8_t motor, int ch) {
    if (motor >= CONFIG_MOTOR || ch >= CONFIG_LADRC2) return 0;
    Ladrc2 *self = &ladrc2_pool[motor][ch];
    self->data = &ladrc2_data_pool[motor][ch];
    return self;
}
#endif
#if CONFIG_SMC
static Smc smc_pool[CONFIG_MOTOR][CONFIG_SMC];
static SmcData smc_data_pool[CONFIG_MOTOR][CONFIG_SMC];
Smc *transfer_smc_get(uint8_t motor, int ch) {
    if (motor >= CONFIG_MOTOR || ch >= CONFIG_SMC) return 0;
    Smc *self = &smc_pool[motor][ch];
    self->data = &smc_data_pool[motor][ch];
    return self;
}
#endif
#if CONFIG_DPCC
static Dpcc dpcc_pool[CONFIG_MOTOR][CONFIG_DPCC];
static DpccData dpcc_data_pool[CONFIG_MOTOR][CONFIG_DPCC];
Dpcc *transfer_dpcc_get(uint8_t motor, int ch) {
    if (motor >= CONFIG_MOTOR || ch >= CONFIG_DPCC) return 0;
    Dpcc *self = &dpcc_pool[motor][ch];
    self->data = &dpcc_data_pool[motor][ch];
    return self;
}
#endif
#if CONFIG_PIR
static Pir pir_pool[CONFIG_MOTOR][CONFIG_PIR];
static PirData pir_data_pool[CONFIG_MOTOR][CONFIG_PIR];
Pir *transfer_pir_get(uint8_t motor, int ch) {
    if (motor >= CONFIG_MOTOR || ch >= CONFIG_PIR) return 0;
    Pir *self = &pir_pool[motor][ch];
    self->data = &pir_data_pool[motor][ch];
    return self;
}
#endif
#if CONFIG_PID
static Pid pid_pool[CONFIG_MOTOR][CONFIG_PID];
static PidData pid_data_pool[CONFIG_MOTOR][CONFIG_PID];
Pid *transfer_pid_get(uint8_t motor, int ch) {
    if (motor >= CONFIG_MOTOR || ch >= CONFIG_PID) return 0;
    Pid *self = &pid_pool[motor][ch];
    self->data = &pid_data_pool[motor][ch];
    return self;
}
#endif
#if CONFIG_PLL
static Pll pll_pool[CONFIG_MOTOR][CONFIG_PLL];
static PllData pll_data_pool[CONFIG_MOTOR][CONFIG_PLL];
Pll *transfer_pll_get(uint8_t motor, int ch) {
    if (motor >= CONFIG_MOTOR || ch >= CONFIG_PLL) return 0;
    Pll *self = &pll_pool[motor][ch];
    self->data = &pll_data_pool[motor][ch];
    return self;
}
#endif
#if CONFIG_LPF1
static Lpf1 lpf1_pool[CONFIG_MOTOR][CONFIG_LPF1];
static Lpf1Data lpf1_data_pool[CONFIG_MOTOR][CONFIG_LPF1];
Lpf1 *transfer_lpf1_get(uint8_t motor, int ch) {
    if (motor >= CONFIG_MOTOR || ch >= CONFIG_LPF1) return 0;
    Lpf1 *self = &lpf1_pool[motor][ch];
    self->data = &lpf1_data_pool[motor][ch];
    return self;
}
#endif
#if CONFIG_LPF2
static Lpf2 lpf2_pool[CONFIG_MOTOR][CONFIG_LPF2];
static Lpf2Data lpf2_data_pool[CONFIG_MOTOR][CONFIG_LPF2];
Lpf2 *transfer_lpf2_get(uint8_t motor, int ch) {
    if (motor >= CONFIG_MOTOR || ch >= CONFIG_LPF2) return 0;
    Lpf2 *self = &lpf2_pool[motor][ch];
    self->data = &lpf2_data_pool[motor][ch];
    return self;
}
#endif
#if CONFIG_HPF1
static Hpf1 hpf1_pool[CONFIG_MOTOR][CONFIG_HPF1];
static Hpf1Data hpf1_data_pool[CONFIG_MOTOR][CONFIG_HPF1];
Hpf1 *transfer_hpf1_get(uint8_t motor, int ch) {
    if (motor >= CONFIG_MOTOR || ch >= CONFIG_HPF1) return 0;
    Hpf1 *self = &hpf1_pool[motor][ch];
    self->data = &hpf1_data_pool[motor][ch];
    return self;
}
#endif
#if CONFIG_HPF2
static Hpf2 hpf2_pool[CONFIG_MOTOR][CONFIG_HPF2];
static Hpf2Data hpf2_data_pool[CONFIG_MOTOR][CONFIG_HPF2];
Hpf2 *transfer_hpf2_get(uint8_t motor, int ch) {
    if (motor >= CONFIG_MOTOR || ch >= CONFIG_HPF2) return 0;
    Hpf2 *self = &hpf2_pool[motor][ch];
    self->data = &hpf2_data_pool[motor][ch];
    return self;
}
#endif
#if CONFIG_BPF1
static Bpf1 bpf1_pool[CONFIG_MOTOR][CONFIG_BPF1];
static Bpf1Data bpf1_data_pool[CONFIG_MOTOR][CONFIG_BPF1];
Bpf1 *transfer_bpf1_get(uint8_t motor, int ch) {
    if (motor >= CONFIG_MOTOR || ch >= CONFIG_BPF1) return 0;
    Bpf1 *self = &bpf1_pool[motor][ch];
    self->data = &bpf1_data_pool[motor][ch];
    return self;
}
#endif
#if CONFIG_BPF2
static Bpf2 bpf2_pool[CONFIG_MOTOR][CONFIG_BPF2];
static Bpf2Data bpf2_data_pool[CONFIG_MOTOR][CONFIG_BPF2];
Bpf2 *transfer_bpf2_get(uint8_t motor, int ch) {
    if (motor >= CONFIG_MOTOR || ch >= CONFIG_BPF2) return 0;
    Bpf2 *self = &bpf2_pool[motor][ch];
    self->data = &bpf2_data_pool[motor][ch];
    return self;
}
#endif
#if CONFIG_NF
static Nf nf_pool[CONFIG_MOTOR][CONFIG_NF];
static NfData nf_data_pool[CONFIG_MOTOR][CONFIG_NF];
Nf *transfer_nf_get(uint8_t motor, int ch) {
    if (motor >= CONFIG_MOTOR || ch >= CONFIG_NF) return 0;
    Nf *self = &nf_pool[motor][ch];
    self->data = &nf_data_pool[motor][ch];
    return self;
}
#endif
#if CONFIG_TPNF
static Tpnf tpnf_pool[CONFIG_MOTOR][CONFIG_TPNF];
static TpnfData tpnf_data_pool[CONFIG_MOTOR][CONFIG_TPNF];
Tpnf *transfer_tpnf_get(uint8_t motor, int ch) {
    if (motor >= CONFIG_MOTOR || ch >= CONFIG_TPNF) return 0;
    Tpnf *self = &tpnf_pool[motor][ch];
    self->data = &tpnf_data_pool[motor][ch];
    return self;
}
#endif
#if CONFIG_DOB
static Dob dob_pool[CONFIG_MOTOR][CONFIG_DOB];
static DobData dob_data_pool[CONFIG_MOTOR][CONFIG_DOB];
Dob *transfer_dob_get(uint8_t motor, int ch) {
    if (motor >= CONFIG_MOTOR || ch >= CONFIG_DOB) return 0;
    Dob *self = &dob_pool[motor][ch];
    self->data = &dob_data_pool[motor][ch];
    return self;
}
#endif
#if CONFIG_RLS
static Rls rls_pool[CONFIG_MOTOR][CONFIG_RLS];
static RlsData rls_data_pool[CONFIG_MOTOR][CONFIG_RLS];
Rls *transfer_rls_get(uint8_t motor, int ch) {
    if (motor >= CONFIG_MOTOR || ch >= CONFIG_RLS) return 0;
    Rls *self = &rls_pool[motor][ch];
    self->data = &rls_data_pool[motor][ch];
    return self;
}
#endif
#if CONFIG_SMO
static Smo smo_pool[CONFIG_MOTOR][CONFIG_SMO];
static SmoData smo_data_pool[CONFIG_MOTOR][CONFIG_SMO];
Smo *transfer_smo_get(uint8_t motor, int ch) {
    if (motor >= CONFIG_MOTOR || ch >= CONFIG_SMO) return 0;
    Smo *self = &smo_pool[motor][ch];
    self->data = &smo_data_pool[motor][ch];
    return self;
}
#endif
#if CONFIG_NLFO
static Nlfo nlfo_pool[CONFIG_MOTOR][CONFIG_NLFO];
static NlfoData nlfo_data_pool[CONFIG_MOTOR][CONFIG_NLFO];
Nlfo *transfer_nlfo_get(uint8_t motor, int ch) {
    if (motor >= CONFIG_MOTOR || ch >= CONFIG_NLFO) return 0;
    Nlfo *self = &nlfo_pool[motor][ch];
    self->data = &nlfo_data_pool[motor][ch];
    return self;
}
#endif
#if CONFIG_VCFO
static Vcfo vcfo_pool[CONFIG_MOTOR][CONFIG_VCFO];
static VcfoData vcfo_data_pool[CONFIG_MOTOR][CONFIG_VCFO];
Vcfo *transfer_vcfo_get(uint8_t motor, int ch) {
    if (motor >= CONFIG_MOTOR || ch >= CONFIG_VCFO) return 0;
    Vcfo *self = &vcfo_pool[motor][ch];
    self->data = &vcfo_data_pool[motor][ch];
    return self;
}
#endif
#if CONFIG_HFI
static Hfi hfi_pool[CONFIG_MOTOR][CONFIG_HFI];
static HfiData hfi_data_pool[CONFIG_MOTOR][CONFIG_HFI];
Hfi *transfer_hfi_get(uint8_t motor, int ch) {
    if (motor >= CONFIG_MOTOR || ch >= CONFIG_HFI) return 0;
    Hfi *self = &hfi_pool[motor][ch];
    self->data = &hfi_data_pool[motor][ch];
    return self;
}
#endif
#if CONFIG_ROLO
static Rolo rolo_pool[CONFIG_MOTOR][CONFIG_ROLO];
static RoloData rolo_data_pool[CONFIG_MOTOR][CONFIG_ROLO];
Rolo *transfer_rolo_get(uint8_t motor, int ch) {
    if (motor >= CONFIG_MOTOR || ch >= CONFIG_ROLO) return 0;
    Rolo *self = &rolo_pool[motor][ch];
    self->data = &rolo_data_pool[motor][ch];
    return self;
}
#endif
#if CONFIG_MARS
static Mars mars_pool[CONFIG_MOTOR][CONFIG_MARS];
static MarsData mars_data_pool[CONFIG_MOTOR][CONFIG_MARS];
Mars *transfer_mars_get(uint8_t motor, int ch) {
    if (motor >= CONFIG_MOTOR || ch >= CONFIG_MARS) return 0;
    Mars *self = &mars_pool[motor][ch];
    self->data = &mars_data_pool[motor][ch];
    return self;
}
#endif
#if CONFIG_EKF
static Ekf ekf_pool[CONFIG_MOTOR][CONFIG_EKF];
static EkfData ekf_data_pool[CONFIG_MOTOR][CONFIG_EKF];
Ekf *transfer_ekf_get(uint8_t motor, int ch) {
    if (motor >= CONFIG_MOTOR || ch >= CONFIG_EKF) return 0;
    Ekf *self = &ekf_pool[motor][ch];
    self->data = &ekf_data_pool[motor][ch];
    return self;
}
#endif
#if CONFIG_DELAY1
static Delay1 delay1_pool[CONFIG_MOTOR][CONFIG_DELAY1];
Delay1 *transfer_delay1_get(uint8_t motor, int ch) {
    if (motor >= CONFIG_MOTOR || ch >= CONFIG_DELAY1) return 0;
    return &delay1_pool[motor][ch];
}
#endif
#if CONFIG_DELAY2
static Delay2 delay2_pool[CONFIG_MOTOR][CONFIG_DELAY2];
Delay2 *transfer_delay2_get(uint8_t motor, int ch) {
    if (motor >= CONFIG_MOTOR || ch >= CONFIG_DELAY2) return 0;
    return &delay2_pool[motor][ch];
}
#endif
#if CONFIG_DELAY3
static Delay3 delay3_pool[CONFIG_MOTOR][CONFIG_DELAY3];
Delay3 *transfer_delay3_get(uint8_t motor, int ch) {
    if (motor >= CONFIG_MOTOR || ch >= CONFIG_DELAY3) return 0;
    return &delay3_pool[motor][ch];
}
#endif


// ==================== 单一功能模块实例（单实例） ====================
static Sine sine_pool[CONFIG_MOTOR];
Sine *transfer_sine_get(uint8_t motor) {
    if (motor >= CONFIG_MOTOR) return 0;
    return &sine_pool[motor];
}

static Cosine cosine_pool[CONFIG_MOTOR];
Cosine *transfer_cosine_get(uint8_t motor) {
    if (motor >= CONFIG_MOTOR) return 0;
    return &cosine_pool[motor];
}

static Sign sign_pool[CONFIG_MOTOR];
Sign *transfer_sign_get(uint8_t motor) {
    if (motor >= CONFIG_MOTOR) return 0;
    return &sign_pool[motor];
}

static Clarke clarke_pool[CONFIG_MOTOR];
Clarke *transfer_clarke_get(uint8_t motor) {
    if (motor >= CONFIG_MOTOR) return 0;
    return &clarke_pool[motor];
}

static Park park_pool[CONFIG_MOTOR];
Park *transfer_park_get(uint8_t motor) {
    if (motor >= CONFIG_MOTOR) return 0;
    return &park_pool[motor];
}

static Ipark ipark_pool[CONFIG_MOTOR];
Ipark *transfer_ipark_get(uint8_t motor) {
    if (motor >= CONFIG_MOTOR) return 0;
    return &ipark_pool[motor];
}

static Spwm0 spwm0_pool[CONFIG_MOTOR];
Spwm0 *transfer_spwm0_get(uint8_t motor) {
    if (motor >= CONFIG_MOTOR) return 0;
    return &spwm0_pool[motor];
}

static Spwm spwm_pool[CONFIG_MOTOR];
Spwm *transfer_spwm_get(uint8_t motor) {
    if (motor >= CONFIG_MOTOR) return 0;
    return &spwm_pool[motor];
}

static Svpwm svpwm_pool[CONFIG_MOTOR];
Svpwm *transfer_svpwm_get(uint8_t motor) {
    if (motor >= CONFIG_MOTOR) return 0;
    return &svpwm_pool[motor];
}

static SingleRs singlers_pool[CONFIG_MOTOR];
SingleRs *transfer_singlers_get(uint8_t motor) {
    if (motor >= CONFIG_MOTOR) return 0;
    return &singlers_pool[motor];
}

// ==================== 新增块：Tan/Atan/Limit + SinCos/Swpwm 实例 ====================
void transfer_tan_loop(Tan *tan){
    // TODO: 正切求解
}

void transfer_atan_loop(Atan *atan){
    // TODO: 反正切求解
}

void transfer_limit_loop(Limit *limit){
    // TODO: 限幅函数
}

static SinCos sincos_pool[CONFIG_MOTOR];
SinCos *transfer_sincos_get(uint8_t motor) {
    if (motor >= CONFIG_MOTOR) return 0;
    return &sincos_pool[motor];
}

static Swpwm swpwm_pool[CONFIG_MOTOR];
Swpwm *transfer_swpwm_get(uint8_t motor) {
    if (motor >= CONFIG_MOTOR) return 0;
    return &swpwm_pool[motor];
}

static Tan tan_pool[CONFIG_MOTOR];
Tan *transfer_tan_get(uint8_t motor) {
    if (motor >= CONFIG_MOTOR) return 0;
    return &tan_pool[motor];
}

static Atan atan_pool[CONFIG_MOTOR];
Atan *transfer_atan_get(uint8_t motor) {
    if (motor >= CONFIG_MOTOR) return 0;
    return &atan_pool[motor];
}

static Limit limit_pool[CONFIG_MOTOR];
Limit *transfer_limit_get(uint8_t motor) {
    if (motor >= CONFIG_MOTOR) return 0;
    return &limit_pool[motor];
}

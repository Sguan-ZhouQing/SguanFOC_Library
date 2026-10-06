/*
 * @Author: 星必尘Sguan
 * @GitHub: https://github.com/Sguan-ZhouQing
 * @Date: 2026-03-20 10:21:01
 * @LastEditors: 星必尘Sguan|3464647102@qq.com
 * @LastEditTime: 2026-04-01 03:44:41
 * @FilePath: \SguanFOC_Debug\SguanFOC\Sguan_SVPWM.c
 * @Description: SguanFOC库的“七段式SVPWM空间矢量合成”实现
 * 
 * Copyright (c) 2026 by $星必尘Sguan, All Rights Reserved. 
 */
#include "Sguan_SVPWM.h"

// 常量宏定义声明
#define Value_INV_SQRT3_q31 1239850240
#define Value_SQRT3_2_q31   1859775393

uint8_t sector_SVPWM = 0;

// 这里输入的alpha和beta轴的数据都是归一化的Q31数值
// (使用前可以调用iqmath_div得到归一化数值)
void SVPWM(Q31_t u_alpha, Q31_t u_beta, 
        Q31_t *d_u, Q31_t *d_v, Q31_t *d_w){
    static const Q31_t ts = Q31_MAX;

    Q31_t u1 = u_beta;
    Q31_t u2 = iqmath_mul(-Value_SQRT3_2_q31,u_alpha) - iqmath_mul(Q31_HALF,u_beta);
    Q31_t u3 = iqmath_mul(Value_SQRT3_2_q31,u_alpha) - iqmath_mul(Q31_HALF,u_beta);
    
    uint8_t sector = (u1 > 0) + ((u2 > 0) << 1) + ((u3 > 0) << 2);
    
    Q31_t t_a, t_b, t_c;
    Q31_t k_svpwm;
    if (sector == 5) {
        sector_SVPWM = 1;

        Q31_t t4 = u3;
        Q31_t t6 = u1;
        Q31_t sum = t4 + t6;
        if (sum > ts) {
            k_svpwm = iqmath_div(ts,sum);
            t4 = iqmath_mul(k_svpwm,t4);
            t6 = iqmath_mul(k_svpwm,t6);
        }
        Q31_t t7 = (ts - t4 - t6) / 2;
        t_a = t4 + t6 + t7;
        t_b = t6 + t7;
        t_c = t7;
    } else if (sector == 1) {
        sector_SVPWM = 2;

        Q31_t t2 = -u3;
        Q31_t t6 = -u2;
        Q31_t sum = t2 + t6;
        if (sum > ts) {
            k_svpwm = iqmath_div(ts,sum);
            t2 = iqmath_mul(k_svpwm,t2);
            t6 = iqmath_mul(k_svpwm,t6);
        }
        Q31_t t7 = (ts - t2 - t6) / 2;
        t_a = t6 + t7;
        t_b = t2 + t6 + t7;
        t_c = t7;
    } else if (sector == 3) {
        sector_SVPWM = 3;

        Q31_t t2 = u1;
        Q31_t t3 = u2;
        Q31_t sum = t2 + t3;
        if (sum > ts) {
            k_svpwm = iqmath_div(ts,sum);
            t2 = iqmath_mul(k_svpwm,t2);
            t3 = iqmath_mul(k_svpwm,t3);
        }
        Q31_t t7 = (ts - t2 - t3) / 2;
        t_a = t7;
        t_b = t2 + t3 + t7;
        t_c = t3 + t7;
    } else if (sector == 2) {
        sector_SVPWM = 4;

        Q31_t t1 = -u1;
        Q31_t t3 = -u3;
        Q31_t sum = t1 + t3;
        if (sum > ts) {
            k_svpwm = iqmath_div(ts,sum);
            t1 = iqmath_mul(k_svpwm,t1);
            t3 = iqmath_mul(k_svpwm,t3);
        }
        Q31_t t7 = (ts - t1 - t3) / 2;
        t_a = t7;
        t_b = t3 + t7;
        t_c = t1 + t3 + t7;
    } else if (sector == 6) {
        sector_SVPWM = 5;

        Q31_t t1 = u2;
        Q31_t t5 = u3;
        Q31_t sum = t1 + t5;
        if (sum > ts) {
            k_svpwm = iqmath_div(ts,sum);
            t1 = iqmath_mul(k_svpwm,t1);
            t5 = iqmath_mul(k_svpwm,t5);
        }
        Q31_t t7 = (ts - t1 - t5) / 2;
        t_a = t5 + t7;
        t_b = t7;
        t_c = t1 + t5 + t7;
    } else if (sector == 4) {
        sector_SVPWM = 6;

        Q31_t t4 = -u2;
        Q31_t t5 = -u1;
        Q31_t sum = t4 + t5;
        if (sum > ts) {
            k_svpwm = iqmath_div(ts,sum);
            t4 = iqmath_mul(k_svpwm,t4);
            t5 = iqmath_mul(k_svpwm,t5);
        }
        Q31_t t7 = (ts - t4 - t5) / 2;
        t_a = t4 + t5 + t7;
        t_b = t7;
        t_c = t5 + t7;
    } else {
        t_a = Q31_HALF;
        t_b = Q31_HALF;
        t_c = Q31_HALF;
    }
    *d_u = t_a;
    *d_v = t_b;
    *d_w = t_c;
}

void clarke(Q31_t *i_alpha,Q31_t *i_beta,Q31_t i_a,Q31_t i_b) {
  *i_alpha = i_a;
  *i_beta = iqmath_mul(iqmath_add(i_a,2*i_b), Value_INV_SQRT3_q31);
}

void park(Q31_t *i_d,Q31_t *i_q,Q31_t i_alpha,Q31_t i_beta,Q31_t sine,Q31_t cosine) {
  *i_d = iqmath_mul(i_alpha,cosine) + iqmath_mul(i_beta,sine);
  *i_q = iqmath_mul(i_beta,cosine) - iqmath_mul(i_alpha,sine);
}

void ipark(Q31_t *u_alpha,Q31_t *u_beta,Q31_t u_d,Q31_t u_q,Q31_t sine,Q31_t cosine) {
  *u_alpha = iqmath_mul(u_d,cosine) - iqmath_mul(u_q,sine);
  *u_beta = iqmath_mul(u_q,cosine) + iqmath_mul(u_d,sine);
}


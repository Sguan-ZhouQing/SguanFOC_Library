/*
 * @Author: 星必尘Sguan
 * @GitHub: https://github.com/Sguan-ZhouQing
 * @Date: 2026-01-26 22:28:00
 * @LastEditors: 星必尘Sguan|3464647102@qq.com
 * @LastEditTime: 2026-02-05 01:09:15
 * @FilePath: \demo_SguanFOCCode\SguanFOC库\Sguan_math.c
 * @Description: SguanFOC库的“数据处理库”实现
 * 1.快速正余弦  2.电机基本变换及SVPWM生成  3.参数限幅设置
 * 
 * Copyright (c) 2026 by $星必尘Sguan, All Rights Reserved. 
 */
#include "Sguan_math.h"

/* 外部C标准库文件声明 */
#include <stdbool.h>

// 内部宏定义声明
#define Value_rad60 1.047197551196598f
#define Value_INV_SQRT3 0.5773502691896257f
#define Value_SQRT3 1.73205080756887729353f

// 重写fmodf函数
static float Sguan_fmodf(float x, float y) {
  if (y == 0.0f) return 0.0f;
  int quotient = (int)(x / y); // 1次除法运算
  return x - quotient * y;     // 1次乘1次减
}

// 重写fabsf函数
float Sguan_fabsf(float x) {
  union {
      float f;
      uint32_t i;
  } u;
  u.f = x;
  u.i &= 0x7FFFFFFF; // 1次位与操作
  return u.f;
}

// 数值限幅
float Value_Limit(float val, float max, float min) {
    if (val > max) return max;
    if (val < min) return min;
    return val;
}

// 参数限定
float normalize_angle(float angle) {
  float normalized = Sguan_fmodf(angle, Value_PI*2);
  // 如果结果为负，加上2π使其在[0, 2π)范围内
  if (normalized < 0) {
      normalized += Value_PI*2;
  }
  return normalized;
}

// 快速sine和cosine求解的局部函数
static float f1(float x) {
  float u = 1.3528548e-10f;
  u = u * x + -2.4703144e-08f;
  u = u * x + 2.7532926e-06f;
  u = u * x + -0.00019840381f;
  u = u * x + 0.0083333179f;
  return u * x + -0.16666666f;
}

// 快速sine和cosine求解的局部函数
static float f2(float x) {
  float u = 1.7290616e-09f;
  u = u * x + -2.7093486e-07f;
  u = u * x + 2.4771643e-05f;
  u = u * x + -0.0013887906f;
  u = u * x + 0.041666519f;
  return u * x + -0.49999991f;
}

// 快速求解sine
float fast_sin(float x) {
  int si = (int)(x * 0.31830988f);
  x = x - (float)si * Value_PI;
  if (si & 1) {
    x = x > 0.0f ? x - Value_PI : x + Value_PI;
  }
  return x + x * x * x * f1(x * x);
}

// 快速求解cosine
float fast_cos(float x) {
  int si = (int)(x * 0.31830988f);
  x = x - (float)si * Value_PI;
  if (si & 1) {
    x = x > 0.0f ? x - Value_PI : x + Value_PI;
  }
  return 1.0f + x * x * f2(x * x);
}

// 快速求解sine和cosine
void fast_sin_cos(float x, float *sin_x, float *cos_x) {
  int si = (int)(x * 0.31830988f);
  x = x - (float)si * Value_PI;
  if (si & 1) {
    x = x > 0.0f ? x - Value_PI : x + Value_PI;
  }
  *sin_x = x + x * x * x * f1(x * x);
  *cos_x = 1.0f + x * x * f2(x * x);
}

// 电机SVPWM空间矢量调制函数
void SVPWM(float phi, float d, float q, float *d_u, float *d_v, float *d_w) {
  d = Value_Limit(d,1,-1);   // 限幅函数
  q = Value_Limit(q,1,-1);
  const int v[6][3] = {{1, 0, 0}, {1, 1, 0}, {0, 1, 0}, {0, 1, 1}, {0, 0, 1}, {1, 0, 1}};
  const int K_to_sector[] = {4, 6, 5, 5, 3, 1, 2, 2};
  float sin_phi,cos_phi;
  fast_sin_cos(phi,&sin_phi,&cos_phi);
  float alpha,beta;
  ipark(&alpha, &beta, d, q, sin_phi, cos_phi);

  bool A = beta > 0;
  bool B = Sguan_fabsf(beta) > Value_SQRT3 * Sguan_fabsf(alpha);
  bool C = alpha > 0;
  int K = 4 * A + 2 * B + C;
  int sector = K_to_sector[K];

  float angle_data0 = sector * Value_rad60;
  float angle_data1 = angle_data0 - Value_rad60;
  float sin_m,cos_m,sin_n,cos_n;
  fast_sin_cos(angle_data0,&sin_m,&cos_m);
  fast_sin_cos(angle_data1,&sin_n,&cos_n);

  float t_m = sin_m * alpha - cos_m * beta;
  float t_n = beta * cos_n - alpha * sin_n;
  float t_0 = 1 - t_m - t_n;
  *d_u = t_m * v[sector - 1][0] + t_n * v[sector % 6][0] + t_0 / 2;
  *d_v = t_m * v[sector - 1][1] + t_n * v[sector % 6][1] + t_0 / 2;
  *d_w = t_m * v[sector - 1][2] + t_n * v[sector % 6][2] + t_0 / 2;
}

// 克拉克变换
void clarke(float *i_alpha,float *i_beta,float i_a,float i_b) {
  *i_alpha = i_a;
  *i_beta = (i_a + 2 * i_b) * Value_INV_SQRT3;
}

// 帕克变换
void park(float *i_d,float *i_q,float i_alpha,float i_beta,float sine,float cosine) {
  *i_d = i_alpha * cosine + i_beta * sine;
  *i_q = i_beta * cosine - i_alpha * sine;
}

// 帕克逆变换
void ipark(float *u_alpha,float *u_beta,float u_d,float u_q,float sine,float cosine) {
  *u_alpha = u_d * cosine - u_q * sine;
  *u_beta = u_q * cosine + u_d * sine;
}


/*
 * @Author: 星必尘Sguan
 * @GitHub: https://github.com/Sguan-ZhouQing
 * @Date: 2026-02-06 03:54:11
 * @LastEditors: 星必尘Sguan|3464647102@qq.com
 * @LastEditTime: 2026-03-05 23:53:07
 * @FilePath: \stm_SguanFOCtest\SguanFOC\Sguan_math.c
 * @Description: SguanFOC库的“数学运算函数”实现
 * 
 * Copyright (c) 2026 by $星必尘Sguan, All Rights Reserved. 
 */
#include "Sguan_math.h"

// 内部宏定义声明
#define Value_rad60 1.047197551196598f
#define Value_INV_SQRT3 0.5773502691896257f
#define Value_SQRT3 1.73205080756887729353f

// 内部静态函数声明
static float Value_fmodf(float x, float y);
static float f1(float x);
static float f2(float x);
static void Overmod(float *d, float *q);

// 重写fmodf函数
static float Value_fmodf(float x, float y){
  if (y == 0.0f) return 0.0f;
  int quotient = (int)(x / y); // 1次除法运算
  return x - quotient * y;     // 1次乘1次减
}

// 重写fabsf函数
float Value_fabsf(float x){
  union {
      float f;
      uint32_t i;
  } u;
  u.f = x;
  u.i &= 0x7FFFFFFF; // 1次位与操作
  return u.f;
}

// 重写isinf函数
int Value_isinf(float x){
  union{
      float f;
      uint32_t i;
  } u = {x};
  
  // 单精度浮点数格式：
  // 符号位(1bit) | 指数位(8bits) | 尾数位(23bits)
  // 无穷大：指数位全1，尾数位全0
  // 去除符号位后检查
  uint32_t exp_mask = 0x7F800000;  // 指数位全1的掩码
  uint32_t mant_mask = 0x007FFFFF;  // 尾数位掩码
  
  // 检查指数位是否全1且尾数位全0
  if ((u.i & exp_mask) == exp_mask && (u.i & mant_mask) == 0) {
      return 1;  // 是无穷大
  }
  return 0;  // 不是无穷大
}

// 重写isnan函数
int Value_isnan(float x){
  union{
      float f;
      uint32_t i;
  } u = {x};
  
  uint32_t exp_mask = 0x7F800000;  // 指数位全1的掩码
  uint32_t mant_mask = 0x007FFFFF;  // 尾数位掩码
  if ((u.i & exp_mask) == exp_mask && (u.i & mant_mask) != 0) {
      return 1;  // 是NaN
  }
  return 0;  // 不是NaN
}

// 重写sqrtf函数
float Value_sqrtf(float x) {
  if (x <= 0.0f) {
      return 0.0f;
  }
  
  float guess = x;
  
  // 固定3次牛顿迭代 - 执行时间完全可预测
  guess = (guess + x / guess) * 0.5f;  // 第1次迭代
  guess = (guess + x / guess) * 0.5f;  // 第2次迭代
  guess = (guess + x / guess) * 0.5f;  // 第3次迭代
  
  return guess;
}
// 数值限幅
float Value_Limit(float val, float max, float min) {
  if (val > max) return max;
  if (val < min) return min;
  return val;
}

// 参数取模
float Value_normalize(float angle) {
  float normalized = Value_fmodf(angle, Value_PI*2);
  // 如果结果为负，加上2π使其在[0, 2π)范围内
  if (normalized < 0) {
      normalized += Value_PI*2;
  }
  return normalized;
}

// 快速sine和cosine求解的局部函数
static float f1(float x){
  float u = 1.3528548e-10f;
  u = u * x + -2.4703144e-08f;
  u = u * x + 2.7532926e-06f;
  u = u * x + -0.00019840381f;
  u = u * x + 0.0083333179f;
  return u * x + -0.16666666f;
}

// 快速sine和cosine求解的局部函数
static float f2(float x){
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

// 对DQ轴电压进行幅值限制，防止过调制
static void Overmod(float *d, float *q){
    // 计算合成“矢量幅值的平方”
    float Vref = (*d)*(*d) + (*q)*(*q);
    
    if (Vref > 1.0f) {
      float scale = 1.0f / Value_sqrtf(Vref);
      *d *= scale;
      *q *= scale;
      // 幅值限制处理,如果“幅值平方”超过 1,进行等比例缩放
    }
}

// 电机SVPWM空间矢量调制函数
void SVPWM(float d, float q, float sin_phi, float cos_phi, float *d_u, float *d_v, float *d_w){
  d = Value_Limit(d,1,-1);   // 限幅函数
  q = Value_Limit(q,1,-1);

  Overmod(&d, &q);    // 过调制处理

  const int v[6][3] = {{1, 0, 0}, {1, 1, 0}, {0, 1, 0}, {0, 1, 1}, {0, 0, 1}, {1, 0, 1}};
  const int K_to_sector[] = {4, 6, 5, 5, 3, 1, 2, 2};
  float alpha,beta;
  ipark(&alpha, &beta, d, q, sin_phi, cos_phi);

  int A = (beta > 0);
  int B = (Value_fabsf(beta) > Value_SQRT3 * Value_fabsf(alpha));
  int C = (alpha > 0);
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



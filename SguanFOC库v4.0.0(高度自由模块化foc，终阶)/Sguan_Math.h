#ifndef __SGUAN_MATH_H
#define __SGUAN_MATH_H

/* SguanFOC配置文件声明 */
#include "Sguan_Value.h"

CPLUSPLUS_C_BEGIN // C和C++配置文件声明_BEGIN
SguanQ math_from_float(SguanF f, SguanF base_value);
SguanF math_to_float(SguanQ q, SguanF base_value);
SguanQ math_add(SguanQ a, SguanQ b);
SguanQ math_sub(SguanQ a, SguanQ b);
SguanQ math_mul(SguanQ a, SguanQ b);
SguanQ math_div(SguanQ a, SguanQ b);
SguanQ math_convert_base(SguanQ q, SguanF old_base, SguanF new_base);
SguanQ math_sin(SguanQ x);
SguanQ math_cos(SguanQ x);
void math_sin_cos(SguanQ x, SguanQ *sin_x, SguanQ *cos_x);
SguanQ math_tan(SguanQ x, uint8_t base_out);
SguanQ math_atan(SguanQ x, SguanF base_in);

void *math_memset(void *ptr, int value, size_t num);
void *math_memcpy(void *dst, const void *src, size_t n);
uint8_t math_delay(uint32_t ms, uint32_t *count);
SguanQ math_normalize(SguanQ angle);
void math_rad_loop(SguanQ *out, SguanQ in, SguanQ time);
void math_correct(SguanQ *angle, SguanQ error);
SguanQ math_limit(SguanQ value, SguanQ max, SguanQ min);
SguanQ math_sign(SguanQ value, SguanF base_value);
SguanQ math_sqrt(SguanQ x);
SguanQ math_max(SguanQ a, SguanQ b);
SguanQ math_min(SguanQ a, SguanQ b);
SguanQ math_ads(SguanQ x);
SguanF math_pow(SguanF base, int n);
SguanF math_ln(SguanF x);
SguanF math_log2(SguanF x);
SguanF math_find_base(SguanF x, int *n);

void math_clarke(
    SguanQ *i_alpha, 
    SguanQ *i_beta, 
    SguanQ i_a, 
    SguanQ i_b);
void math_park(
    SguanQ *i_d, 
    SguanQ *i_q, 
    SguanQ i_alpha, 
    SguanQ i_beta, 
    SguanQ sine, 
    SguanQ cosine);
void math_ipark(
    SguanQ *u_alpha, 
    SguanQ *u_beta, 
    SguanQ u_d, 
    SguanQ u_q, 
    SguanQ sine, 
    SguanQ cosine);
void math_spwm(SguanQ u_alpha, SguanQ u_beta, 
    SguanQ *d_u, SguanQ *d_v, SguanQ *d_w);
uint8_t math_svpwm(SguanQ u_alpha, SguanQ u_beta, 
    SguanQ *d_u, SguanQ *d_v, SguanQ *d_w);
uint8_t math_swpwm(int8_t *sector, uint32_t *num_count, 
    SguanQ const_ubus, SguanQ const_we, uint32_t const_count, 
    SguanQ ubus, SguanQ we, 
    SguanQ ua, SguanQ ub, SguanQ uc, SguanQ umid, 
    uint8_t key[3], SguanQ *d_u, SguanQ *d_v, SguanQ *d_w);
void math_singlers_start(
    uint8_t sector_svpwm, 
    int16_t motor_ix[3], 
    uint16_t raw_ch0, 
    uint16_t raw_ch1, 
    uint16_t const_mid);
uint8_t math_singlers_end(
    SguanQ du, SguanQ dv, SguanQ dw,
    SguanQ const_duty_sw,
    SguanQ const_duty_adc,
    SguanQ adc_value[2],
    SguanQ duty_uvw[6]);
CPLUSPLUS_C_END // C和C++配置文件声明_END


#endif // SGUAN_MATH_H

#include "Sguan_IQmath.h"

#define IQmath_Define_Q31 0x01
#define IQmath_Define_Q15 0x02
// ===================== Q31 常量定义 =====================
static const int64_t Q31_MAX_64 = 2147483647LL;
static const int64_t Q31_MIN_64 = -2147483648LL;
// ===================== Q15 常量定义 =====================
static const int32_t Q15_MAX_32 = 32767;
static const int32_t Q15_MIN_32 = -32768;

// ===================== 辅助函数 =====================
static float Value_fabsf(float x) {
    return (x < 0) ? -x : x;
}

// ================== 局部静态函数(Q31版本)=====================
static Q31 IQmath_Q31_from_float(float f, float base_value){
    double scaled;
    if (base_value <= 0.0f || f != f){ 
        return 0;
    }
    else if(base_value != 1.0f){
        float normalized = f / base_value;
        if (normalized >= 1.0f){
            return Q31_MAX;
        } else if (normalized <= -1.0f){
            return Q31_MIN;
        }
        scaled = (double)normalized * 2147483648.0;
    }
    else{
        scaled = (double)f * 2147483648.0;
    }

    if (scaled >= 0.0){
        scaled += 0.5;
    } else{
        scaled -= 0.5;
    }

    int64_t result = (int64_t)scaled;
    if (result > Q31_MAX_64){
        return Q31_MAX;
    } else if (result < Q31_MIN_64){
        return Q31_MIN;
    }
    return (Q31)result;
}

static float IQmath_Q31_to_float(Q31 q, float base_value){
    if (base_value <= 0.0f){
        return 0.0f;
    }

    double normalized = (double)q * (1.0/2147483648.0);
    return (float)(normalized * base_value);
}

static Q31 IQmath_Q31_add(Q31 a, Q31 b){
    int32_t sum = a + b;
    if ((a > 0) && (b > 0) && (sum <= 0)){
        return Q31_MAX;
    }
    if ((a < 0) && (b < 0) && (sum >= 0)){
        return Q31_MIN;
    }
    return sum;
}

static Q31 IQmath_Q31_sub(Q31 a, Q31 b){
    int32_t diff = a - b;
    if ((a > 0) && (b < 0) && (diff <= 0)){
        return Q31_MAX;
    }
    if ((a < 0) && (b > 0) && (diff >= 0)){
        return Q31_MIN;
    }
    return diff;
}

static Q31 IQmath_Q31_mul(Q31 a, Q31 b){
    int64_t result = (int64_t)a * (int64_t)b;
    result += 1LL << 30;
    result >>= 31;

    if (result > Q31_MAX_64){
        return Q31_MAX;
    }
    if (result < Q31_MIN_64){
        return Q31_MIN;
    }
    return (Q31)result;
}

static Q31 IQmath_Q31_div(Q31 a, Q31 b){
    if (b == 0){
        return (a >= 0) ? Q31_MAX : Q31_MIN;
    }

    int64_t numerator = (int64_t)a << 31;
    int64_t result = numerator / b;
    int64_t remainder = numerator % b;
    if (remainder != 0){
        int64_t rem_abs = (remainder < 0) ? -remainder : remainder;
        int64_t b_abs = (b < 0) ? -(int64_t)b : (int64_t)b;
        if (rem_abs * 2 >= b_abs){
            result += (numerator >= 0) ? 1 : -1;
        }
    }

    if (result > Q31_MAX_64){
        return Q31_MAX;
    }
    if (result < Q31_MIN_64){
        return Q31_MIN;
    }
    return (Q31)result;
}

static Q31 IQmath_Q31_convert_base(Q31 q, float old_base, float new_base){
    if (old_base <= 0.0f || new_base <= 0.0f){
        return 0;
    }
    
    if (Value_fabsf(old_base - new_base) < 1e-6f){
        return q;
    }
    
    double ratio = (float)old_base / (float)new_base;
    double result = (double)q * ratio;
    if (result > 2147483647.0){
        return Q31_MAX;
    }
    if (result < -2147483648.0){
        return Q31_MIN;
    }
    
    if (result >= 0){
        result += 0.5;
    } else{
        result -= 0.5;
    }
    
    return (Q31)result;
}

// ================== 局部静态函数(Q15版本)=====================
static Q15 IQmath_Q15_from_float(float f, float base_value){
    double scaled;
    if (base_value <= 0.0f || f != f){ 
        return 0;
    }
    else if(base_value != 1.0f){
        float normalized = f / base_value;
        if (normalized >= 1.0f){
            return Q15_MAX;
        } else if (normalized <= -1.0f){
            return Q15_MIN;
        }
        scaled = (double)normalized * 32768.0;
    }
    else{
        scaled = (double)f * 32768.0;
    }

    if (scaled >= 0.0){
        scaled += 0.5;
    } else{
        scaled -= 0.5;
    }

    int32_t result = (int32_t)scaled;
    if (result > Q15_MAX_32){
        return Q15_MAX;
    } else if (result < Q15_MIN_32){
        return Q15_MIN;
    }
    return (Q15)result;
}

static float IQmath_Q15_to_float(Q15 q, float base_value){
    if (base_value <= 0.0f){
        return 0.0f;
    }

    double normalized = (double)q * (1.0/32768.0);
    return (float)(normalized * base_value);
}

static Q15 IQmath_Q15_add(Q15 a, Q15 b){
    int16_t sum = a + b;
    if ((a > 0) && (b > 0) && (sum <= 0)){
        return Q15_MAX;
    }
    if ((a < 0) && (b < 0) && (sum >= 0)){
        return Q15_MIN;
    }
    return sum;
}

static Q15 IQmath_Q15_sub(Q15 a, Q15 b){
    int16_t diff = a - b;
    if ((a > 0) && (b < 0) && (diff <= 0)){
        return Q15_MAX;
    }
    if ((a < 0) && (b > 0) && (diff >= 0)){
        return Q15_MIN;
    }
    return diff;
}

static Q15 IQmath_Q15_mul(Q15 a, Q15 b){
    int32_t result = (int32_t)a * (int32_t)b;
    result += 1 << 14;
    result >>= 15;

    if (result > Q15_MAX_32){
        return Q15_MAX;
    }
    if (result < Q15_MIN_32){
        return Q15_MIN;
    }
    return (Q15)result;
}

static Q15 IQmath_Q15_div(Q15 a, Q15 b){
    if (b == 0){
        return (a >= 0) ? Q15_MAX : Q15_MIN;
    }

    int32_t numerator = (int32_t)a << 15;
    int32_t result = numerator / b;
    int32_t remainder = numerator % b;
    if (remainder != 0){
        int32_t rem_abs = (remainder < 0) ? -remainder : remainder;
        int32_t b_abs = (b < 0) ? -(int32_t)b : (int32_t)b;
        if (rem_abs * 2 >= b_abs){
            result += (numerator >= 0) ? 1 : -1;
        }
    }

    if (result > Q15_MAX_32){
        return Q15_MAX;
    }
    if (result < Q15_MIN_32){
        return Q15_MIN;
    }
    return (Q15)result;
}

static Q15 IQmath_Q15_convert_base(Q15 q, float old_base, float new_base){
    if (old_base <= 0.0f || new_base <= 0.0f){
        return 0;
    }
    
    if (Value_fabsf(old_base - new_base) < 1e-6f){
        return q;
    }
    
    double ratio = (double)old_base / (double)new_base;
    double result = (double)q * ratio;
    if (result > 32767.0){
        return Q15_MAX;
    }
    if (result < -32768.0){
        return Q15_MIN;
    }
    
    if (result >= 0){
        result += 0.5;
    } else{
        result -= 0.5;
    }
    
    return (Q15)result;
}

// ================== 全局函数(同时兼容Q15和Q31)=====================
SguanQ IQmath_Q_from_float(float f, float base_value){
    #if CONFIG_IQMATH==IQmath_Define_Q15
    (void)IQmath_Q31_from_float;
    return IQmath_Q15_from_float(f, base_value);
    #else // CONFIG_IQMATH
    (void)IQmath_Q15_from_float;
    return IQmath_Q31_from_float(f, base_value);
    #endif // CONFIG_IQMATH
}

float IQmath_Q_to_float(SguanQ q, float base_value){
    #if CONFIG_IQMATH==IQmath_Define_Q15
    (void)IQmath_Q31_to_float;
    return IQmath_Q15_to_float(q, base_value);
    #else // CONFIG_IQMATH
    (void)IQmath_Q15_to_float;
    return IQmath_Q31_to_float(q, base_value);
    #endif // CONFIG_IQMATH
}

SguanQ IQmath_Q_add(SguanQ a, SguanQ b){
    #if CONFIG_IQMATH==IQmath_Define_Q15
    (void)IQmath_Q31_add;
    return IQmath_Q15_add(a, b);
    #else // CONFIG_IQMATH
    (void)IQmath_Q15_add;
    return IQmath_Q31_add(a, b);
    #endif // CONFIG_IQMATH
}

SguanQ IQmath_Q_sub(SguanQ a, SguanQ b){
    #if CONFIG_IQMATH==IQmath_Define_Q15
    (void)IQmath_Q31_sub;
    return IQmath_Q15_sub(a, b);
    #else // CONFIG_IQMATH
    (void)IQmath_Q15_sub;
    return IQmath_Q31_sub(a, b);
    #endif // CONFIG_IQMATH
}

SguanQ IQmath_Q_mul(SguanQ a, SguanQ b){
    #if CONFIG_IQMATH==IQmath_Define_Q15
    (void)IQmath_Q31_mul;
    return IQmath_Q15_mul(a, b);
    #else // CONFIG_IQMATH
    (void)IQmath_Q15_mul;
    return IQmath_Q31_mul(a, b);
    #endif // CONFIG_IQMATH
}

SguanQ IQmath_Q_div(SguanQ a, SguanQ b){
    #if CONFIG_IQMATH==IQmath_Define_Q15
    (void)IQmath_Q31_div;
    return IQmath_Q15_div(a, b);
    #else // CONFIG_IQMATH
    (void)IQmath_Q15_div;
    return IQmath_Q31_div(a, b);
    #endif // CONFIG_IQMATH
}

SguanQ IQmath_Q_convert_base(SguanQ q, float old_base, float new_base){
    #if CONFIG_IQMATH==IQmath_Define_Q15
    (void)IQmath_Q31_convert_base;
    return IQmath_Q15_convert_base(q, old_base, new_base);
    #else // CONFIG_IQMATH
    (void)IQmath_Q15_convert_base;
    return IQmath_Q31_convert_base(q, old_base, new_base);
    #endif // CONFIG_IQMATH
}


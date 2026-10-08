#include "Sguan_IQmath.h"

// ===================== Q31 常量定义 =====================
static const int64_t Q31_MAX_64 = 2147483647LL;
static const int64_t Q31_MIN_64 = -2147483648LL;
// ===================== Q15 常量定义 =====================
static const int32_t Q15_MAX_32 = 32767;
static const int32_t Q15_MIN_32 = -32768;

// ===================== 辅助函数 =====================
static SguanF value_fabsf(SguanF x) {
    return (x < 0) ? -x : x;
}

// ================== 局部静态函数(Q31版本)=====================
static Q31 iqmath_q31_from_float(SguanF f, SguanF base_value){
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

static SguanF iqmath_q31_to_float(Q31 q, SguanF base_value){
    if (base_value <= 0.0f){
        return 0.0f;
    }

    double normalized = (double)q * (1.0/2147483648.0);
    return (SguanF)(normalized * base_value);
}

static Q31 iqmath_q31_add(Q31 a, Q31 b){
    int32_t sum = a + b;
    if ((a > 0) && (b > 0) && (sum <= 0)){
        return Q31_MAX;
    }
    if ((a < 0) && (b < 0) && (sum >= 0)){
        return Q31_MIN;
    }
    return sum;
}

static Q31 iqmath_q31_sub(Q31 a, Q31 b){
    int32_t diff = a - b;
    if ((a > 0) && (b < 0) && (diff <= 0)){
        return Q31_MAX;
    }
    if ((a < 0) && (b > 0) && (diff >= 0)){
        return Q31_MIN;
    }
    return diff;
}

static Q31 iqmath_q31_mul(Q31 a, Q31 b){
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

static Q31 iqmath_q31_div(Q31 a, Q31 b){
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

static Q31 iqmath_q31_convert_base(Q31 q, SguanF old_base, SguanF new_base){
    if (old_base <= 0.0f || new_base <= 0.0f){
        return 0;
    }
    
    if (value_fabsf(old_base - new_base) < 1e-6f){
        return q;
    }
    
    double ratio = (double)old_base / (double)new_base;
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
static Q15 iqmath_q15_from_float(SguanF f, SguanF base_value){
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

static SguanF iqmath_q15_to_float(Q15 q, SguanF base_value){
    if (base_value <= 0.0f){
        return 0.0f;
    }

    double normalized = (double)q * (1.0/32768.0);
    return (SguanF)(normalized * base_value);
}

static Q15 iqmath_q15_add(Q15 a, Q15 b){
    int16_t sum = a + b;
    if ((a > 0) && (b > 0) && (sum <= 0)){
        return Q15_MAX;
    }
    if ((a < 0) && (b < 0) && (sum >= 0)){
        return Q15_MIN;
    }
    return sum;
}

static Q15 iqmath_q15_sub(Q15 a, Q15 b){
    int16_t diff = a - b;
    if ((a > 0) && (b < 0) && (diff <= 0)){
        return Q15_MAX;
    }
    if ((a < 0) && (b > 0) && (diff >= 0)){
        return Q15_MIN;
    }
    return diff;
}

static Q15 iqmath_q15_mul(Q15 a, Q15 b){
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

static Q15 iqmath_q15_div(Q15 a, Q15 b){
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

static Q15 iqmath_q15_convert_base(Q15 q, SguanF old_base, SguanF new_base){
    if (old_base <= 0.0f || new_base <= 0.0f){
        return 0;
    }
    
    if (value_fabsf(old_base - new_base) < 1e-6f){
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
SguanQ iqmath_from_float(SguanF f, SguanF base_value){
    #if VALUE_MATH==VALUE_Q31
    (void)iqmath_q15_from_float;
    return iqmath_q31_from_float(f, base_value);
    #elif VALUE_MATH==VALUE_Q15
    (void)iqmath_q31_from_float;
    return iqmath_q15_from_float(f, base_value);
    #else // VALUE_MATH
    (void)iqmath_q31_from_float;
    (void)iqmath_q15_from_float;
    return (f/base_value);
    #endif // VALUE_MATH
}

float iqmath_to_float(SguanQ q, SguanF base_value){
    #if VALUE_MATH==VALUE_Q31
    (void)iqmath_q15_to_float;
    return iqmath_q31_to_float(q, base_value);
    #elif VALUE_MATH==VALUE_Q15
    (void)iqmath_q31_to_float;
    return iqmath_q15_to_float(q, base_value);
    #else // VALUE_MATH
    (void)iqmath_q31_to_float;
    (void)iqmath_q15_to_float;
    return (q*base_value);
    #endif // VALUE_MATH
}

SguanQ iqmath_add(SguanQ a, SguanQ b){
    #if VALUE_MATH==VALUE_Q31
    (void)iqmath_q15_add;
    return iqmath_q31_add(a, b);
    #elif VALUE_MATH==VALUE_Q15
    (void)iqmath_q31_add;
    return iqmath_q15_add(a, b);
    #else // VALUE_MATH
    (void)iqmath_q31_add;
    (void)iqmath_q15_add;
    return (a + b);
    #endif // VALUE_MATH
}

SguanQ iqmath_sub(SguanQ a, SguanQ b){
    #if VALUE_MATH==VALUE_Q31
    (void)iqmath_q15_sub;
    return iqmath_q31_sub(a, b);
    #elif VALUE_MATH==VALUE_Q15
    (void)iqmath_q31_sub;
    return iqmath_q15_sub(a, b);
    #else // VALUE_MATH
    (void)iqmath_q31_sub;
    (void)iqmath_q15_sub;
    return (a - b);
    #endif // VALUE_MATH
}

SguanQ iqmath_mul(SguanQ a, SguanQ b){
    #if VALUE_MATH==VALUE_Q31
    (void)iqmath_q15_mul;
    return iqmath_q31_mul(a, b);
    #elif VALUE_MATH==VALUE_Q15
    (void)iqmath_q31_mul;
    return iqmath_q15_mul(a, b);
    #else // VALUE_MATH
    (void)iqmath_q31_mul;
    (void)iqmath_q15_mul;
    return (a * b);
    #endif // VALUE_MATH
}

SguanQ iqmath_div(SguanQ a, SguanQ b){
    #if VALUE_MATH==VALUE_Q31
    (void)iqmath_q15_div;
    return iqmath_q31_div(a, b);
    #elif VALUE_MATH==VALUE_Q15
    (void)iqmath_q31_div;
    return iqmath_q15_div(a, b);
    #else // VALUE_MATH
    (void)iqmath_q31_div;
    (void)iqmath_q15_div;
    return (a / b);
    #endif // VALUE_MATH
}

SguanQ iqmath_convert_base(SguanQ q, SguanF old_base, SguanF new_base){
    #if VALUE_MATH==VALUE_Q31
    (void)iqmath_q15_convert_base;
    return iqmath_q31_convert_base(q, old_base, new_base);
    #elif VALUE_MATH==VALUE_Q15
    (void)iqmath_q31_convert_base;
    return iqmath_q15_convert_base(q, old_base, new_base);
    #else // VALUE_MATH
    (void)iqmath_q31_convert_base;
    (void)iqmath_q15_convert_base;
    return (q * (new_base/old_base));
    #endif // VALUE_MATH
}

// ========================================================================
SguanQ iqmath_abs(SguanQ x){
    return (x < 0) ? -x : x;
}

SguanQ iqmath_zero(void){
    #if VALUE_MATH==VALUE_Q31
    return 0;
    #elif VALUE_MATH==VALUE_Q15
    return 0;
    #else // VALUE_MATH
    return 0.0f;
    #endif // VALUE_MATH
}

// =========================================================================
// ========================================================================
// 查表法求得Q31的角度正余弦值
static const Q31 sin_tab_q31[512] = { 
    0,26405458,52804476,79197048,105578888,131943544,158286720,184608432,210900080,237168096,263388864,
    289566688,315723040,341814976,367842464,393827040,419768640,445624320,471415616,497142464,522804896,
    548359936,573850560,599255296,624574144,649785600,674889664,699907840,724818688,749622144,774318208,
    798885376,823323776,847654720,871856896,895908672,919853120,943625792,967291072,990784512,1014127680,
    1037341952,1060384448,1083255168,1105975552,1128524160,1150900864,1173105920,1195139072,1216978944,
    1238647040,1260121984,1281403520,1302491776,1323386752,1344088576,1364575488,1384869248,1404948224,
    1424812416,1444461952,1463875200,1483095168,1502078976,1520826496,1539359232,1557655808,1575716096,
    1593540224,1611106688,1628458368,1645530880,1662388608,1678967168,1695309440,1711394176,1727199616,
    1742768896,1758059008,1773091328,1787865984,1802340096,1816577920,1830515072,1844173056,1857573376,
    1870673024,1883515008,1896056320,1908296960,1920258432,1931940736,1943322368,1954424832,1965205248,
    1975706368,1985906944,1995806848,2005406080,2014683264,2023681152,2032356992,2040732288,2048806784,
    2056559232,2064011008,2071140608,2077969664,2084476416,2090661248,2096545280,2102107264,2107347200,
    2112264960,2116860544,2121155456,2125106816,2128757632,2132064768,2135071232,2137734016,2140096256,
    2142114944,2143811456,2145207296,2146259584,2146989696,2147397760,2147483647,2147225984,2146667648,
    2145765632,2144541568,2143016832,2141148544,2138958080,2136445568,2133610880,2130454144,2126975232,
    2123174144,2119051008,2114605696,2109838208,2104748672,2099358592,2093646208,2087611776,2081255296,
    2074598016,2067618688,2060317312,2052715136,2044812416,2036587648,2028062080,2019235968,2010087680,
    2000638720,1990889088,1980838912,1970488064,1959858048,1948905856,1937674496,1926142464,1914309888,
    1902219520,1889807104,1877136896,1864166144,1850916096,1837387008,1823578752,1809491200,1795145984,
    1780500224,1765618048,1750456832,1735016448,1719318400,1703384064,1687170560,1670699392,1653991936,
    1637026816,1619803904,1602344960,1584649600,1566718208,1548529024,1530125056,1511484928,1492608512,
    1473517440,1454190080,1434647936,1414891136,1394919424,1374754560,1354353536,1333759104,1312971520,
    1291969152,1270794880,1249405952,1227845248,1206091264,1184144000,1162024832,1139734016,1117271296,
    1094636800,1071830592,1048873984,1025745536,1002488320,979059264,955479872,931750208,907891648,
    883904256,859766528,835499968,811126080,786601792,761991616,737231168,712384768,687430976,662348352,
    637179904,611925440,586563712,561116032,535582432,509984416,484300512,458530720,432696480,406797824,
    380856224,354828736,328758272,302644864,276488512,250289216,224025488,197757472,171450800,145118352,
    118762288,92389040,66002908,39606040,13202515,-13202515,-39606040,-66002908,-92389040,-118762288,
    -145118352,-171450800,-197757472,-224025488,-250289216,-276488512,-302644864,-328758272,-354828736,
    -380856224,-406797824,-432696480,-458530720,-484300512,-509984416,-535582432,-561116032,-586563712,
    -611925440,-637179904,-662348352,-687430976,-712384768,-737231168,-761991616,-786601792,-811126080,
    -835499968,-859766528,-883904256,-907891648,-931750208,-955479872,-979059264,-1002488320,-1025745536,
    -1048873984,-1071830592,-1094636800,-1117271296,-1139734016,-1162024832,-1184144000,-1206091264,
    -1227845248,-1249405952,-1270794880,-1291969152,-1312971520,-1333759104,-1354353536,-1374754560,
    -1394919424,-1414891136,-1434647936,-1454190080,-1473517440,-1492608512,-1511484928,-1530125056,
    -1548529024,-1566718208,-1584649600,-1602344960,-1619803904,-1637026816,-1653991936,-1670699392,
    -1687170560,-1703384064,-1719318400,-1735016448,-1750456832,-1765618048,-1780500224,-1795145984,
    -1809491200,-1823578752,-1837387008,-1850916096,-1864166144,-1877136896,-1889807104,-1902219520,
    -1914309888,-1926142464,-1937674496,-1948905856,-1959858048,-1970488064,-1980838912,-1990889088,
    -2000638720,-2010087680,-2019235968,-2028062080,-2036587648,-2044812416,-2052715136,-2060317312,
    -2067618688,-2074598016,-2081255296,-2087611776,-2093646208,-2099358592,-2104748672,-2109838208,
    -2114605696,-2119051008,-2123174144,-2126975232,-2130454144,-2133610880,-2136445568,-2138958080,
    -2141148544,-2143016832,-2144541568,-2145765632,-2146667648,-2147225984,-2147483648,-2147397760,
    -2146989696,-2146259584,-2145207296,-2143811456,-2142114944,-2140096256,-2137734016,-2135071232,
    -2132064768,-2128757632,-2125106816,-2121155456,-2116860544,-2112264960,-2107347200,-2102107264,
    -2096545280,-2090661248,-2084476416,-2077969664,-2071140608,-2064011008,-2056559232,-2048806784,
    -2040732288,-2032356992,-2023681152,-2014683264,-2005406080,-1995806848,-1985906944,-1975706368,
    -1965205248,-1954424832,-1943322368,-1931940736,-1920258432,-1908296960,-1896056320,-1883515008,
    -1870673024,-1857573376,-1844173056,-1830515072,-1816577920,-1802340096,-1787865984,-1773091328,
    -1758059008,-1742768896,-1727199616,-1711394176,-1695309440,-1678967168,-1662388608,-1645530880,
    -1628458368,-1611106688,-1593540224,-1575716096,-1557655808,-1539359232,-1520826496,-1502078976,
    -1483095168,-1463875200,-1444461952,-1424812416,-1404948224,-1384869248,-1364575488,-1344088576,
    -1323386752,-1302491776,-1281403520,-1260121984,-1238647040,-1216978944,-1195139072,-1173105920,
    -1150900864,-1128524160,-1105975552,-1083255168,-1060384448,-1037341952,-1014127680,-990784512,
    -967291072,-943625792,-919853120,-895908672,-871856896,-847654720,-823323776,-798885376,-774318208,
    -749622144,-724818688,-699907840,-674889664,-649785600,-624574144,-599255296,-573850560,-548359936,
    -522804896,-497142464,-471415616,-445624320,-419768640,-393827040,-367842464,-341814976,-315723040,
    -289566688,-263388864,-237168096,-210900080,-184608432,-158286720,-131943544,-105578888,-79197048,
    -52804476,-26405458,0
};

// 快速求解sine
#define SIN_K_q31 618
Q31 fast_sin(Q31 x){
    int32_t idx;
    if (x < 0){
        x = 0;
    }

    idx = (int32_t)(((int64_t)x * SIN_K_q31) >> 31);
    if (idx >= 512){
        idx = 511;
    }
    if (idx < 0){
        idx = 0;
    }

    return sin_tab_q31[idx];
}

// 快速求解cosine
#define Value_PI_2_q31 0
#define Value_2PI_q31 0
Q31 fast_cos(Q31 x){
    Q31 x_shift = x + Value_PI_2_q31;

    if (x_shift >= Value_2PI_q31) {
        x_shift -= Value_2PI_q31;
    }

    return fast_sin(x_shift);
}

// 快速求解sine和cosine
void fast_sin_cos(Q31 x, Q31 *sin_x, Q31 *cos_x){
  *sin_x = fast_sin(x);
  *cos_x = fast_cos(x);
}

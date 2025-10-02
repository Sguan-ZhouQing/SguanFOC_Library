/*
 * @Author: 星必尘Sguan
 * @Date: 2025-10-02 14:40:13
 * @LastEditors: 星必尘Sguan|3464647102@qq.com
 * @LastEditTime: 2025-10-02 16:19:41
 * @FilePath: \SguanFOC库\SguanFOC.c
 * @Description: 
 * 
 * Copyright (c) 2025 by $JUST, All Rights Reserved. 
 */
#include "SguanUser_data.h"
#include "SguanFOC.h"


#define PI 3.14159265358979323846f
// 磁定向控制的结构体变量
SVPWM_HandleTypeDef SguanSVPWM;
FOC_HandleTypeDef SguanFOC;
// 默认模式与目标量
FOC_Mode_t FOC_Mode = FOC_MODE_NONE;
float FOC_Target_Position = 0.0f;
float FOC_Target_Speed = 0.0f;
float FOC_Target_Current = 0.0f;
float FOC_Target_Voltage = 0.2f;
// 开环速度控制相关变量
static float electrical_angle = 0.0f;   // 电角度（弧度）
static uint32_t last_time = 0;          // 上次更新时间
// 闭环电流计算需要用到的参数
#define Sqrt3 1.732050807568877f        // 根号3的浮点值
static float current_Iq = 0.0f;         // Iq电流滤波后的数据
// 初始"电角度"和"机械角度"对齐变量
static float alignment_angle_offset = 0.0f;



/**
 * @description: f1 和 f2（多项式逼近）
 * // lolremez --float --degree 5 --range "1e-50:pi*pi"
 * // "(sin(sqrt(x))-sqrt(x))/(x*sqrt(x))" "1/(x*sqrt(x))"
 * // Estimated max error: 1.455468e-9
 * @param {float} x
 * @return {*}
 */
static float f1(float x) {
  float u = 1.3528548e-10f;
  u = u * x + -2.4703144e-08f;
  u = u * x + 2.7532926e-06f;
  u = u * x + -0.00019840381f;
  u = u * x + 0.0083333179f;
  return u * x + -0.16666666f;
}
/**
 * @description: f1 和 f2（多项式逼近）
 * // lolremez --float --degree 5 --range "1e-50:pi*pi" "(cos(sqrt(x))-1)/x" "1/x"
 * // Estimated max error: 1.1846383e-8
 * @param {float} x
 * @return {*}
 */
static float f2(float x) {
  float u = 1.7290616e-09f;
  u = u * x + -2.7093486e-07f;
  u = u * x + 2.4771643e-05f;
  u = u * x + -0.0013887906f;
  u = u * x + 0.041666519f;
  return u * x + -0.49999991f;
}
/**
 * @description: [核心函数]生成sin;
 * @param {float} x
 * @return {*}
 */
static float fast_sin(float x) {
  // si = (int)(x / pi)
  int si = (int)(x * 0.31830988f);
  x = x - (float)si * PI;
  if (si & 1) {
    x = x > 0.0f ? x - PI : x + PI;
  }
  return x + x * x * x * f1(x * x);
}
/**
 * @description: [核心函数]生成cos;
 * @param {float} x
 * @return {*}
 */
static float fast_cos(float x) {
  // si = (int)(x / pi)
  int si = (int)(x * 0.31830988f);
  x = x - (float)si * PI;
  if (si & 1) {
    x = x > 0.0f ? x - PI : x + PI;
  }
  return 1.0f + x * x * f2(x * x);
}
/**
 * @description: [核心函数]联合求解sin和cos;
 * @param {float} x
 * @param {float} *sin_x
 * @param {float} *cos_x
 * @return {*}
 */
static void fast_sin_cos(float x, float *sin_x, float *cos_x) {
  // si = (int)(x / pi)
  int si = (int)(x * 0.31830988f);
  x = x - (float)si * PI;
  if (si & 1) {
    x = x > 0.0f ? x - PI : x + PI;
  }
  *sin_x = x + x * x * x * f1(x * x);
  *cos_x = 1.0f + x * x * f2(x * x);
}




/**
 * @description: 低通滤波
 * @param {float} input
 * @param {float} last_output
 * @param {float} alpha
 * @return {*}
 */
static float low_pass_filter(float input, float last_output, float alpha)
{
    return alpha * input + (1.0 - alpha) * last_output;
}
/**
 * @description: 一阶卡尔曼滤波（其一）
 * @param {float} input
 * @param {float} r
 * @param {float} q
 * @return {*}
 */
static float kalman_filter_std(float input, float r, float q)
{
    static float z;
    static float p = 1;
    float g = 0;
    p = p + q;
    g = p / (p + r);
    z = z + g * (input - z);
    p = (1 - g) * p;
    return z;
}
/**
 * @description: 一阶卡尔曼滤波（其二）
 * @param {float} input
 * @param {float} r
 * @param {float} q
 * @return {*}
 */
static float kalman_filter_dir(float input, float r, float q)
{
    static float z_dir;
    static float pp = 1;
    float g = 0;
    pp = pp + q;
    g = pp / (pp + r);
    z_dir = z_dir + g * (input - z_dir);
    pp = (1 - g) * pp;
    return z_dir;
}




/**
 * @description: 【逆Park变换1】计算并更新u_alpha、u_beta，同时计算sinθ和cosθ
 * @param {FOC_HandleTypeDef*} foc: FOC控制结构体
 * 作用：将旋转坐标系(dq)的电压转换回静止坐标系(αβ)
 */
static void ipark(SVPWM_HandleTypeDef* foc) {
    // 计算当前角度的正弦和余弦值
    foc->sine = fast_sin(foc->theta);
    foc->cosine = fast_cos(foc->theta);
    
    // 逆Park变换公式：从dq坐标系转换到αβ坐标系
    foc->u_alpha = foc->u_d * foc->cosine - foc->u_q * foc->sine;
    foc->u_beta = foc->u_q * foc->cosine + foc->u_d * foc->sine;
}
/**
 * @description: 【逆Park变换2】仅计算u_alpha和u_beta，复用已有的sinθ和cosθ
 * @param {FOC_HandleTypeDef*} foc: FOC控制结构体
 * 作用：与ipark()相同，但效率更高（避免重复计算三角函数）
 */
static void ipark2(SVPWM_HandleTypeDef* foc) {
    // 直接使用预先计算好的三角函数值进行逆Park变换
    foc->u_alpha = foc->u_d * foc->cosine - foc->u_q * foc->sine;
    foc->u_beta = foc->u_q * foc->cosine + foc->u_d * foc->sine;
}
/**
 * @description: 【Clark变换】将三相电流(a-b-c)转换为两相静止坐标系(α-β)
 * @param {FOC_HandleTypeDef*} foc: FOC控制结构体
 * 作用：简化三相系统为两相系统，便于后续处理
 */
static void clarke(SVPWM_HandleTypeDef* foc) {
    // Clark变换公式（假设三相电流和为0：ia + ib + ic = 0）
    foc->i_alpha = foc->i_a;  // α轴电流等于A相电流
    foc->i_beta = (foc->i_a + 2 * foc->i_b) * 0.5773502691896257f;  // β轴电流 = (ia + 2*ib)/√3
}
/**
 * @description: 【Park变换】将静止坐标系(α-β)的电流转换到旋转坐标系(d-q)
 * @param {FOC_HandleTypeDef*} foc: FOC控制结构体
 * 作用：将电流转换到随转子旋转的坐标系，便于解耦控制
 */
static void park(SVPWM_HandleTypeDef* foc) {
    // 计算当前角度的正弦和余弦值
    foc->sine = fast_sin(foc->theta);
    foc->cosine = fast_cos(foc->theta);
    
    // Park变换公式：从αβ坐标系转换到dq坐标系
    foc->i_d = foc->i_alpha * foc->cosine + foc->i_beta * foc->sine;
    foc->i_q = foc->i_beta * foc->cosine - foc->i_alpha * foc->sine;
}
/**
 * @description: 【SVPWM核心算法】将α-β轴电压转换为三相PWM占空比
 * @param {FOC_HandleTypeDef*} foc: FOC控制结构体
 * 作用：生成驱动三相逆变器的PWM信号，实现精确的电压矢量控制
 */
static void svpwm(SVPWM_HandleTypeDef* foc) {
    const float ts = 1;  // PWM周期（归一化为1）
    // 步骤1：计算三个参考电压，用于扇区判断
    float u1 = foc->u_beta;  // Uβ
    float u2 = -0.8660254037844386f * foc->u_alpha - 0.5f * foc->u_beta;  // -√3/2 * Uα - 1/2 * Uβ
    float u3 = 0.8660254037844386f * foc->u_alpha - 0.5f * foc->u_beta;   // √3/2 * Uα - 1/2 * Uβ
    // 步骤2：通过符号判断法确定当前扇区（1-6）
    // 原理：根据u1、u2、u3的正负组合确定电压矢量所在的60°扇区
    uint8_t sector = (u1 > 0.0f) + ((u2 > 0.0f) << 1) + ((u3 > 0.0f) << 2);
    // 步骤3：根据不同扇区，计算基本矢量的作用时间
    // 每个扇区使用两个相邻的基本矢量合成目标电压矢量
    if (sector == 5) {
        // 扇区5：使用矢量U4(100)和U6(110)
        float t4 = u3;  // U4的作用时间
        float t6 = u1;  // U6的作用时间
        // 过调制处理：如果总时间超过周期，等比例缩小
        float sum = t4 + t6;
        if (sum > ts) {
            foc->k_svpwm = ts / sum;
            t4 = foc->k_svpwm * t4;
            t6 = foc->k_svpwm * t6;
        }
        // 计算零矢量时间（七段式对称分布）
        float t7 = (ts - t4 - t6) / 2;
        // 计算三相占空比（七段式序列：零-U4-U6-零-U6-U4-零）
        foc->t_a = t4 + t6 + t7;  // A相：U4 + U6 + 零矢量
        foc->t_b = t6 + t7;       // B相：U6 + 零矢量
        foc->t_c = t7;            // C相：零矢量
        
    } else if (sector == 1) {
        // 扇区1：使用矢量U2(010)和U6(110)
        float t2 = -u3;  // U2的作用时间
        float t6 = -u2;  // U6的作用时间
        
        float sum = t2 + t6;
        if (sum > ts) {
            foc->k_svpwm = ts / sum;
            t2 = foc->k_svpwm * t2;
            t6 = foc->k_svpwm * t6;
        }
        float t7 = (ts - t2 - t6) / 2;
        // 七段式序列：零-U6-U2-零-U2-U6-零
        foc->t_a = t6 + t7;          // A相：U6 + 零矢量
        foc->t_b = t2 + t6 + t7;     // B相：U2 + U6 + 零矢量
        foc->t_c = t7;               // C相：零矢量
        
    } 
    // ...（其他扇区的计算类似，只是使用的矢量和计算公式不同）
    else if (sector == 3) {
        // 扇区3：使用矢量U2(010)和U3(011)
        float t2 = u1;
        float t3 = u2;
        float sum = t2 + t3;
        if (sum > ts) {
            foc->k_svpwm = ts / sum;
            t2 = foc->k_svpwm * t2;
            t3 = foc->k_svpwm * t3;
        }
        float t7 = (ts - t2 - t3) / 2;
        foc->t_a = t7;
        foc->t_b = t2 + t3 + t7;
        foc->t_c = t3 + t7;
        
    } else if (sector == 2) {
        // 扇区2：使用矢量U1(001)和U3(011)
        float t1 = -u1;
        float t3 = -u3;
        float sum = t1 + t3;
        if (sum > ts) {
            foc->k_svpwm = ts / sum;
            t1 = foc->k_svpwm * t1;
            t3 = foc->k_svpwm * t3;
        }
        float t7 = (ts - t1 - t3) / 2;
        foc->t_a = t7;
        foc->t_b = t3 + t7;
        foc->t_c = t1 + t3 + t7;
        
    } else if (sector == 6) {
        // 扇区6：使用矢量U1(001)和U5(101)
        float t1 = u2;
        float t5 = u3;
        float sum = t1 + t5;
        if (sum > ts) {
            foc->k_svpwm = ts / sum;
            t1 = foc->k_svpwm * t1;
            t5 = foc->k_svpwm * t5;
        }
        float t7 = (ts - t1 - t5) / 2;
        foc->t_a = t5 + t7;
        foc->t_b = t7;
        foc->t_c = t1 + t5 + t7;
        
    } else if (sector == 4) {
        // 扇区4：使用矢量U4(100)和U5(101)
        float t4 = -u2;
        float t5 = -u1;
        float sum = t4 + t5;
        if (sum > ts) {
            foc->k_svpwm = ts / sum;
            t4 = foc->k_svpwm * t4;
            t5 = foc->k_svpwm * t5;
        }
        float t7 = (ts - t4 - t5) / 2;
        foc->t_a = t4 + t5 + t7;
        foc->t_b = t7;
        foc->t_c = t5 + t7;
    }
    // 至此，t_a, t_b, t_c 就是三相PWM的占空比，可以直接写入定时器寄存器
}








// 初始化FOC控制器的底层硬件
void FOC_Init(void) {
    // 初始化FOC结构体
    memset(&SguanSVPWM, 0, sizeof(SVPWM_HandleTypeDef));
    current_Iq = 0.0f;     // 初始化电流Iq值
    
    // 1.定时器初始化(包括PWM)
    Sguan_TimerInit();
    // 2.磁编码器初始化
    Sguan_PosSensorInit();
    // 3.ADC相电流采样初始化
    Sguan_CurSamplingInit();
    // 4.串口初始化
    Sguan_UartInit();

    // 电角度对齐(end)
    FOC_EncoderAlignment();
}
// 添加constrain函数实现
static float constrain(float value, float min_val, float max_val) {
    if (value < min_val) {
        return min_val;
    } else if (value > max_val) {
        return max_val;
    } else {
        return value;
    }
}
// 设置三相PWM占空比
static void set_pwm_duty_cycle(float duty_a, float duty_b, float duty_c) {
    // 限制占空比在安全范围内
    duty_a = constrain(duty_a, Dead_Time, (1.0f - Dead_Time));
    duty_b = constrain(duty_b, Dead_Time, (1.0f - Dead_Time));
    duty_c = constrain(duty_c, Dead_Time, (1.0f - Dead_Time));
    // 电机占空比计算（ABC三相电压）
    uint16_t FinalDuty_A,FinalDuty_B, ccr_a, ccr_b, ccr_c;
    ccr_a = (uint16_t)(duty_a * SguanFOC_ARR);
    ccr_b = (uint16_t)(duty_b * SguanFOC_ARR);
    ccr_c = (uint16_t)(duty_c * SguanFOC_ARR);
    // 根据电机方向调整相位
    if (Motor_Dir) {
        // 正转：正常输出
        FinalDuty_A = ccr_a;
        FinalDuty_B = ccr_b;
    } else {
        // 反转：交换A相和B相
        FinalDuty_A = ccr_b;
        FinalDuty_B = ccr_a;
    }
    Sguan_PwmSet(1,FinalDuty_A);
    Sguan_PwmSet(2,FinalDuty_B);
    Sguan_PwmSet(3,ccr_c);
}
// 生成SVPWM波
static void generate_svpwm_waveforms(void) {
    ipark(&SguanSVPWM);      // 逆Park变换
    svpwm(&SguanSVPWM);      // SVPWM计算
    set_pwm_duty_cycle(SguanSVPWM.t_a, SguanSVPWM.t_b, SguanSVPWM.t_c); // 设置PWM
}
// 角度归一化，将角度限制在0-2π范围内
static float normalize_angle(float angle) {
    // 一次性处理所有情况
    angle = fmodf(angle, 2.0f * PI);
    return angle < 0 ? angle + 2.0f * PI : angle;
}
/**
 * @brief 编码器零位自动校准
 * @description: 通过注入d轴电流将转子拉到d轴位置，记录此时的编码器位置作为偏移量
 */
static void FOC_EncoderAlignment(void) {
    // 1. 注入d轴电流将转子拉到d轴位置
    SguanSVPWM.u_d = 0.5f;  // 设置d轴电压
    SguanSVPWM.u_q = 0.0f;  // 注入q轴电压
    SguanSVPWM.theta = 0.0f; // 假设电角度为0时q轴对齐
    // 生成SVPWM，将转子拉到d轴位置
    generate_svpwm_waveforms();
    Sguan_UserDelay(1000);  // 等待800ms让转子稳定
    
    // 2. 读取此时的编码器机械角度
    float mechanical_angle_rad;
    if (MT6701_ReadAngle(&mechanical_angle_rad)) {
        // 计算偏移量：电角度 = (机械角度 + 偏移量) * 极对数
        alignment_angle_offset = -mechanical_angle_rad;
    }
    // 3. 停止注入电流
    SguanSVPWM.u_d = 0.0f;
    SguanSVPWM.u_q = 0.0f;
    generate_svpwm_waveforms();
}
/**
 * @description: 计算q轴电流Iq值
 * @note: 带一阶卡尔曼滤波（计算后输出平稳曲线）
 * @return {float} Iq电流值
 */
static float FOC_Calculate_Iq(void) {
    // 1. 采样三相电流
    int32_t Iu_Raw,Iv_Raw;
    Iu_Raw = Sguan_CurAcquisition(0) - Intermediate_Raw;  // U相电流（电压值）
    Iv_Raw = Sguan_CurAcquisition(1) - Intermediate_Raw;  // V相电流（电压值）
    // 将原始ADC值转换为实际电流值（单位：A）
    // 计算公式：电流(A) = (ADC原始值 * 3.3V / 4096) / (增益 * 采样电阻)
    // 简化后：电流(A) = ADC原始值 * (3.3 / (4096 * 50 * 0.02))
    // 计算系数：3.3 / (4096 * 50 * 0.02) = 3.3 / 4096 ≈ 0.00080566
    const float ADC_to_Current = 3.3f / 4096.0f / Operational_Num / (Shunt_Resistor / 1000.0f);
    float Iu = ADC_to_Current * (float)Iu_Raw;
    float Iv = ADC_to_Current * (float)Iv_Raw;
    // Iw = -Iu - Iv;                // W相电流（根据KCL定律）
    
    // 2. Clarke变换 (3相 → 2相)
    float I_alpha = Iu;
    float I_beta = (Iu + 2.0f * Iv) * (1.0f / Sqrt3);  // 1/√3
    // 3. Park变换 (静止 → 旋转)
    float sin_theta, cos_theta;
    fast_sin_cos(SguanSVPWM.theta,&sin_theta,&cos_theta);
    
    // Park变换公式: 
    // Id = I_alpha * cos(theta) + I_beta * sin(theta)
    // Iq = -I_alpha * sin(theta) + I_beta * cos(theta)
    float Raw_Iq = -I_alpha * sin_theta + I_beta * cos_theta;
    current_Iq = kalman_filter_std(Raw_Iq, M_NOISE, P_NOISE);
    return current_Iq;
}
/**
 * @description: [Mode1]FOC开环位置控制
 * @param {float} angle_deg 机械角度度数(0到359度)
 * @param {int} pole_pairs 电机极对数
 * @param {float} voltage 电压幅值(0-1)
 */
static void FOC_OpenPosition_Loop(float angle_deg, float voltage) {
    // 1. 机械角度转电角度
    float mechanical_angle = (angle_deg / 360.0f) * 2.0f * PI;
    SguanSVPWM.theta = normalize_angle(mechanical_angle * Pole_Pairs);
    // 2. 设置电压（控制转矩）
    SguanSVPWM.u_q = 0.0f;
    SguanSVPWM.u_d = voltage;
    // 3. 生成SVPWM波形
    generate_svpwm_waveforms();
}
/**
 * @description: [Mode2]FOC开环速度控制
 * @param {float} velocity_rad_s 机械角速度（弧度/秒）
 * @param {float} voltage 电压幅值(0-1)
 */
static void FOC_OpenVelocity_Loop(float velocity_rad_s, float voltage) {
    // 1. 计算时间差（毫秒）
    uint32_t current_time = Sguan_UserGetTick();
    float delta_time_ms = (float)(current_time - last_time);
    last_time = current_time;
    // 2. 将机械角速度转换为电角速度
    float electrical_velocity = velocity_rad_s * Pole_Pairs;  // 电角速度（弧度/秒）
    // 3. 计算角度增量（积分得到角度）
    // delta_time_ms转换为秒：/ 1000.0f
    float angle_increment = electrical_velocity * (delta_time_ms / 1000.0f);
    // 4. 更新电角度（保持角度在0-2π范围内）
    electrical_angle += angle_increment;
    electrical_angle = normalize_angle(electrical_angle);
    
    // 5. 设置电角度
    SguanSVPWM.theta = electrical_angle;
    // 6. 设置电压（控制转矩）
    SguanSVPWM.u_q = 0.0f;
    SguanSVPWM.u_d = voltage;
    // 7. 生成SVPWM波形
    generate_svpwm_waveforms();
}

/**
 * @description: [Mode3]FOC位置单环闭环控制
 * @param {float} target_angle_rad  目标多圈位置 (弧度)
 */
static void FOC_Position_SingleLoop(float target_angle_rad) {
    float actual_angle_rad;
    Sguan_ReadMultiTurnAngle(&actual_angle_rad);

    // --- 1. 误差计算 ---
    float error = target_angle_rad - actual_angle_rad;

    // --- 2. PID控制器 ---
    float *Kp = &SguanFOC.Position_PID.Kp;
    float *Ki = &SguanFOC.Position_PID.Ki;
    float *Kd = &SguanFOC.Position_PID.Kd;
    float *Integral = &SguanFOC.Position_PID.Integral;
    float *Prev_error = &SguanFOC.Position_PID.Prev_error;
    float limit = SguanFOC.Position_PID.Output_limit;

    *Integral += error;
    // 防止积分过大
    if (*Integral > limit) *Integral = limit;
    if (*Integral < -limit) *Integral = -limit;

    float derivative = error - *Prev_error;
    *Prev_error = error;

    float pid_output = (*Kp * error) + (*Ki * (*Integral)) + (*Kd * derivative);

    // --- 3. 限幅 ---
    pid_output = constrain(pid_output, -limit, limit);

    // --- 4. 设置FOC电压 ---
    SguanSVPWM.u_q = pid_output; // 误差越大，扭矩越大
    SguanSVPWM.u_d = 0.0f;

    // 实际电角度 = 机械角度 * 极对数 + 偏移
    SguanSVPWM.theta = normalize_angle((actual_angle_rad + alignment_angle_offset) * Pole_Pairs);
    // --- 5. 输出SVPWM ---
    generate_svpwm_waveforms();
}

/**
 * @description: [Mode4]FOC速度单环闭环控制
 * @param {float} target_speed_rad_s 目标速度 (rad/s)
 */
static void FOC_Velocity_SingleLoop(float target_speed_rad_s) {
    float mech_angle_rad;
    Sguan_ReadAngle(&mech_angle_rad);
    float actual_speed_rad_s;
    Sguan_FilteredAngularVelocity(&actual_speed_rad_s);

    // --- 1. 误差计算 ---
    float error = target_speed_rad_s - actual_speed_rad_s;

    // --- 2. PID控制器 ---
    float *Kp = &SguanFOC.Velocity_PID.Kp;
    float *Ki = &SguanFOC.Velocity_PID.Ki;
    float *Kd = &SguanFOC.Velocity_PID.Kd;
    float *Integral = &SguanFOC.Velocity_PID.Integral;
    float *Prev_error = &SguanFOC.Velocity_PID.Prev_error;
    float limit = SguanFOC.Velocity_PID.Output_limit;

    *Integral += error;
    if (*Integral > limit) *Integral = limit;
    if (*Integral < -limit) *Integral = -limit;

    float derivative = error - *Prev_error;
    *Prev_error = error;

    float pid_output = (*Kp * error) + (*Ki * (*Integral)) + (*Kd * derivative);

    // --- 3. 限幅 ---
    pid_output = constrain(pid_output,-limit, limit);

    // --- 4. 设置FOC电压 ---
    SguanSVPWM.u_q = pid_output;
    SguanSVPWM.u_d = 0.0f;

    // --- 5. 正确的角度补偿 ---
    // 基于速度的角度超前补偿
    float compensation = actual_speed_rad_s * OPTIMAL_DELAY_TIME;
    SguanSVPWM.theta = normalize_angle(
        (mech_angle_rad + alignment_angle_offset) * Pole_Pairs + compensation);

    // --- 6. 输出SVPWM ---
    generate_svpwm_waveforms();
}

/**
 * @description: [Mode5]FOC电流单环闭环控制
 * @param {float} target_iq 目标q轴电流 (A)
 */
static void FOC_Current_SingleLoop(float target_iq) {
    // --- 1. 获取实际Iq ---
    float actual_iq = FOC_Calculate_Iq();

    // --- 2. 误差计算 ---
    float error = target_iq - actual_iq;

    // --- 3. PID控制器 ---
    float *Kp = &SguanFOC.Current_PID.Kp;
    float *Ki = &SguanFOC.Current_PID.Ki;
    float *Kd = &SguanFOC.Current_PID.Kd;
    float *Integral = &SguanFOC.Current_PID.Integral;
    float *Prev_error = &SguanFOC.Current_PID.Prev_error;
    float limit = SguanFOC.Current_PID.Output_limit;

    *Integral += error;
    if (*Integral > limit) *Integral = limit;
    if (*Integral < -limit) *Integral = -limit;

    float derivative = error - *Prev_error;
    *Prev_error = error;

    float pid_output = (*Kp * error) + (*Ki * (*Integral)) + (*Kd * derivative);

    // --- 4. 限幅 ---
    pid_output = constrain(pid_output, -limit, limit);

    // --- 5. 设置FOC电压 ---
    SguanSVPWM.u_q = pid_output;
    SguanSVPWM.u_d = 0.0f;

    // 实际电角度 = (机械角度 + 偏移) * 极对数
    float mech_angle_rad;
    if (Sguan_ReadAngle(&mech_angle_rad)) {
        float actual_speed_rad_s;
        Sguan_FilteredAngularVelocity(&actual_speed_rad_s);
        // 基于速度的角度超前补偿
        float compensation = actual_speed_rad_s * OPTIMAL_DELAY_TIME;
        SguanSVPWM.theta = normalize_angle(
            (mech_angle_rad + alignment_angle_offset) * Pole_Pairs + compensation);
    }
    // --- 6. 输出SVPWM ---
    generate_svpwm_waveforms();
}

/**
 * @description: [Mode6] FOC速度-电流串级控制（电流环比速度环快7倍）
 * @param {float} target_speed_rad_s 目标速度 (rad/s)
 */
static void FOC_Velocity_Current_Cascade_FastInner(float target_speed_rad_s) {
    static uint8_t speed_loop_counter = 0;
    static float Iq_ref = 0.0f;  // 缓存速度环计算出的目标电流

    // --- 1. 外环速度PID (每7次执行一次) ---
    float actual_speed_rad_s;
    if (speed_loop_counter == 0) {
        Sguan_FilteredAngularVelocity(&actual_speed_rad_s);

        float v_error = target_speed_rad_s - actual_speed_rad_s;

        float *vKp = &SguanFOC.Velocity_Current_Cascade.Velocity.Kp;
        float *vKi = &SguanFOC.Velocity_Current_Cascade.Velocity.Ki;
        float *vKd = &SguanFOC.Velocity_Current_Cascade.Velocity.Kd;
        float *vIntegral = &SguanFOC.Velocity_Current_Cascade.Velocity.Integral;
        float *vPrev_error = &SguanFOC.Velocity_Current_Cascade.Velocity.Prev_error;
        float v_limit = SguanFOC.Velocity_Current_Cascade.Velocity.Output_limit;

        *vIntegral += v_error;
        if (*vIntegral > v_limit) *vIntegral = v_limit;
        if (*vIntegral < -v_limit) *vIntegral = -v_limit;

        float v_derivative = v_error - *vPrev_error;
        *vPrev_error = v_error;

        Iq_ref = (*vKp * v_error) + (*vKi * (*vIntegral)) + (*vKd * v_derivative);

        // 限幅目标电流
        Iq_ref = constrain(Iq_ref, -SguanFOC.Velocity_Current_Cascade.Current.Output_limit,
                                    SguanFOC.Velocity_Current_Cascade.Current.Output_limit);
    }

    // --- 2. 内环电流PID (每次都执行) ---
    float actual_iq = FOC_Calculate_Iq();
    float c_error = Iq_ref - actual_iq;

    float *cKp = &SguanFOC.Velocity_Current_Cascade.Current.Kp;
    float *cKi = &SguanFOC.Velocity_Current_Cascade.Current.Ki;
    float *cKd = &SguanFOC.Velocity_Current_Cascade.Current.Kd;
    float *cIntegral = &SguanFOC.Velocity_Current_Cascade.Current.Integral;
    float *cPrev_error = &SguanFOC.Velocity_Current_Cascade.Current.Prev_error;
    float c_limit = SguanFOC.Velocity_Current_Cascade.Current.Output_limit;

    *cIntegral += c_error;
    if (*cIntegral > c_limit) *cIntegral = c_limit;
    if (*cIntegral < -c_limit) *cIntegral = -c_limit;

    float c_derivative = c_error - *cPrev_error;
    *cPrev_error = c_error;

    float pid_output_current = (*cKp * c_error) + (*cKi * (*cIntegral)) + (*cKd * c_derivative);

    // 限幅
    pid_output_current = constrain(pid_output_current, -c_limit, c_limit);

    // --- 3. 设置FOC电压 ---
    SguanSVPWM.u_q = pid_output_current;
    SguanSVPWM.u_d = 0.0f;

    float mech_angle_rad;
    if (Sguan_ReadAngle(&mech_angle_rad)) {
        // 基于速度的角度超前补偿
        float compensation = actual_speed_rad_s * OPTIMAL_DELAY_TIME;
        SguanSVPWM.theta = normalize_angle(
            (mech_angle_rad + alignment_angle_offset) * Pole_Pairs + compensation);
    }

    generate_svpwm_waveforms();

    // --- 4. 更新速度环计数器 ---
    speed_loop_counter++;
    if (speed_loop_counter >= 7) {
        speed_loop_counter = 0;
    }
}

/**
 * @description: [Mode7] FOC位置-速度串级控制（速度环比位置环快7倍）
 * @param {float} target_angle_rad 目标位置 (rad，多圈角度)
 */
static void FOC_Position_Velocity_Cascade_FastInner(float target_angle_rad) {
    static uint8_t pos_loop_counter = 0;
    static float velocity_ref = 0.0f;  // 缓存位置环计算出的目标速度

    // --- 1. 外环位置PID (每7次执行一次) ---
    if (pos_loop_counter == 0) {
        float actual_angle_rad;
        if (!Sguan_ReadMultiTurnAngle(&actual_angle_rad)) {
            return; // 如果读取失败，直接退出
        }

        float p_error = target_angle_rad - actual_angle_rad;

        float *pKp = &SguanFOC.Position_Velocity_Cascade.Position.Kp;
        float *pKi = &SguanFOC.Position_Velocity_Cascade.Position.Ki;
        float *pKd = &SguanFOC.Position_Velocity_Cascade.Position.Kd;
        float *pIntegral = &SguanFOC.Position_Velocity_Cascade.Position.Integral;
        float *pPrev_error = &SguanFOC.Position_Velocity_Cascade.Position.Prev_error;
        float p_limit = SguanFOC.Position_Velocity_Cascade.Position.Output_limit;

        *pIntegral += p_error;
        if (*pIntegral > p_limit) *pIntegral = p_limit;
        if (*pIntegral < -p_limit) *pIntegral = -p_limit;

        float p_derivative = p_error - *pPrev_error;
        *pPrev_error = p_error;

        velocity_ref = (*pKp * p_error) + (*pKi * (*pIntegral)) + (*pKd * p_derivative);

        // 限幅目标速度
        velocity_ref = constrain(velocity_ref,
                                 -SguanFOC.Position_Velocity_Cascade.Velocity.Output_limit,
                                  SguanFOC.Position_Velocity_Cascade.Velocity.Output_limit);
    }

    // --- 2. 内环速度PID (每次执行) ---
    float actual_speed_rad_s;
    Sguan_FilteredAngularVelocity(&actual_speed_rad_s);

    float v_error = velocity_ref - actual_speed_rad_s;

    float *vKp = &SguanFOC.Position_Velocity_Cascade.Velocity.Kp;
    float *vKi = &SguanFOC.Position_Velocity_Cascade.Velocity.Ki;
    float *vKd = &SguanFOC.Position_Velocity_Cascade.Velocity.Kd;
    float *vIntegral = &SguanFOC.Position_Velocity_Cascade.Velocity.Integral;
    float *vPrev_error = &SguanFOC.Position_Velocity_Cascade.Velocity.Prev_error;
    float v_limit = SguanFOC.Position_Velocity_Cascade.Velocity.Output_limit;

    *vIntegral += v_error;
    if (*vIntegral > v_limit) *vIntegral = v_limit;
    if (*vIntegral < -v_limit) *vIntegral = -v_limit;

    float v_derivative = v_error - *vPrev_error;
    *vPrev_error = v_error;

    float pid_output_velocity = (*vKp * v_error) + (*vKi * (*vIntegral)) + (*vKd * v_derivative);

    // 限幅
    pid_output_velocity = constrain(pid_output_velocity, -v_limit, v_limit);

    // --- 3. 设置FOC电压 ---
    SguanSVPWM.u_q = 0.0f;
    SguanSVPWM.u_d = pid_output_velocity;

    float mech_angle_rad;
    if (Sguan_ReadAngle(&mech_angle_rad)) {
        SguanSVPWM.theta = normalize_angle((mech_angle_rad + alignment_angle_offset) * Pole_Pairs);
    }

    generate_svpwm_waveforms();

    // --- 4. 更新位置环计数器 ---
    pos_loop_counter++;
    if (pos_loop_counter >= 7) {
        pos_loop_counter = 0;
    }
}

/**
 * @description: [Mode8] FOC位置-速度-电流三环串级控制
 *               电流环比速度环快5倍，速度环比位置环快5倍
 * @param {float} target_angle_rad 目标位置 (rad，多圈角度)
 */
static void FOC_Position_Velocity_Current_Cascade_Triple(float target_angle_rad) {
    static uint8_t pos_counter = 0;   // 位置环计数器
    static uint8_t vel_counter = 0;   // 速度环计数器
    static float velocity_ref = 0.0f; // 缓存位置环输出
    static float Iq_ref = 0.0f;       // 缓存速度环输出

    // --- 1. 位置环 (每25次执行1次) ---
    if (pos_counter == 0) {
        float actual_angle_rad;
        if (Sguan_ReadMultiTurnAngle(&actual_angle_rad)) {
            float p_error = target_angle_rad - actual_angle_rad;

            float *pKp = &SguanFOC.Position_Velocity_Current_Cascade.Position.Kp;
            float *pKi = &SguanFOC.Position_Velocity_Current_Cascade.Position.Ki;
            float *pKd = &SguanFOC.Position_Velocity_Current_Cascade.Position.Kd;
            float *pIntegral = &SguanFOC.Position_Velocity_Current_Cascade.Position.Integral;
            float *pPrev_error = &SguanFOC.Position_Velocity_Current_Cascade.Position.Prev_error;
            float p_limit = SguanFOC.Position_Velocity_Current_Cascade.Position.Output_limit;

            *pIntegral += p_error;
            if (*pIntegral > p_limit) *pIntegral = p_limit;
            if (*pIntegral < -p_limit) *pIntegral = -p_limit;

            float p_derivative = p_error - *pPrev_error;
            *pPrev_error = p_error;

            velocity_ref = (*pKp * p_error) + (*pKi * (*pIntegral)) + (*pKd * p_derivative);

            // 限幅目标速度
            velocity_ref = constrain(velocity_ref,
                                     -SguanFOC.Position_Velocity_Current_Cascade.Velocity.Output_limit,
                                      SguanFOC.Position_Velocity_Current_Cascade.Velocity.Output_limit);
        }
    }

    // --- 2. 速度环 (每5次执行1次) ---
    if (vel_counter == 0) {
        float actual_speed_rad_s;
        Sguan_FilteredAngularVelocity(&actual_speed_rad_s);

        float v_error = velocity_ref - actual_speed_rad_s;

        float *vKp = &SguanFOC.Position_Velocity_Current_Cascade.Velocity.Kp;
        float *vKi = &SguanFOC.Position_Velocity_Current_Cascade.Velocity.Ki;
        float *vKd = &SguanFOC.Position_Velocity_Current_Cascade.Velocity.Kd;
        float *vIntegral = &SguanFOC.Position_Velocity_Current_Cascade.Velocity.Integral;
        float *vPrev_error = &SguanFOC.Position_Velocity_Current_Cascade.Velocity.Prev_error;
        float v_limit = SguanFOC.Position_Velocity_Current_Cascade.Velocity.Output_limit;

        *vIntegral += v_error;
        if (*vIntegral > v_limit) *vIntegral = v_limit;
        if (*vIntegral < -v_limit) *vIntegral = -v_limit;

        float v_derivative = v_error - *vPrev_error;
        *vPrev_error = v_error;

        Iq_ref = (*vKp * v_error) + (*vKi * (*vIntegral)) + (*vKd * v_derivative);

        // 限幅目标电流
        Iq_ref = constrain(Iq_ref,
                           -SguanFOC.Position_Velocity_Current_Cascade.Current.Output_limit,
                            SguanFOC.Position_Velocity_Current_Cascade.Current.Output_limit);
    }

    // --- 3. 电流环 (每次都执行) ---
    float actual_iq = FOC_Calculate_Iq();
    float c_error = Iq_ref - actual_iq;

    float *cKp = &SguanFOC.Position_Velocity_Current_Cascade.Current.Kp;
    float *cKi = &SguanFOC.Position_Velocity_Current_Cascade.Current.Ki;
    float *cKd = &SguanFOC.Position_Velocity_Current_Cascade.Current.Kd;
    float *cIntegral = &SguanFOC.Position_Velocity_Current_Cascade.Current.Integral;
    float *cPrev_error = &SguanFOC.Position_Velocity_Current_Cascade.Current.Prev_error;
    float c_limit = SguanFOC.Position_Velocity_Current_Cascade.Current.Output_limit;

    *cIntegral += c_error;
    if (*cIntegral > c_limit) *cIntegral = c_limit;
    if (*cIntegral < -c_limit) *cIntegral = -c_limit;

    float c_derivative = c_error - *cPrev_error;
    *cPrev_error = c_error;

    float pid_output_current = (*cKp * c_error) + (*cKi * (*cIntegral)) + (*cKd * c_derivative);

    // 限幅
    pid_output_current = constrain(pid_output_current, -c_limit, c_limit);

    // --- 4. 设置FOC电压 ---
    SguanSVPWM.u_q = 0.0f;
    SguanSVPWM.u_d = pid_output_current;

    float mech_angle_rad;
    if (Sguan_ReadAngle(&mech_angle_rad)) {
        SguanSVPWM.theta = normalize_angle((mech_angle_rad + alignment_angle_offset) * Pole_Pairs);
    }

    generate_svpwm_waveforms();

    // --- 5. 更新计数器 ---
    vel_counter++;
    if (vel_counter >= 5) vel_counter = 0;

    pos_counter++;
    if (pos_counter >= 25) pos_counter = 0;
}

// 弧度制电角度输入，输出电机打印信息(测试专用)
static void FOC_Pos_Loop(float angle_rad, float voltage) {
    float Real_Eanlge;
    Sguan_ReadAngle(&Real_Eanlge);
    // 1. 机械角度转电角度
    SguanSVPWM.theta = normalize_angle(angle_rad + alignment_angle_offset*Pole_Pairs);
    // 2. 设置电压（控制转矩）
    SguanSVPWM.u_q = voltage;
    SguanSVPWM.u_d = 0.0f;
    // 3. 生成SVPWM波形
    generate_svpwm_waveforms();
    // printf("%.5f,%.5f,%.5f\n",(Real_Eanlge + alignment_angle_offset)*Pole_Pairs,Adjustable_Data,0.0f);
}
// 弧度制电角度输入，输出电机打印信息(测试专用)
static void FOC_Vel_Loop(float voltage) {
    float num;
    Sguan_ReadAngle(&num);
    // 1. 机械角度转电角度
    float actual_speed_rad_s;
    Sguan_FilteredAngularVelocity(&actual_speed_rad_s);
    // 基于速度的角度超前补偿
    float compensation = actual_speed_rad_s * OPTIMAL_DELAY_TIME;
    SguanSVPWM.theta = normalize_angle(
        (num + alignment_angle_offset) * Pole_Pairs + compensation);
    // 2. 设置电压（控制转矩）
    SguanSVPWM.u_q = voltage;
    SguanSVPWM.u_d = 0.0f;
    // 3. 生成SVPWM波形
    generate_svpwm_waveforms();
    // printf("%.5f,%.5f,%.5f\n",(Real_Eanlge + alignment_angle_offset)*Pole_Pairs,Adjustable_Data,0.0f);
}

// 电机多环调控运行函数
void FOC_LoopHandler(void) {
    switch (FOC_Mode) {
        // --- 开环 ---
        case FOC_MODE_OPEN_POSITION:
            // 注意：开环角度用度数作为输入
            FOC_OpenPosition_Loop((FOC_Target_Position / (2.0f * PI)) * 360.0f, FOC_Target_Voltage);
            break;
        case FOC_MODE_OPEN_VELOCITY:
            FOC_OpenVelocity_Loop(FOC_Target_Speed, FOC_Target_Voltage);
            break;

        // --- 单环闭环 ---
        case FOC_MODE_POSITION_SINGLE:
            FOC_Position_SingleLoop(FOC_Target_Position);
            break;
        case FOC_MODE_VELOCITY_SINGLE:
            FOC_Velocity_SingleLoop(FOC_Target_Speed);
            break;
        case FOC_MODE_CURRENT_SINGLE:
            FOC_Current_SingleLoop(FOC_Target_Current);
            break;

        // --- 双环闭环 ---
        case FOC_MODE_POSITION_VELOCITY_CASCADE:
            FOC_Position_Velocity_Cascade_FastInner(FOC_Target_Position);
            break;
        case FOC_MODE_VELOCITY_CURRENT_CASCADE:
            FOC_Velocity_Current_Cascade_FastInner(FOC_Target_Speed);
            break;

        // --- 三环闭环 ---
        case FOC_MODE_POSITION_VELOCITY_CURRENT_CASCADE:
            FOC_Position_Velocity_Current_Cascade_Triple(FOC_Target_Position);
            break;
        default:
            // 默认空闲：关电机
            SguanSVPWM.u_q = 0.0f;
            SguanSVPWM.u_d = 0.0f;
            generate_svpwm_waveforms();
            break;
    }
}

/**
 * @description: 设置指定环路的PID参数
 * @param loop   "pos", "vel", "cur",
 *               "pos_vel", "vel_cur", "pos_vel_cur"
 * @param kp     比例系数
 * @param ki     积分系数
 * @param kd     微分系数
 * @param limit  输出限幅
 */
void FOC_SetPIDParams(const char *loop, float kp, float ki, float kd, float limit) {
    if (strcmp(loop, "pos") == 0) {
        SguanFOC.Position_PID.Kp = kp;
        SguanFOC.Position_PID.Ki = ki;
        SguanFOC.Position_PID.Kd = kd;
        SguanFOC.Position_PID.Output_limit = limit;
    }
    else if (strcmp(loop, "vel") == 0) {
        SguanFOC.Velocity_PID.Kp = kp;
        SguanFOC.Velocity_PID.Ki = ki;
        SguanFOC.Velocity_PID.Kd = kd;
        SguanFOC.Velocity_PID.Output_limit = limit;
    }
    else if (strcmp(loop, "cur") == 0) {
        SguanFOC.Current_PID.Kp = kp;
        SguanFOC.Current_PID.Ki = ki;
        SguanFOC.Current_PID.Kd = kd;
        SguanFOC.Current_PID.Output_limit = limit;
    }
    else if (strcmp(loop, "pos_vel") == 0) {
        SguanFOC.Position_Velocity_Cascade.Position.Kp = kp;
        SguanFOC.Position_Velocity_Cascade.Position.Ki = ki;
        SguanFOC.Position_Velocity_Cascade.Position.Kd = kd;
        SguanFOC.Position_Velocity_Cascade.Position.Output_limit = limit;
    }
    else if (strcmp(loop, "vel_cur") == 0) {
        SguanFOC.Velocity_Current_Cascade.Velocity.Kp = kp;
        SguanFOC.Velocity_Current_Cascade.Velocity.Ki = ki;
        SguanFOC.Velocity_Current_Cascade.Velocity.Kd = kd;
        SguanFOC.Velocity_Current_Cascade.Velocity.Output_limit = limit;
    }
    else if (strcmp(loop, "pos_vel_cur") == 0) {
        SguanFOC.Position_Velocity_Current_Cascade.Position.Kp = kp;
        SguanFOC.Position_Velocity_Current_Cascade.Position.Ki = ki;
        SguanFOC.Position_Velocity_Current_Cascade.Position.Kd = kd;
        SguanFOC.Position_Velocity_Current_Cascade.Position.Output_limit = limit;
    }
}

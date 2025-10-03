'''
Author: 星必尘Sguan
Date: 2025-10-03 20:03:51
LastEditors: 星必尘Sguan|3464647102@qq.com
LastEditTime: 2025-10-03 20:08:34
FilePath: \SguanFOC库\SguanFOC.py
Description: SguanFOC库v1.0(Python版本)

Copyright (c) 2025 by $JUST, All Rights Reserved. 
'''
import SguanUser_data
from SguanUser_data import *
import math

# 定义常量
PI = math.pi
Sqrt3 = math.sqrt(3)

# 全局变量
SguanSVPWM = {
    'u_d': 0.0,
    'u_q': 0.0,
    'theta': 0.0,
    'u_alpha': 0.0,
    'u_beta': 0.0,
    't_a': 0.0,
    't_b': 0.0,
    't_c': 0.0,
    'i_a': 0.0,
    'i_b': 0.0,
    'i_c': 0.0,
    'i_alpha': 0.0,
    'i_beta': 0.0,
    'i_d': 0.0,
    'i_q': 0.0,
    'sine': 0.0,
    'cosine': 0.0,
    'k_svpwm': 0.0
}

current_Iq = 0.0
electrical_angle = 0.0
last_time = 0
alignment_angle_offset = 0.0
FOC_Mode = 0
FOC_Target_Position = 0.0
FOC_Target_Voltage = 0.0
FOC_Target_Speed = 0.0
FOC_Target_Current = 0.0

# FOC结构体定义
SguanFOC = {
    'Position_PID': {
        'Kp': 0.0,
        'Ki': 0.0,
        'Kd': 0.0,
        'Integral': 0.0,
        'Prev_error': 0.0,
        'Output_limit': 0.0
    },
    'Velocity_PID': {
        'Kp': 0.0,
        'Ki': 0.0,
        'Kd': 0.0,
        'Integral': 0.0,
        'Prev_error': 0.0,
        'Output_limit': 0.0
    },
    'Current_PID': {
        'Kp': 0.0,
        'Ki': 0.0,
        'Kd': 0.0,
        'Integral': 0.0,
        'Prev_error': 0.0,
        'Output_limit': 0.0
    },
    'Velocity_Current_Cascade': {
        'Velocity': {
            'Kp': 0.0,
            'Ki': 0.0,
            'Kd': 0.0,
            'Integral': 0.0,
            'Prev_error': 0.0,
            'Output_limit': 0.0
        },
        'Current': {
            'Kp': 0.0,
            'Ki': 0.0,
            'Kd': 0.0,
            'Integral': 0.0,
            'Prev_error': 0.0,
            'Output_limit': 0.0
        }
    },
    'Position_Velocity_Cascade': {
        'Position': {
            'Kp': 0.0,
            'Ki': 0.0,
            'Kd': 0.0,
            'Integral': 0.0,
            'Prev_error': 0.0,
            'Output_limit': 0.0
        },
        'Velocity': {
            'Kp': 0.0,
            'Ki': 0.0,
            'Kd': 0.0,
            'Integral': 0.0,
            'Prev_error': 0.0,
            'Output_limit': 0.0
        }
    },
    'Position_Velocity_Current_Cascade': {
        'Position': {
            'Kp': 0.0,
            'Ki': 0.0,
            'Kd': 0.0,
            'Integral': 0.0,
            'Prev_error': 0.0,
            'Output_limit': 0.0
        },
        'Velocity': {
            'Kp': 0.0,
            'Ki': 0.0,
            'Kd': 0.0,
            'Integral': 0.0,
            'Prev_error': 0.0,
            'Output_limit': 0.0
        },
        'Current': {
            'Kp': 0.0,
            'Ki': 0.0,
            'Kd': 0.0,
            'Integral': 0.0,
            'Prev_error': 0.0,
            'Output_limit': 0.0
        }
    }
}

# FOC模式枚举
class FOC_MODE:
    FOC_MODE_NONE = 0
    FOC_MODE_OPEN_POSITION = 1
    FOC_MODE_OPEN_VELOCITY = 2
    FOC_MODE_POSITION_SINGLE = 3
    FOC_MODE_VELOCITY_SINGLE = 4
    FOC_MODE_CURRENT_SINGLE = 5
    FOC_MODE_POSITION_VELOCITY_CASCADE = 6
    FOC_MODE_VELOCITY_CURRENT_CASCADE = 7
    FOC_MODE_POSITION_VELOCITY_CURRENT_CASCADE = 8


def f1(x: float) -> float:
    """多项式逼近函数f1"""
    u = 1.3528548e-10
    u = u * x + -2.4703144e-08
    u = u * x + 2.7532926e-06
    u = u * x + -0.00019840381
    u = u * x + 0.0083333179
    return u * x + -0.16666666


def f2(x: float) -> float:
    """多项式逼近函数f2"""
    u = 1.7290616e-09
    u = u * x + -2.7093486e-07
    u = u * x + 2.4771643e-05
    u = u * x + -0.0013887906
    u = u * x + 0.041666519
    return u * x + -0.49999991


def fast_cos(x: float) -> float:
    """快速计算cosine值"""
    si = int(x * 0.31830988)  # 1/pi的近似值
    x = x - float(si) * PI
    if si & 1:
        x = x - PI if x > 0.0 else x + PI
    return 1.0 + x * x * f2(x * x)


def fast_sin(x: float) -> float:
    """快速计算sine值"""
    si = int(x * 0.31830988)
    x = x - float(si) * PI
    if si & 1:
        x = x - PI if x > 0.0 else x + PI
    return x + x * x * x * f1(x * x)


def fast_sin_cos(x: float) -> tuple[float, float]:
    """联合计算sin和cos值"""
    si = int(x * 0.31830988)
    x = x - float(si) * PI
    if si & 1:
        x = x - PI if x > 0.0 else x + PI
    sin_x = x + x * x * x * f1(x * x)
    cos_x = 1.0 + x * x * f2(x * x)
    return sin_x, cos_x


def kalman_filter_std(input_val: float, r: float, q: float) -> float:
    """一阶卡尔曼滤波器"""
    static = {'z': 0.0, 'p': 1.0}
    static['p'] += q
    g = static['p'] / (static['p'] + r)
    static['z'] += g * (input_val - static['z'])
    static['p'] = (1 - g) * static['p']
    return static['z']


def normalize_angle(angle: float) -> float:
    """将角度归一化到0-2π范围"""
    angle = angle % (2 * PI)
    if angle < 0:
        angle += 2 * PI
    return angle


def constrain(value: float, min_val: float, max_val: float) -> float:
    """限制值在min和max之间"""
    return max(min_val, min(value, max_val))


def svpwm(foc: dict) -> None:
    """SVPWM核心算法"""
    ts = 1.0  # PWM周期（归一化）
    u1 = foc['u_beta']
    u2 = -0.8660254037844386 * foc['u_alpha'] - 0.5 * foc['u_beta']
    u3 = 0.8660254037844386 * foc['u_alpha'] - 0.5 * foc['u_beta']
    
    sector = (1 if u1 > 0.0 else 0) + ((1 if u2 > 0.0 else 0) << 1) + ((1 if u3 > 0.0 else 0) << 2)
    
    if sector == 5:
        t4 = u3
        t6 = u1
        sum_t = t4 + t6
        if sum_t > ts:
            foc['k_svpwm'] = ts / sum_t
            t4 *= foc['k_svpwm']
            t6 *= foc['k_svpwm']
        t7 = (ts - t4 - t6) / 2
        foc['t_a'] = t4 + t6 + t7
        foc['t_b'] = t6 + t7
        foc['t_c'] = t7
    elif sector == 1:
        t2 = -u3
        t6 = -u2
        sum_t = t2 + t6
        if sum_t > ts:
            foc['k_svpwm'] = ts / sum_t
            t2 *= foc['k_svpwm']
            t6 *= foc['k_svpwm']
        t7 = (ts - t2 - t6) / 2
        foc['t_a'] = t6 + t7
        foc['t_b'] = t2 + t6 + t7
        foc['t_c'] = t7
    elif sector == 3:
        t2 = u1
        t3 = u2
        sum_t = t2 + t3
        if sum_t > ts:
            foc['k_svpwm'] = ts / sum_t
            t2 *= foc['k_svpwm']
            t3 *= foc['k_svpwm']
        t7 = (ts - t2 - t3) / 2
        foc['t_a'] = t7
        foc['t_b'] = t2 + t3 + t7
        foc['t_c'] = t3 + t7
    elif sector == 2:
        t1 = -u1
        t3 = -u3
        sum_t = t1 + t3
        if sum_t > ts:
            foc['k_svpwm'] = ts / sum_t
            t1 *= foc['k_svpwm']
            t3 *= foc['k_svpwm']
        t7 = (ts - t1 - t3) / 2
        foc['t_a'] = t7
        foc['t_b'] = t3 + t7
        foc['t_c'] = t1 + t3 + t7
    elif sector == 6:
        t1 = u2
        t5 = u3
        sum_t = t1 + t5
        if sum_t > ts:
            foc['k_svpwm'] = ts / sum_t
            t1 *= foc['k_svpwm']
            t5 *= foc['k_svpwm']
        t7 = (ts - t1 - t5) / 2
        foc['t_a'] = t5 + t7
        foc['t_b'] = t7
        foc['t_c'] = t1 + t5 + t7
    elif sector == 4:
        t4 = -u2
        t5 = -u1
        sum_t = t4 + t5
        if sum_t > ts:
            foc['k_svpwm'] = ts / sum_t
            t4 *= foc['k_svpwm']
            t5 *= foc['k_svpwm']
        t7 = (ts - t4 - t5) / 2
        foc['t_a'] = t4 + t5 + t7
        foc['t_b'] = t7
        foc['t_c'] = t5 + t7


def generate_svpwm_waveforms() -> None:
    """生成SVPWM波形并输出到PWM通道"""
    global SguanSVPWM
    # 逆Park变换: 将d-q电压转换为α-β电压
    sin_theta, cos_theta = fast_sin_cos(SguanSVPWM['theta'])
    SguanSVPWM['u_alpha'] = SguanSVPWM['u_d'] * cos_theta - SguanSVPWM['u_q'] * sin_theta
    SguanSVPWM['u_beta'] = SguanSVPWM['u_d'] * sin_theta + SguanSVPWM['u_q'] * cos_theta
    
    # 执行SVPWM计算
    svpwm(SguanSVPWM)
    
    # 将占空比输出到PWM（通过用户函数）
    from SguanUser_data import Sguan_PwmSet
    # 假设PWM通道1-3对应A、B、C相
    Sguan_PwmSet(1, int(SguanSVPWM['t_a'] * 4095))  # 假设12位PWM
    Sguan_PwmSet(2, int(SguanSVPWM['t_b'] * 4095))
    Sguan_PwmSet(3, int(SguanSVPWM['t_c'] * 4095))


def FOC_Calculate_Iq() -> float:
    """计算q轴电流Iq值（带卡尔曼滤波）"""
    global current_Iq, SguanSVPWM
    from SguanUser_data import Sguan_CurAcquisition
    
    # 采样三相电流
    Iu_Raw = Sguan_CurAcquisition(1) - Intermediate_Raw
    Iv_Raw = Sguan_CurAcquisition(0) - Intermediate_Raw
    
    # ADC转换为电流值
    ADC_to_Current = 3.3 / 4096.0 / Operational_Num / (Shunt_Resistor / 1000.0)
    Iu = ADC_to_Current * float(Iu_Raw)
    Iv = ADC_to_Current * float(Iv_Raw)
    
    # Clarke变换
    I_alpha = Iu
    I_beta = (Iu + 2.0 * Iv) * (1.0 / Sqrt3)
    
    # Park变换
    sin_theta, cos_theta = fast_sin_cos(SguanSVPWM['theta'])
    Raw_Iq = -I_alpha * sin_theta + I_beta * cos_theta
    
    # 卡尔曼滤波
    current_Iq = kalman_filter_std(Raw_Iq, M_NOISE, P_NOISE)
    return current_Iq


def FOC_EncoderAlignment() -> None:
    """电角度对齐（简化实现）"""
    global alignment_angle_offset
    # 实际应用中需要更复杂的对齐逻辑
    alignment_angle_offset = 0.0


def FOC_Init() -> None:
    """初始化FOC控制器的底层硬件"""
    global SguanSVPWM, current_Iq
    # 初始化FOC结构体
    SguanSVPWM = {k: 0.0 for k in SguanSVPWM}
    current_Iq = 0.0
    
    # 初始化硬件
    from SguanUser_data import (
        Sguan_TimerInit, Sguan_PosSensorInit,
        Sguan_CurSamplingInit, Sguan_UartInit
    )
    Sguan_TimerInit()
    Sguan_PosSensorInit()
    Sguan_CurSamplingInit()
    Sguan_UartInit()
    
    # 电角度对齐
    FOC_EncoderAlignment()


def FOC_OpenPosition_Loop(angle_deg: float, voltage: float) -> None:
    """FOC开环位置控制"""
    global SguanSVPWM
    mechanical_angle = (angle_deg / 360.0) * 2.0 * PI
    SguanSVPWM['theta'] = normalize_angle(mechanical_angle * Pole_Pairs)
    SguanSVPWM['u_q'] = 0.0
    SguanSVPWM['u_d'] = voltage
    generate_svpwm_waveforms()


def FOC_OpenVelocity_Loop(velocity_rad_s: float, voltage: float) -> None:
    """FOC开环速度控制"""
    global SguanSVPWM, electrical_angle, last_time
    from SguanUser_data import Sguan_UserGetTick
    
    current_time = Sguan_UserGetTick()
    delta_time_ms = float(current_time - last_time)
    last_time = current_time
    
    electrical_velocity = velocity_rad_s * Pole_Pairs
    angle_increment = electrical_velocity * (delta_time_ms / 1000.0)
    electrical_angle += angle_increment
    electrical_angle = normalize_angle(electrical_angle)
    
    SguanSVPWM['theta'] = electrical_angle
    SguanSVPWM['u_q'] = 0.0
    SguanSVPWM['u_d'] = voltage
    generate_svpwm_waveforms()


def FOC_Position_SingleLoop(target_angle_rad: float) -> None:
    """FOC位置单环闭环控制"""
    global SguanSVPWM
    from SguanUser_data import Sguan_ReadMultiTurnAngle
    
    actual_angle_rad = [0.0]
    Sguan_ReadMultiTurnAngle(actual_angle_rad)
    actual_angle_rad = actual_angle_rad[0]
    
    error = target_angle_rad - actual_angle_rad
    
    pid = SguanFOC['Position_PID']
    pid['Integral'] += error
    pid['Integral'] = constrain(pid['Integral'], -pid['Output_limit'], pid['Output_limit'])
    
    derivative = error - pid['Prev_error']
    pid['Prev_error'] = error
    
    pid_output = pid['Kp'] * error + pid['Ki'] * pid['Integral'] + pid['Kd'] * derivative
    pid_output = constrain(pid_output, -pid['Output_limit'], pid['Output_limit'])
    
    SguanSVPWM['u_q'] = pid_output
    SguanSVPWM['u_d'] = 0.0
    SguanSVPWM['theta'] = normalize_angle((actual_angle_rad + alignment_angle_offset) * Pole_Pairs)
    
    generate_svpwm_waveforms()


def FOC_Velocity_SingleLoop(target_speed_rad_s: float) -> None:
    """FOC速度单环闭环控制"""
    global SguanSVPWM
    from SguanUser_data import Sguan_ReadAngle, Sguan_FilteredAngularVelocity
    
    mech_angle_rad = [0.0]
    Sguan_ReadAngle(mech_angle_rad)
    mech_angle_rad = mech_angle_rad[0]
    
    actual_speed_rad_s = [0.0]
    Sguan_FilteredAngularVelocity(actual_speed_rad_s)
    actual_speed_rad_s = actual_speed_rad_s[0]
    
    error = target_speed_rad_s - actual_speed_rad_s
    
    pid = SguanFOC['Velocity_PID']
    pid['Integral'] += error
    pid['Integral'] = constrain(pid['Integral'], -pid['Output_limit'], pid['Output_limit'])
    
    derivative = error - pid['Prev_error']
    pid['Prev_error'] = error
    
    pid_output = pid['Kp'] * error + pid['Ki'] * pid['Integral'] + pid['Kd'] * derivative
    pid_output = constrain(pid_output, -pid['Output_limit'], pid['Output_limit'])
    
    SguanSVPWM['u_q'] = pid_output
    SguanSVPWM['u_d'] = 0.0
    
    compensation = actual_speed_rad_s * OPTIMAL_DELAY_TIME
    SguanSVPWM['theta'] = normalize_angle(
        (mech_angle_rad + alignment_angle_offset) * Pole_Pairs + compensation
    )
    
    generate_svpwm_waveforms()


def FOC_Current_SingleLoop(target_iq: float) -> None:
    """FOC电流单环闭环控制"""
    global SguanSVPWM
    from SguanUser_data import Sguan_ReadAngle, Sguan_FilteredAngularVelocity
    
    actual_iq = FOC_Calculate_Iq()
    error = target_iq - actual_iq
    
    pid = SguanFOC['Current_PID']
    pid['Integral'] += error
    pid['Integral'] = constrain(pid['Integral'], -pid['Output_limit'], pid['Output_limit'])
    
    derivative = error - pid['Prev_error']
    pid['Prev_error'] = error
    
    pid_output = pid['Kp'] * error + pid['Ki'] * pid['Integral'] + pid['Kd'] * derivative
    pid_output = constrain(pid_output, -pid['Output_limit'], pid['Output_limit'])
    
    SguanSVPWM['u_q'] = pid_output
    SguanSVPWM['u_d'] = 0.0
    
    mech_angle_rad = [0.0]
    if Sguan_ReadAngle(mech_angle_rad):
        mech_angle_rad = mech_angle_rad[0]
        actual_speed_rad_s = [0.0]
        Sguan_FilteredAngularVelocity(actual_speed_rad_s)
        actual_speed_rad_s = actual_speed_rad_s[0]
        
        compensation = actual_speed_rad_s * OPTIMAL_DELAY_TIME
        SguanSVPWM['theta'] = normalize_angle(
            (mech_angle_rad + alignment_angle_offset) * Pole_Pairs + compensation
        )
    
    generate_svpwm_waveforms()


def FOC_Position_Velocity_Cascade_FastInner(target_angle_rad: float) -> None:
    """FOC位置-速度串级控制"""
    global SguanSVPWM
    from SguanUser_data import Sguan_ReadMultiTurnAngle, Sguan_FilteredAngularVelocity
    
    # 静态变量模拟
    static = {'pos_loop_counter': 0, 'velocity_ref': 0.0}
    
    if static['pos_loop_counter'] == 0:
        actual_angle_rad = [0.0]
        if not Sguan_ReadMultiTurnAngle(actual_angle_rad):
            return
        actual_angle_rad = actual_angle_rad[0]
        
        error = target_angle_rad - actual_angle_rad
        pos_pid = SguanFOC['Position_Velocity_Cascade']['Position']
        
        pos_pid['Integral'] += error
        pos_pid['Integral'] = constrain(pos_pid['Integral'], -pos_pid['Output_limit'], pos_pid['Output_limit'])
        
        derivative = error - pos_pid['Prev_error']
        pos_pid['Prev_error'] = error
        
        static['velocity_ref'] = pos_pid['Kp'] * error + pos_pid['Ki'] * pos_pid['Integral'] + pos_pid['Kd'] * derivative
        static['velocity_ref'] = constrain(
            static['velocity_ref'],
            -SguanFOC['Position_Velocity_Cascade']['Velocity']['Output_limit'],
            SguanFOC['Position_Velocity_Cascade']['Velocity']['Output_limit']
        )
    
    # 速度环
    actual_speed_rad_s = [0.0]
    Sguan_FilteredAngularVelocity(actual_speed_rad_s)
    actual_speed_rad_s = actual_speed_rad_s[0]
    
    error = static['velocity_ref'] - actual_speed_rad_s
    vel_pid = SguanFOC['Position_Velocity_Cascade']['Velocity']
    
    vel_pid['Integral'] += error
    vel_pid['Integral'] = constrain(vel_pid['Integral'], -vel_pid['Output_limit'], vel_pid['Output_limit'])
    
    derivative = error - vel_pid['Prev_error']
    vel_pid['Prev_error'] = error
    
    pid_output = vel_pid['Kp'] * error + vel_pid['Ki'] * vel_pid['Integral'] + vel_pid['Kd'] * derivative
    pid_output = constrain(pid_output, -vel_pid['Output_limit'], vel_pid['Output_limit'])
    
    SguanSVPWM['u_q'] = 0.0
    SguanSVPWM['u_d'] = pid_output
    
    mech_angle_rad = [0.0]
    if Sguan_ReadAngle(mech_angle_rad):
        mech_angle_rad = mech_angle_rad[0]
        SguanSVPWM['theta'] = normalize_angle(
            (mech_angle_rad + alignment_angle_offset) * Pole_Pairs
        )
    
    generate_svpwm_waveforms()
    
    static['pos_loop_counter'] = (static['pos_loop_counter'] + 1) % 7


def FOC_Velocity_Current_Cascade_FastInner(target_speed_rad_s: float) -> None:
    """FOC速度-电流串级控制"""
    global SguanSVPWM
    from SguanUser_data import Sguan_FilteredAngularVelocity, Sguan_ReadAngle
    
    # 静态变量模拟
    static = {'speed_loop_counter': 0, 'Iq_ref': 0.0}
    
    if static['speed_loop_counter'] == 0:
        actual_speed_rad_s = [0.0]
        Sguan_FilteredAngularVelocity(actual_speed_rad_s)
        actual_speed_rad_s = actual_speed_rad_s[0]
        
        error = target_speed_rad_s - actual_speed_rad_s
        vel_pid = SguanFOC['Velocity_Current_Cascade']['Velocity']
        
        vel_pid['Integral'] += error
        vel_pid['Integral'] = constrain(vel_pid['Integral'], -vel_pid['Output_limit'], vel_pid['Output_limit'])
        
        derivative = error - vel_pid['Prev_error']
        vel_pid['Prev_error'] = error
        
        static['Iq_ref'] = vel_pid['Kp'] * error + vel_pid['Ki'] * vel_pid['Integral'] + vel_pid['Kd'] * derivative
        static['Iq_ref'] = constrain(
            static['Iq_ref'],
            -SguanFOC['Velocity_Current_Cascade']['Current']['Output_limit'],
            SguanFOC['Velocity_Current_Cascade']['Current']['Output_limit']
        )
    
    # 电流环
    actual_iq = FOC_Calculate_Iq()
    error = static['Iq_ref'] - actual_iq
    cur_pid = SguanFOC['Velocity_Current_Cascade']['Current']
    
    cur_pid['Integral'] += error
    cur_pid['Integral'] = constrain(cur_pid['Integral'], -cur_pid['Output_limit'], cur_pid['Output_limit'])
    
    derivative = error - cur_pid['Prev_error']
    cur_pid['Prev_error'] = error
    
    pid_output = cur_pid['Kp'] * error + cur_pid['Ki'] * cur_pid['Integral'] + cur_pid['Kd'] * derivative
    pid_output = constrain(pid_output, -cur_pid['Output_limit'], cur_pid['Output_limit'])
    
    SguanSVPWM['u_q'] = pid_output
    SguanSVPWM['u_d'] = 0.0
    
    mech_angle_rad = [0.0]
    if Sguan_ReadAngle(mech_angle_rad):
        mech_angle_rad = mech_angle_rad[0]
        actual_speed = [0.0]
        Sguan_FilteredAngularVelocity(actual_speed)
        compensation = actual_speed[0] * OPTIMAL_DELAY_TIME
        SguanSVPWM['theta'] = normalize_angle(
            (mech_angle_rad + alignment_angle_offset) * Pole_Pairs + compensation
        )
    
    generate_svpwm_waveforms()
    static['speed_loop_counter'] = (static['speed_loop_counter'] + 1) % 7


def FOC_Position_Velocity_Current_Cascade_Triple(target_angle_rad: float) -> None:
    """FOC位置-速度-电流三环串级控制"""
    global SguanSVPWM
    from SguanUser_data import Sguan_ReadMultiTurnAngle, Sguan_FilteredAngularVelocity, Sguan_ReadAngle
    
    # 静态变量模拟
    static = {'pos_counter': 0, 'vel_counter': 0, 'velocity_ref': 0.0, 'Iq_ref': 0.0}
    
    # 位置环
    if static['pos_counter'] == 0:
        actual_angle_rad = [0.0]
        if Sguan_ReadMultiTurnAngle(actual_angle_rad):
            actual_angle_rad = actual_angle_rad[0]
            error = target_angle_rad - actual_angle_rad
            pos_pid = SguanFOC['Position_Velocity_Current_Cascade']['Position']
            
            pos_pid['Integral'] += error
            pos_pid['Integral'] = constrain(pos_pid['Integral'], -pos_pid['Output_limit'], pos_pid['Output_limit'])
            
            derivative = error - pos_pid['Prev_error']
            pos_pid['Prev_error'] = error
            
            static['velocity_ref'] = pos_pid['Kp'] * error + pos_pid['Ki'] * pos_pid['Integral'] + pos_pid['Kd'] * derivative
            static['velocity_ref'] = constrain(
                static['velocity_ref'],
                -SguanFOC['Position_Velocity_Current_Cascade']['Velocity']['Output_limit'],
                SguanFOC['Position_Velocity_Current_Cascade']['Velocity']['Output_limit']
            )
    
    # 速度环
    if static['vel_counter'] == 0:
        actual_speed_rad_s = [0.0]
        Sguan_FilteredAngularVelocity(actual_speed_rad_s)
        actual_speed_rad_s = actual_speed_rad_s[0]
        
        error = static['velocity_ref'] - actual_speed_rad_s
        vel_pid = SguanFOC['Position_Velocity_Current_Cascade']['Velocity']
        
        vel_pid['Integral'] += error
        vel_pid['Integral'] = constrain(vel_pid['Integral'], -vel_pid['Output_limit'], vel_pid['Output_limit'])
        
        derivative = error - vel_pid['Prev_error']
        vel_pid['Prev_error'] = error
        
        static['Iq_ref'] = vel_pid['Kp'] * error + vel_pid['Ki'] * vel_pid['Integral'] + vel_pid['Kd'] * derivative
        static['Iq_ref'] = constrain(
            static['Iq_ref'],
            -SguanFOC['Position_Velocity_Current_Cascade']['Current']['Output_limit'],
            SguanFOC['Position_Velocity_Current_Cascade']['Current']['Output_limit']
        )
    
    # 电流环
    actual_iq = FOC_Calculate_Iq()
    error = static['Iq_ref'] - actual_iq
    cur_pid = SguanFOC['Position_Velocity_Current_Cascade']['Current']
    
    cur_pid['Integral'] += error
    cur_pid['Integral'] = constrain(cur_pid['Integral'], -cur_pid['Output_limit'], cur_pid['Output_limit'])
    
    derivative = error - cur_pid['Prev_error']
    cur_pid['Prev_error'] = error
    
    pid_output = cur_pid['Kp'] * error + cur_pid['Ki'] * cur_pid['Integral'] + cur_pid['Kd'] * derivative
    pid_output = constrain(pid_output, -cur_pid['Output_limit'], cur_pid['Output_limit'])
    
    SguanSVPWM['u_q'] = 0.0
    SguanSVPWM['u_d'] = pid_output
    
    mech_angle_rad = [0.0]
    if Sguan_ReadAngle(mech_angle_rad):
        mech_angle_rad = mech_angle_rad[0]
        SguanSVPWM['theta'] = normalize_angle(
            (mech_angle_rad + alignment_angle_offset) * Pole_Pairs
        )
    
    generate_svpwm_waveforms()
    
    # 更新计数器
    static['vel_counter'] = (static['vel_counter'] + 1) % 5
    static['pos_counter'] = (static['pos_counter'] + 1) % 25


def FOC_Pos_Loop(angle_rad: float, voltage: float) -> None:
    """测试用：弧度制电角度输入"""
    global SguanSVPWM
    from SguanUser_data import Sguan_ReadAngle
    
    real_angle = [0.0]
    Sguan_ReadAngle(real_angle)
    SguanSVPWM['theta'] = normalize_angle(angle_rad + alignment_angle_offset * Pole_Pairs)
    SguanSVPWM['u_q'] = voltage
    SguanSVPWM['u_d'] = 0.0
    generate_svpwm_waveforms()


def FOC_Vel_Loop(voltage: float) -> None:
    """测试用：速度环控制"""
    global SguanSVPWM
    from SguanUser_data import Sguan_ReadAngle, Sguan_FilteredAngularVelocity
    
    num = [0.0]
    Sguan_ReadAngle(num)
    actual_speed_rad_s = [0.0]
    Sguan_FilteredAngularVelocity(actual_speed_rad_s)
    actual_speed_rad_s = actual_speed_rad_s[0]
    
    compensation = actual_speed_rad_s * OPTIMAL_DELAY_TIME
    SguanSVPWM['theta'] = normalize_angle(
        (num[0] + alignment_angle_offset) * Pole_Pairs + compensation
    )
    SguanSVPWM['u_q'] = voltage
    SguanSVPWM['u_d'] = 0.0
    generate_svpwm_waveforms()


def FOC_LoopHandler() -> None:
    """电机多环调控运行函数"""
    global FOC_Mode, FOC_Target_Position, FOC_Target_Voltage, FOC_Target_Speed, FOC_Target_Current
    
    mode = FOC_Mode
    if mode == FOC_MODE.FOC_MODE_OPEN_POSITION:
        FOC_OpenPosition_Loop((FOC_Target_Position / (2.0 * PI)) * 360.0, FOC_Target_Voltage)
    elif mode == FOC_MODE.FOC_MODE_OPEN_VELOCITY:
        FOC_OpenVelocity_Loop(FOC_Target_Speed, FOC_Target_Voltage)
    elif mode == FOC_MODE.FOC_MODE_POSITION_SINGLE:
        FOC_Position_SingleLoop(FOC_Target_Position)
    elif mode == FOC_MODE.FOC_MODE_VELOCITY_SINGLE:
        FOC_Velocity_SingleLoop(FOC_Target_Speed)
    elif mode == FOC_MODE.FOC_MODE_CURRENT_SINGLE:
        FOC_Current_SingleLoop(FOC_Target_Current)
    elif mode == FOC_MODE.FOC_MODE_POSITION_VELOCITY_CASCADE:
        FOC_Position_Velocity_Cascade_FastInner(FOC_Target_Position)
    elif mode == FOC_MODE.FOC_MODE_VELOCITY_CURRENT_CASCADE:
        FOC_Velocity_Current_Cascade_FastInner(FOC_Target_Speed)
    elif mode == FOC_MODE.FOC_MODE_POSITION_VELOCITY_CURRENT_CASCADE:
        FOC_Position_Velocity_Current_Cascade_Triple(FOC_Target_Position)
    else:
        # 默认关闭电机
        SguanSVPWM['u_q'] = 0.0
        SguanSVPWM['u_d'] = 0.0
        generate_svpwm_waveforms()

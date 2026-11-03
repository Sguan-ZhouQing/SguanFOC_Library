#ifndef __UNITLIB_DESIGN_H
#define __UNITLIB_DESIGN_H

#define DESIGN_NUM_TRANSFER1        5
#define DESIGN_NUM_TRANSFER2        2
#define DESIGN_NUM_TRANSFER3        1
#define DESIGN_NUM_TRANSFER4        0
#define DESIGN_NUM_TRANSFER5        0
#define DESIGN_NUM_INTEGRATOR       0
#define DESIGN_NUM_DERIVATIVE       0
#define DESIGN_NUM_CURVE            0
#define DESIGN_NUM_DFT              0
#define DESIGN_NUM_HALL             0
#define DESIGN_NUM_LADRC1           0
#define DESIGN_NUM_LADRC2           0
#define DESIGN_NUM_SMC              0
#define DESIGN_NUM_DPCC             0
#define DESIGN_NUM_PIR              0
#define DESIGN_NUM_PID              0
#define DESIGN_NUM_PLL              0
#define DESIGN_NUM_LPF1             0
#define DESIGN_NUM_LPF2             0
#define DESIGN_NUM_HPF1             0
#define DESIGN_NUM_HPF2             0
#define DESIGN_NUM_BPF1             0
#define DESIGN_NUM_BPF2             0
#define DESIGN_NUM_SOGI             0
#define DESIGN_NUM_NF               0
#define DESIGN_NUM_TPNF             0
#define DESIGN_NUM_DOB              0
#define DESIGN_NUM_RLS              0
#define DESIGN_NUM_SMO              0
#define DESIGN_NUM_NLFO             0
#define DESIGN_NUM_VCFO             0
#define DESIGN_NUM_HFI              0
#define DESIGN_NUM_ROLO             0
#define DESIGN_NUM_MARS             0
#define DESIGN_NUM_EKF              0
#define DESIGN_NUM_DELAY1           0
#define DESIGN_NUM_DELAY2           0
#define DESIGN_NUM_DELAY3           0

#define DESIGN_NUM_FLOAT            0
#define DESIGN_NUM_INT8             0
#define DESIGN_NUM_UINT8            0
#define DESIGN_NUM_INT16            0
#define DESIGN_NUM_UINT16           0
#define DESIGN_NUM_INT32            0
#define DESIGN_NUM_UINT32           0

#define DESIGN_COUNT_MOTOR          1

// 宏定义(0指float，1指double，2指Q15，3指Q31)
#define DESIGN_MODE_MATH            3

#define DESIGN_BASE_TIME            0.001953125
#define DESIGN_BASE_VOLTAGE         1024.0
#define DESIGN_BASE_CURRENT         64.0


#endif // UNITLIB_DESIGN_H

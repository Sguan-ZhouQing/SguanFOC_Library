#ifndef __UNITLIB_DESIGN_H
#define __UNITLIB_DESIGN_H

#define DESIGN_NUM_TRANSFER1        5
#define DESIGN_NUM_TRANSFER2        2
#define DESIGN_NUM_TRANSFER3        1
#define DESIGN_NUM_TRANSFER4        1
#define DESIGN_NUM_TRANSFER5        1
#define DESIGN_NUM_INTEGRATOR       1
#define DESIGN_NUM_DERIVATIVE       1
#define DESIGN_NUM_CURVE            1
#define DESIGN_NUM_DFT              1
#define DESIGN_NUM_HALL             1
#define DESIGN_NUM_LADRC1           1
#define DESIGN_NUM_LADRC2           1
#define DESIGN_NUM_SMC              1
#define DESIGN_NUM_DPCC             1
#define DESIGN_NUM_PIR              1
#define DESIGN_NUM_PID              1
#define DESIGN_NUM_PLL              1
#define DESIGN_NUM_LPF1             1
#define DESIGN_NUM_LPF2             1
#define DESIGN_NUM_HPF1             1
#define DESIGN_NUM_HPF2             1
#define DESIGN_NUM_BPF1             1
#define DESIGN_NUM_BPF2             1
#define DESIGN_NUM_SOGI             1
#define DESIGN_NUM_NF               1
#define DESIGN_NUM_TPNF             1
#define DESIGN_NUM_DOB              1
#define DESIGN_NUM_RLS              1
#define DESIGN_NUM_SMO              1
#define DESIGN_NUM_NLFO             1
#define DESIGN_NUM_VCFO             1
#define DESIGN_NUM_HFI              1
#define DESIGN_NUM_ROLO             1
#define DESIGN_NUM_MARS             1
#define DESIGN_NUM_EKF              1
#define DESIGN_NUM_DELAY1           1
#define DESIGN_NUM_DELAY2           1
#define DESIGN_NUM_DELAY3           1

#define DESIGN_NUM_FLOAT            1
#define DESIGN_NUM_INT8             1
#define DESIGN_NUM_UINT8            1
#define DESIGN_NUM_INT16            1
#define DESIGN_NUM_UINT16           1
#define DESIGN_NUM_INT32            1
#define DESIGN_NUM_UINT32           1

#define DESIGN_COUNT_MOTOR          1

// 宏定义(0指float，1指double，2指Q15，3指Q31)
#define DESIGN_MODE_MATH            0

#define DESIGN_BASE_TIME            0.001953125
#define DESIGN_BASE_VOLTAGE         1024.0
#define DESIGN_BASE_CURRENT         64.0
#define DESIGN_BASE_OTHER           512.0


#endif // UNITLIB_DESIGN_H

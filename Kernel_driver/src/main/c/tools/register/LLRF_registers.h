#ifndef LLRF_REG_DEFS_H_
#define LLRF_REG_DEFS_H_

#define INIT_DONE                  1<<0 
#define UPDATE_PARAM_PULSE         1<<1 
#define NEW_PULSE_TYPE_PULSE       1<<2 
#define UPDATE_FF_PULSE            1<<3
#define UPDATE_SP_PULSE            1<<4 
#define PULSE_COMMING              1<<5
#define PULSE_START                1<<6
#define PULSE_END                  1<<7
#define PMS                        1<<8
#define GET_PARAM                  1<<9
#define SW_RST                     1<<10





#define OFFSET 0x400
//long word address offset
#define LLRF_ID                    0 + OFFSET
#define LLRF_INST_ID               1 + OFFSET
#define LLRF_GOP                   2 + OFFSET
#define LLRF_GIP                   3 + OFFSET
#define LLRF_GIP_S                 4 + OFFSET
#define LLRF_GIP_C                 5 + OFFSET
#define LLRF_PI_1_K                6 + OFFSET
#define LLRF_PI_1_TS_DIV_TI        7 + OFFSET
#define LLRF_PI_1_SAT_MAX          8 + OFFSET
#define LLRF_PI_1_SAT_MIN          9 + OFFSET
#define LLRF_PI_1_CTRL             10 + OFFSET
#define LLRF_PI_1_FIXED_SP         11 + OFFSET
#define LLRF_PI_1_FIXED_FF         12 + OFFSET
#define LLRF_PI_2_K                13 + OFFSET
#define LLRF_PI_2_TS_DIV_TI        14 + OFFSET
#define LLRF_PI_2_SAT_MAX          15 + OFFSET
#define LLRF_PI_2_SAT_MIN          16 + OFFSET
#define LLRF_PI_2_CTRL             17 + OFFSET
#define LLRF_PI_2_FIXED_SP         18 + OFFSET
#define LLRF_PI_2_FIXED_FF         19 + OFFSET
#define LLRF_IQ_CTRL               20 + OFFSET
#define LLRF_IQ_ANGLE              21 + OFFSET
#define LLRF_VM_ANGLE              22 + OFFSET
#define LLRF_VM_CTRL               23 + OFFSET
#define LLRF_VM_MAG_LIMIT          24 + OFFSET
#define LLRF_SAMPLE_CNT            25 + OFFSET
#define LLRF_PULSE_START_CNT       26 + OFFSET
#define LLRF_PULSE_ACTIVE_CNT      27 + OFFSET
#define LLRF_SPFF_CTRL_1_PARAM     28 + OFFSET
#define LLRF_SPFF_CTRL_2_PARAM     29 + OFFSET
#define LLRF_MEM_CTRL_1_PARAM      30 + OFFSET
#define LLRF_MEM_CTRL_2_PARAM      31 + OFFSET
#define LLRF_MEM_CTRL_3_PARAM      32 + OFFSET
#define LLRF_MEM_CTRL_4_PARAM      33 + OFFSET
#define LLRF_MEM_CTRL_5_PARAM      34 + OFFSET
#define LLRF_MEM_CTRL_6_PARAM      35 + OFFSET
#define LLRF_MEM_CTRL_7_PARAM      36 + OFFSET
#define LLRF_MEM_CTRL_8_PARAM      37 + OFFSET
#define LLRF_PI_ERR_CNT            38 + OFFSET
#define LLRF_BOARD_SETUP           39 + OFFSET
#define LLRF_NEAR_IQ_1_PARAM       40 + OFFSET
#define LLRF_NEAR_IQ_2_PARAM       41 + OFFSET
#define LLRF_NEAR_IQ_DATA          42 + OFFSET
#define LLRF_NEAR_IQ_ADDR          43 + OFFSET
#define LLRF_FILTER_S              44 + OFFSET
#define LLRF_FILTER_C              45 + OFFSET
#define LLRF_FILTER_A_CTRL         46 + OFFSET
#define LLRF_CAV_MA                47 + OFFSET
#define LLRF_REF_MA                48 + OFFSET
#define LLRF_MON_PARAM_1           49 + OFFSET
#define LLRF_MON_PARAM_2           50 + OFFSET
#define LLRF_MON_LIMIT_1           51 + OFFSET
#define LLRF_MON_LIMIT_2           52 + OFFSET
#define LLRF_MON_LIMIT_3           53 + OFFSET
#define LLRF_MON_LIMIT_4           54 + OFFSET
#define LLRF_MON_STATUS            55 + OFFSET
#define LLRF_MON_STATUS_MAG_1      56 + OFFSET
#define LLRF_MON_STATUS_MAG_2      57 + OFFSET
#define LLRF_MON_STATUS_MAG_3      58 + OFFSET
#define LLRF_MON_STATUS_MAG_4      59 + OFFSET
#define LLRF_VM_PREDIST_R0         60 + OFFSET
#define LLRF_VM_PREDIST_R1         61 + OFFSET
#define LLRF_VM_PREDIST_DC         62 + OFFSET
#define LLRF_NOTCH_A               63 + OFFSET
#define LLRF_NOTCH_B               64 + OFFSET
#define LLRF_NOTCH_CTRL            65 + OFFSET
#define LLRF_IQ_LP_A               66 + OFFSET
#define LLRF_IQ_LP_B               67 + OFFSET
#define LLRF_IQ_LP_CTRL            68 + OFFSET
#define LLRF_IL_LP_COEF            69 + OFFSET
#define LLRF_IL_PI_PARAM           70 + OFFSET
#define LLRF_IL_PI_I_START         71 + OFFSET
#define LLRF_IL_PHASE_CTRL         72 + OFFSET
#define LLRF_LIN_LUT_ADDR          73 + OFFSET
#define LLRF_LIN_LUT_DATA          74 + OFFSET
#define LLRF_IQ_DEBUG1             75 + OFFSET
#define LLRF_IQ_DEBUG2             76 + OFFSET
#define LLRF_IQ_DEBUG3             77 + OFFSET
#define LLRF_IQ_DEBUG4             78 + OFFSET



#endif
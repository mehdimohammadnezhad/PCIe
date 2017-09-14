#ifndef SIS8300_REG_H_
#define SIS8300_REG_H_

/* All bit definitions are bit numbers. */

/* Register map of bar 0. */
#define SIS8300_IDENTIFIER_VERSION_REG              0x00
#define SIS8300_SERIAL_NUMBER_REG                   0x01
#define SIS8300_XILINX_JTAG_REG                     0x02

#define SIS8300_USER_CONTROL_STATUS_REG             0x04
#define SIS8300_FIRMWARE_OPTIONS_REG                0x05

/* Firmware options bits */
#define SIS8300_FPGA_SX_1GByte_Memory               (1<<8)
#define SIS8300_DUAL_CHANNEL_SAMPLING               (1<<2)
#define SIS8300_RINGBUFFER_DELAY_EN                 (1<<1)
#define SIS8300_TRIGGER_BLOCK_EN                    (1<<0)

#define SIS8300_ACQUISITION_CONTROL_STATUS_REG      0x10
#define SIS8300_SAMPLE_CONTROL_REG                  0x11
#define SIS8300_MLVDS_IO_CONTROL_REG                0x12
#define SIS8300_HARLINK_IN_OUT_CONTROL_REG          0x13

#define SIS8300_CLOCK_DISTRIBUTION_MUX_REG          0x40
#define SIS8300_AD9510_SPI_REG                      0x41
#define SIS8300_CLOCK_MULTIPLIER_SPI_REG            0x42
#define SIS8300_MGTCLK_SYNTH_I2C_REG                0x43
#define SIS8300_FLASH_SPI_REG                       0x44

#define SIS8300_DAC_CONTROL_REG                     0x45
#define SIS8300_DAC_DATA_REG                        0x46

#define SIS8300_RTM_I2C_BUS_REG                     0x47

#define SIS8300_ADC_SPI_REG                         0x48
#define SIS8300_ADC_INPUT_TAP_DELAY_REG             0x49

#define SIS8300_VIRTEX5_SYSTEM_MONITOR_DATA_REG     0x90
#define SIS8300_VIRTEX5_SYSTEM_MONITOR_ADDR_REG     0x91
#define SIS8300_VIRTEX5_SYSTEM_MONITOR_CTRL_REG     0x92

#define SIS8300_MASTER_RESET_REG                    0xFF

#define SIS8300_TRIGGER_SETUP_CH1_REG               0x100
#define SIS8300_TRIGGER_SETUP_CH2_REG               0x101
#define SIS8300_TRIGGER_SETUP_CH3_REG               0x102
#define SIS8300_TRIGGER_SETUP_CH4_REG               0x103
#define SIS8300_TRIGGER_SETUP_CH5_REG               0x104
#define SIS8300_TRIGGER_SETUP_CH6_REG               0x105
#define SIS8300_TRIGGER_SETUP_CH7_REG               0x106
#define SIS8300_TRIGGER_SETUP_CH8_REG               0x107
#define SIS8300_TRIGGER_SETUP_CH9_REG               0x108
#define SIS8300_TRIGGER_SETUP_CH10_REG              0x109

#define SIS8300_TRIGGER_THRESHOLD_CH1_REG           0x110
#define SIS8300_TRIGGER_THRESHOLD_CH2_REG           0x111
#define SIS8300_TRIGGER_THRESHOLD_CH3_REG           0x112
#define SIS8300_TRIGGER_THRESHOLD_CH4_REG           0x113
#define SIS8300_TRIGGER_THRESHOLD_CH5_REG           0x114
#define SIS8300_TRIGGER_THRESHOLD_CH6_REG           0x115
#define SIS8300_TRIGGER_THRESHOLD_CH7_REG           0x116
#define SIS8300_TRIGGER_THRESHOLD_CH8_REG           0x117
#define SIS8300_TRIGGER_THRESHOLD_CH9_REG           0x118
#define SIS8300_TRIGGER_THRESHOLD_CH10_REG          0x119

#define SIS8300_SAMPLE_START_ADDRESS_CH1_REG        0x120
#define SIS8300_SAMPLE_START_ADDRESS_CH2_REG        0x121
#define SIS8300_SAMPLE_START_ADDRESS_CH3_REG        0x122
#define SIS8300_SAMPLE_START_ADDRESS_CH4_REG        0x123
#define SIS8300_SAMPLE_START_ADDRESS_CH5_REG        0x124
#define SIS8300_SAMPLE_START_ADDRESS_CH6_REG        0x125
#define SIS8300_SAMPLE_START_ADDRESS_CH7_REG        0x126
#define SIS8300_SAMPLE_START_ADDRESS_CH8_REG        0x127
#define SIS8300_SAMPLE_START_ADDRESS_CH9_REG        0x128
#define SIS8300_SAMPLE_START_ADDRESS_CH10_REG       0x129

#define SIS8300_SAMPLE_LENGTH_REG                   0x12A
#define SIS8300_PRETRIGGER_DELAY_REG                0x12B

#define SIS8300_TEST_HISTO_MEM_ADDR                 0x12C
#define SIS8300_TEST_HISTO_MEM_DATA_WR              0x12D
#define SIS8300_TEST_HISTO_CONTROL                  0x12E
#define SIS8300_RTM_LVDS_IO_CONTROL_REG             0x12F

#define DMA_READ_DST_ADR_LO32                       0x200
#define DMA_READ_DST_ADR_HI32                       0x201
#define DMA_READ_SRC_ADR_LO32                       0x202
#define DMA_READ_LEN                                0x203
#define DMA_READ_CTRL                               0x204

#define DMA_WRITE_SRC_ADR_LO32                      0x210
#define DMA_WRITE_SRC_ADR_HI32                      0x211
#define DMA_WRITE_DST_ADR_LO32                      0x212
#define DMA_WRITE_LEN                               0x213
#define DMA_WRITE_CTRL                              0x214

#define SIS8300_PCIE_REQUEST_NUM                    0x215

#define IRQ_ENABLE                                  0x220
#define IRQ_STATUS                                  0x221
#define IRQ_CLEAR                                   0x222
#define IRQ_REFRESH                                 0x223

#define SIS8300_DDR2_ACCESS_CONTROL                 0x230



/* DMA_READ_CTRL bits (bit numbers). */
#define DMA_READ_START              0    /* write access */
#define DMA_READ_RUNNING            0    /* read access */

/* DMA_READ_CTRL bits. */
#define DMA_WRITE_START             0    /* write access */
#define DMA_WRITE_RUNNING           0    /* read access*/

/* IRQ bitmasks. */
#define IRQ_MASTER_ENABLE           0x0000c003
#define IRQ_MASTER_DISABLE          0xc0030000
#define IRQ_MASTER_CLEAR            0x0000c003

/* IRQ_ENABLE & IRQ_STATUS & IRQ_CLEAR bits. */
#define DMA_READ_DONE               0
#define DMA_WRITE_DONE              1
#define DAQ_IRQ                     14
#define USER_IRQ                    15

/* DDR2_ACCESS_CONTROL bits. */
#define DDR2_PCIE_TEST_ENABLE       0

/* SIS8300_TRIGGER_SETUP_CHx_REG bits. */
#define CH_SETUP_PEAK               0
#define CH_SETUP_SUMG               8
#define CH_SETUP_PULSE              16
#define CH_SETUP_MODE               24
#define CH_SETUP_CONDITION          25
#define CH_SETUP_ENABLE             26

/* SIS8300_SAMPLE_CONTROL_REG masks. */
#define SAMPLE_CONTROL_CH_DIS       0x3FF
#define SAMPLE_CONTROL_TRIGGER      ~0x3FF

/* External trigger source registers masks. */
#define EXT_TRG_IO                  0xFFFF0000

/* AD9510 control registers. */
#define AD9510_GENERATE_FUNCTION_PULSE_CMD          0x80000000
#define AD9510_GENERATE_SPI_RW_CMD                  0x40000000
#define AD9510_SPI_SET_FUNCTION_SYNCH_FPGA_CLK69    0x10000000
#define AD9510_SPI_SELECT_NO2                       0x01000000
#define AD9510_SPI_READ_CYCLE                       0x00800000

/* Flash SPI programming bitmasks. */
#define FLASH_SPI_RD_BLK_FIFO       (1 << 9)
#define FLASH_SPI_RD_BLK_EN         (1 << 10)
#define FLASH_SPI_WR_BLK_FILL       (1 << 11)
#define FLASH_SPI_EXCH              (1 << 12)
#define FLASH_SPI_WR_BLK_EN         (1 << 13)
#define FLASH_SPI_CS                (1 << 14)
#define FLASH_SPI_MUX_EN            (1 << 15)
#define FLASH_SPI_BUSY              (1 << 31)

#define FLASH_SPI_UNWIND            << 16

#define FLASH_SPI_PROGRAM_PAGE      0x2
#define FLASH_SPI_READ_DATA         0x3
#define FLASH_SPI_READ_STATUS       0x5
#define FLASH_SPI_WRITE_ENABLE      0x6
#define FLASH_SPI_BLOCK_ERASE       0xD8

#define FLASH_SPI_FLASH_BUSY        0x1


/* Device model identifiers and memory sizes based on firmware oprions. */
#define SIS8300_SIS8300             0x8300
#define SIS8300_SIS8300L            0x8301
#define SIS8300_SIS8300L2           0x8302
#define SIS8300_SIS8300KU           0x8303
#define SIS8300_2GB_MEMORY          0x80000000
#define SIS8300_1GB_MEMORY          0x40000000
#define SIS8300_512MB_MEMORY        0x20000000

#endif /* SIS8300_REG_H_ */

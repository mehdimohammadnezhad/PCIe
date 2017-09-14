/**
 * @file sis8300drv_rtm.h
 * @brief Header for RTM control related functions
 */


#ifndef SIS8300DRV_RTM_H_
#define SIS8300DRV_RTM_H_


#include "sis8300drv_utils.h"


#define SIS8300_I2C_LOGIC_BUSY          0x80000000
#define SIS8300_I2C_BYTE_READ_CYCLE     0x2000
#define SIS8300_I2C_BYTE_WRITE_CYCLE    0x1000
#define SIS8300_I2C_STOP                0x800
#define SIS8300_I2C_REPEAT_START        0x400
#define SIS8300_I2C_START               0x200
#define SIS8300_I2C_MASTER_ACK          0x100
#define SIS8300_I2C_DATA_MASK           0xff
#define SIS8300_I2C_TIMEOUT             1000
#define SIS8300_I2C_READ                0x1
#define SIS8300_I2C_WRITE               0x0

#define SIS8300_PCA9535_SPI_BIT         0x02
#define SIS8300_PCA9535_CLK_BIT         0x01
#define SIS8300_PCA9535_I2C_ADDR        0x20

#define SIS8300_PCA9535_ATT_DATA_BITS   6
#define SIS8300_PCA9535_CMDAC_DATA_BITS 24
#define SIS8300_PCA9535_CMDDAC_BIT      0x8

#define SIS8300_LTC2493_I2C_ADDR        0x34
#define SIS8300_LTC2493_AD8363_TEMP_CMD        0x80B8
#define SIS8300_LTC2493_LTC2493_TEMP_CMD       0xC0B0

int sis8300drv_i2c_byte_read(sis8300drv_dev *sisdevice,
        uint32_t i2c_reg_addr,
        uint32_t *data_byte,
        uint32_t ack,
        int timeout);

int sis8300drv_i2c_byte_write(sis8300drv_dev *sisdevice,
        uint32_t i2c_reg_addr,
        uint32_t data_byte,
        uint32_t *ack,
        int timeout);

int sis8300drv_i2c_transfer_and_wait(sis8300drv_dev *sisdevice,
        uint32_t i2c_reg_addr,
        uint32_t *val,
        int timeout);

int sis8300drv_i2c_request_send(sis8300drv_dev *sisdevice,
        uint32_t i2c_reg_addr,
        uint32_t val,
        int timeout);

int sis8300drv_i2c_rw(sis8300drv_dev *sisdevice,
        uint32_t rw, uint32_t
        i2c_reg_addr,
        uint32_t i2c_bus_addr,
        int command_byte,
        unsigned *data_bytes,
        int nr_bytes);


#endif /* SIS8300DRV_RTM_H_ */

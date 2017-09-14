/**
 * @file sis8300drv.h
 * @brief Header file for the sis8300 digitizer userspace library.
 */


#ifndef SIS8300DRV_H_
#define SIS8300DRV_H_

#ifdef __cplusplus
extern "C" {
#endif


/* Device and DAQ related constants. */

#define SIS8300DRV_INT_CLK_FREQ     250000000   /**< Internal clock frequency in Hz. */
#define SIS8300DRV_NUM_AI_CHANNELS  10          /**< Number of AI channels on the device. */
#define SIS8300DRV_NUM_AO_CHANNELS  2           /**< Number of AO channels on the device. */
#define SIS8300DRV_NUM_HAR_CHANNELS 4           /**< Number of Harlink DI/DO channels on the device. */
#define SIS8300DRV_MAX_PRETRIG      2046        /**< Maximum number of pre-trigger samples. */
#define SIS8300DRV_NUM_FP_TRG       4           /**< Number of trigger lines on the front panel. */
#define SIS8300DRV_NUM_BP_TRG       8           /**< Number of trigger lines on the backplane. */
#define SIS8300DRV_BLOCK_SAMPLES    16          /**< Length of a device memory block in samples, granularity for setting number of samples. */


/* I2C communication with RTM related constants. */

#define SIS8300DRV_DWC8VM1_CMDAC_DEFOPT     0x352   /** < Default Common Mode Voltage for DWC8VM1 RTM = 1.7 V */
#define SIS8300DRV_RTM_ATT_0                0x63
#define SIS8300DRV_RTM_ATT_MAX              0x0

#define SIS8300DRV_RTM_ATT_AI_STEP_SIZE     0.50
#define SIS8300DRV_RTM_ATT_VM_STEP_SIZE     0.25
#define SIS8300DRV_RTM_ATT_AI_MAX           -31.50
#define SIS8300DRV_RTM_ATT_VM_MAX           -15.75

#define SIS8300DRV_RTM_ATT_CH0              0
#define SIS8300DRV_RTM_ATT_CH1              1
#define SIS8300DRV_RTM_ATT_CH2              2
#define SIS8300DRV_RTM_ATT_CH3              3
#define SIS8300DRV_RTM_ATT_CH4              4
#define SIS8300DRV_RTM_ATT_CH5              5
#define SIS8300DRV_RTM_ATT_CH6              6
#define SIS8300DRV_RTM_ATT_CH7              7
#define SIS8300DRV_RTM_ATT_CH8              8
#define SIS8300DRV_RTM_ATT_VM               8
#define SIS8300DRV_RTM_ATT_CH9              9
#define SIS8300DRV_RTM_CMDDAC               9

/* Clock divider constants */
#define SIS8300DRV_CLKDIV_MIN           1       /**< Minimum clock divider */
#define SIS8300DRV_CLKDIV_MAX           32      /**< Maximum clock divider*/
#define SIS8300DRV_CLKDIV_INT_MIN       2       /**< Minimum clock divider for internal clock */


/**
 * @brief Enumeration of possible clock sources.
 */
typedef enum {
    clk_src_internal,       /**< Internal clock - must use divider greater than one. */
    clk_src_rtm2,           /**< Clock from RTM clk2 clock line. */
    clk_src_sma,            /**< Clock from frontpanel SMA connector. */
    clk_src_harlink,        /**< Clock from frontpanel Harlink connector. */
    clk_src_backplane_a,    /**< Clock from line A on the backplane. */
    clk_src_backplane_b,    /**< Clock from line B on the backplane. */
    clk_src_rtm01           /**< Clock from RTM clk0 and clk1 clock lines. */
} sis8300drv_clk_src;


/**
 * @brief clock divider type.
 */
typedef unsigned char sis8300drv_clk_div;


/**
 * @brief Enumeration of possible trigger sources.
 */
typedef enum {
    trg_src_soft,           /**< Software trigger (on-demand acquisition). */
    trg_src_external,       /**< External trigger (harlink connector or backplane trigger lines). */
    trg_src_internal        /**< Internal trigger (signal on one of analog input channels). */
} sis8300drv_trg_src;


/**
 * @brief Enumeration of possible external trigger sources.
 */
typedef enum {
    trg_ext_harlink,        /**< Frontpanel harlink input connector. */
    trg_ext_mlvds           /**< Backplane trigger lines. */
} sis8300drv_trg_ext;


/**
 * @brief Enumeration of possible irq types.
 */
typedef enum {
    irq_type_daq,           /**< Daq-done irq that fires at the end of acquisition. */
    irq_type_usr            /**< User-defined irq defined in custom firmware. */
} sis8300drv_irq_type;


/**
 * @brief Enumeration of possible RTM types.
 */
typedef enum {
    rtm_none      = 0,
    rtm_sis8900   = 1,
    rtm_dwc8vm1   = 2,
    rtm_ds8vm1    = 3,
    rtm_dwc8300lf = 4,
} sis8300drv_rtm;

/**
 * @brief Enumeration of possible RTM temperature readout sources.
 */
typedef enum {
    rtm_temp_ad8363 = 0,
    rtm_temp_ltc2493 = 1
} sis8300drv_rtm_temp;

/**
 * @brief Enumeration of possible error codes.
 */
typedef enum {
    status_success          =  0,   /**< Operation successful. */
    status_device_armed     = -1,   /**< The device is armed. */
    status_argument_range   = -2,   /**< Argument out of range. */
    status_argument_invalid = -3,   /**< Invalid argument choice. */
    status_device_access    = -4,   /**< Error in reading/writing device registers. */
    status_read             = -5,   /**< Error in reading device memory. */
    status_write            = -6,   /**< Error in writing to device memory. */
    status_internal         = -7,   /**< Internal library error. */
    status_no_device        = -8,   /**< Device not opened. */
    status_device_state     = -9,   /**< Device in an undefined state. */
    status_irq_error        = -10,  /**< Waiting for irq resulted in error from device. */
    status_irq_timeout      = -11,  /**< Waiting for irq timeout. */
    status_irq_release      = -12,  /**< Waiting for irq was stopped. */
    status_i2c_busy         = -13,  /**< i2c bus busy. */
    status_i2c_nack         = -14,  /**< i2c transfer not acknowledged. */
    status_spi_busy         = -15,  /**< spi interface busy. */
    status_flash_busy       = -16,  /**< Flash chip busy. */
    status_flash_failed     = -17,  /**< Flashing new image failed. */
    status_incompatible     = -18,  /**< Incompatible device. */
    status_unknown          = -19
} sis8300drv_status;


/**
 * @brief Enumeration human readable strings corresponding to error codes.
 */
static const char *sis8300drv_str[] = {
    "operation successful",
    "the device is armed",
    "argument out of range",
    "invalid argument choice",
    "error reading/writing device registers",
    "error reading device memory",
    "error writing to device memory",
    "internal library error",
    "device not found",
    "device in an undefined state",
    "irq wait failed",
    "irq timeout",
    "irq stopped",
    "i2c busy",
    "i2c not acknowledged",
    "spi busy",
    "flash busy",
    "flashing image failed",
    "incompatible device",
    "unknown error code"
};

/**
 * @brief Convert numeric error codes to human readable string.
 * @param [in] status Error code.
 *
 * @retval Error string
 */
static inline const char *sis8300drv_strerror(int status) {
    if (status > 0 || status < status_unknown) {
        status = status_unknown;
    }
    return sis8300drv_str[-status];
}


/**
 * @brief Library user context struct.
 */
typedef struct t_sis8300drv_usr {
    const char  *file;      /**< Name of the device file node in /dev. */
    void        *device;    /**< Device private context.  */
} sis8300drv_usr;


/* Device and DAQ related functions. */

int sis8300drv_open_device(sis8300drv_usr *sisuser);

int sis8300drv_close_device(sis8300drv_usr *sisuser);

int sis8300drv_init_adc(sis8300drv_usr *sisuser);

int sis8300drv_init_dac(sis8300drv_usr *sisuser);

int sis8300drv_master_reset(sis8300drv_usr *sisuser);

int sis8300drv_arm_device(sis8300drv_usr *sisuser);

int sis8300drv_disarm_device(sis8300drv_usr *sisuser);

int sis8300drv_wait_acq_end(sis8300drv_usr *sisuser);

int sis8300drv_wait_remove(sis8300drv_usr *sisuser);

int sis8300drv_wait_irq(sis8300drv_usr *sisuser,
        sis8300drv_irq_type irq_type,
        unsigned timeout);

int sis8300drv_release_irq(sis8300drv_usr *sisuser,
        sis8300drv_irq_type irq_type);

int sis8300drv_read_ai(sis8300drv_usr *sisuser,
        unsigned channel,
        void *data);

int sis8300drv_write_ao(sis8300drv_usr *sisuser,
        unsigned channel,
        double data);

int sis8300drv_read_ao(sis8300drv_usr *sisuser,
        unsigned channel,
        double *data);

int sis8300drv_read_harlink(sis8300drv_usr *sisuser,
        unsigned *data);

int sis8300drv_get_channel_mask(sis8300drv_usr *sisuser,
        unsigned *channel_mask);

int sis8300drv_set_channel_mask(sis8300drv_usr *sisuser,
        unsigned channel_mask);

int sis8300drv_get_nsamples(sis8300drv_usr *sisuser,
        unsigned *nsamples);

int sis8300drv_set_nsamples(sis8300drv_usr *sisuser,
        unsigned nsamples);

int sis8300drv_get_npretrig(sis8300drv_usr *sisuser,
        unsigned *npretrig);

int sis8300drv_set_npretrig(sis8300drv_usr *sisuser,
        unsigned npretrig);

int sis8300drv_get_clock_source(sis8300drv_usr *sisuser,
        sis8300drv_clk_src *clk_src);

int sis8300drv_set_clock_source(sis8300drv_usr *sisuser,
        sis8300drv_clk_src clk_src);

int sis8300drv_get_clock_divider(sis8300drv_usr *sisuser,
        sis8300drv_clk_div *clk_div);

int sis8300drv_set_clock_divider(sis8300drv_usr *sisuser,
        sis8300drv_clk_div clk_div);

int sis8300drv_get_trigger_source(sis8300drv_usr *sisuser,
        sis8300drv_trg_src *trg_src);

int sis8300drv_set_trigger_source(sis8300drv_usr *sisuser,
        sis8300drv_trg_src trg_src);

int sis8300drv_get_external_setup(sis8300drv_usr *sisuser,
        sis8300drv_trg_ext trg_ext,
        unsigned *trg_mask,
        unsigned *edge_mask);

int sis8300drv_set_external_setup(sis8300drv_usr *sisuser,
        sis8300drv_trg_ext trg_ext,
        unsigned trg_mask,
        unsigned edge_mask);

int sis8300drv_set_internal_setup(sis8300drv_usr *sisuser,
        unsigned channel,
        unsigned enable,
        unsigned mode,
        unsigned threshold,
        unsigned condition,
        unsigned pulse_length,
        unsigned sum_gap,
        unsigned peaking_time);

int sis8300drv_get_internal_setup(sis8300drv_usr *sisuser,
        unsigned channel,
        unsigned *enable,
        unsigned *mode,
        unsigned *threshold,
        unsigned *condition,
        unsigned *pulse_length,
        unsigned *sum_gap,
        unsigned *peaking_time);

int sis8300drv_get_serial(sis8300drv_usr *sisuser,
        unsigned *serial);

int sis8300drv_get_fw_version(sis8300drv_usr *sisuser,
        unsigned *fw_version);

int sis8300drv_get_fw_options(sis8300drv_usr *sisuser,
        unsigned *fw_options);
        
int sis8300drv_get_space(sis8300drv_usr *sisuser,
        unsigned *space);

int sis8300drv_write_ram(sis8300drv_usr *sisuser,
        unsigned offset,
        unsigned size,
        void *data);
        
int sis8300drv_read_ram(sis8300drv_usr *sisuser,
        unsigned offset,
        unsigned size,
        void *data);

int sis8300drv_reg_write(sis8300drv_usr *sisuser,
        unsigned address,
        unsigned data);

int sis8300drv_reg_read(sis8300drv_usr *sisuser,
        unsigned address,
        unsigned *data);


/* Firmware flash update related functions. */

int sis8300drv_read_fw_image(sis8300drv_usr *sisuser, 
        unsigned size, 
        void *data, 
        void (*callback)(unsigned offset));
        
int sis8300drv_write_fw_image(sis8300drv_usr *sisuser, 
        unsigned size, 
        void *data, 
        void (*callback)(unsigned offset));


/* I2C communication with RTM related functions. */

int sis8300drv_i2c_rtm_attenuator_set(sis8300drv_usr *sisuser,
        sis8300drv_rtm rtm,
        unsigned att_num,
        unsigned att_idx);

int sis8300drv_i2c_rtm_vm_common_mode_set(sis8300drv_usr *sisuser,
        sis8300drv_rtm rtm,
        unsigned val);

int sis8300drv_rtm_vm_output_enable(sis8300drv_usr *sisuser,
        sis8300drv_rtm rtm,
        unsigned enable);

int sis8300drv_i2c_rtm_temperature_get(sis8300drv_usr *sisuser,
		sis8300drv_rtm rtm, sis8300drv_rtm_temp source, double *value);

#ifdef __cplusplus
}
#endif

#endif /* SIS8300DRV_H_ */


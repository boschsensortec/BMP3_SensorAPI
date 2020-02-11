/**
* Copyright (c) 2020 Bosch Sensortec GmbH. All rights reserved.
*
* BSD-3-Clause
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* 1. Redistributions of source code must retain the above copyright
*    notice, this list of conditions and the following disclaimer.
*
* 2. Redistributions in binary form must reproduce the above copyright
*    notice, this list of conditions and the following disclaimer in the
*    documentation and/or other materials provided with the distribution.
*
* 3. Neither the name of the copyright holder nor the names of its
*    contributors may be used to endorse or promote products derived from
*    this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
* "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
* FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
* COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
* INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
* (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
* HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
* STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
* IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
*
* @file bmp3.h
* @date 10/01/2020
* @version  1.2.1
*
*/
/*! @file bmp3.h, bmp3_defs.h
 * @brief Sensor driver for BMP3 sensor 
*/

/*!
 * @defgroup BMP3 SENSOR API
 * @{*/
#ifndef BMP3_H_
#define BMP3_H_

/* Header includes */
//#include "bmp3_defs.h"

/*! @file bmp3_defs.h
 * @brief Sensor driver for BMP3 sensor */

/*!
 * @defgroup BMP3 SENSOR API
 * @brief
 * @{*/

/*! CPP guard */
#ifdef __cplusplus
extern "C" {
#endif

  
/*************************** Common macros   *****************************/
#include <stdint.h>
#if !defined(UINT8_C) && !defined(INT8_C)
#define INT8_C(x)   ((int8_t)(x))
#define UINT8_C(x)  ((uint8_t)(x))
#endif

#if !defined(UINT16_C) && !defined(INT16_C)
#define INT16_C(x)  ((int16_t)(x))
#define UINT16_C(x) ((uint16_t)(x))
#endif

#if !defined(INT32_C) && !defined(UINT32_C)
#define INT32_C(x)  ((int32_t)(x))
#define UINT32_C(x) ((uint32_t)(x))
#endif

#if !defined(INT64_C) && !defined(UINT64_C)
#define INT64_C(x)  ((int64_t)(x))
#define UINT64_C(x) ((uint64_t)(x))
#endif

/**@}*/
/**\name C standard macros */
#ifndef NULL
#ifdef __cplusplus
#define NULL  0
#else
#define NULL  ((void *) 0)
#endif
#endif

#ifndef TRUE
#define TRUE  UINT8_C(1)
#endif

#ifndef FALSE
#define FALSE UINT8_C(0)
#endif

/********************************************************/
/**\name Compiler switch macros */
/**\name Uncomment the below line to use floating-point compensation */
#ifndef BMP3_DOUBLE_PRECISION_COMPENSATION

/* #define BMP3_DOUBLE_PRECISION_COMPENSATION*/
#endif

/********************************************************/
/**\name Macro definitions */
/**\name I2C addresses */
#define BMP3_I2C_ADDR_PRIM                UINT8_C(0x76)
#define BMP3_I2C_ADDR_SEC                 UINT8_C(0x77)

/**\name BMP3 chip identifier */
#define BMP3_CHIP_ID                      UINT8_C(0x50)

/**\name BMP3 pressure settling time (micro secs)*/
#define BMP3_PRESS_SETTLE_TIME            UINT16_C(392)

/**\name BMP3 temperature settling time (micro secs) */
#define BMP3_TEMP_SETTLE_TIME             UINT16_C(313)

/**\name BMP3 adc conversion time (micro secs) */
#define BMP3_ADC_CONV_TIME                UINT16_C(2000)

/**\name Register Address */
#define BMP3_CHIP_ID_ADDR                 UINT8_C(0x00)
#define BMP3_ERR_REG_ADDR                 UINT8_C(0x02)
#define BMP3_SENS_STATUS_REG_ADDR         UINT8_C(0x03)
#define BMP3_DATA_ADDR                    UINT8_C(0x04)
#define BMP3_EVENT_ADDR                   UINT8_C(0x10)
#define BMP3_INT_STATUS_REG_ADDR          UINT8_C(0x11)
#define BMP3_FIFO_LENGTH_ADDR             UINT8_C(0x12)
#define BMP3_FIFO_DATA_ADDR               UINT8_C(0x14)
#define BMP3_FIFO_WM_ADDR                 UINT8_C(0x15)
#define BMP3_FIFO_CONFIG_1_ADDR           UINT8_C(0x17)
#define BMP3_FIFO_CONFIG_2_ADDR           UINT8_C(0x18)
#define BMP3_INT_CTRL_ADDR                UINT8_C(0x19)
#define BMP3_IF_CONF_ADDR                 UINT8_C(0x1A)
#define BMP3_PWR_CTRL_ADDR                UINT8_C(0x1B)
#define BMP3_OSR_ADDR                     UINT8_C(0X1C)
#define BMP3_CALIB_DATA_ADDR              UINT8_C(0x31)
#define BMP3_CMD_ADDR                     UINT8_C(0x7E)

/**\name Error status macros */
#define BMP3_FATAL_ERR                    UINT8_C(0x01)
#define BMP3_CMD_ERR                      UINT8_C(0x02)
#define BMP3_CONF_ERR                     UINT8_C(0x04)

/**\name Status macros */
#define BMP3_CMD_RDY                      UINT8_C(0x10)
#define BMP3_DRDY_PRESS                   UINT8_C(0x20)
#define BMP3_DRDY_TEMP                    UINT8_C(0x40)

/**\name Power mode macros */
#define BMP3_SLEEP_MODE                   UINT8_C(0x00)
#define BMP3_FORCED_MODE                  UINT8_C(0x01)
#define BMP3_NORMAL_MODE                  UINT8_C(0x03)

/**\name FIFO related macros */
/**\name FIFO enable  */
#define BMP3_ENABLE                       UINT8_C(0x01)
#define BMP3_DISABLE                      UINT8_C(0x00)

/**\name Interrupt pin configuration macros */
/**\name Open drain */
#define BMP3_INT_PIN_OPEN_DRAIN           UINT8_C(0x01)
#define BMP3_INT_PIN_PUSH_PULL            UINT8_C(0x00)

/**\name Level */
#define BMP3_INT_PIN_ACTIVE_HIGH          UINT8_C(0x01)
#define BMP3_INT_PIN_ACTIVE_LOW           UINT8_C(0x00)

/**\name Latch */
#define BMP3_INT_PIN_LATCH                UINT8_C(0x01)
#define BMP3_INT_PIN_NON_LATCH            UINT8_C(0x00)

/**\name Advance settings  */
/**\name I2c watch dog timer period selection */
#define BMP3_I2C_WDT_SHORT_1_25_MS        UINT8_C(0x00)
#define BMP3_I2C_WDT_LONG_40_MS           UINT8_C(0x01)

/**\name FIFO Sub-sampling macros */
#define BMP3_FIFO_NO_SUBSAMPLING          UINT8_C(0x00)
#define BMP3_FIFO_SUBSAMPLING_2X          UINT8_C(0x01)
#define BMP3_FIFO_SUBSAMPLING_4X          UINT8_C(0x02)
#define BMP3_FIFO_SUBSAMPLING_8X          UINT8_C(0x03)
#define BMP3_FIFO_SUBSAMPLING_16X         UINT8_C(0x04)
#define BMP3_FIFO_SUBSAMPLING_32X         UINT8_C(0x05)
#define BMP3_FIFO_SUBSAMPLING_64X         UINT8_C(0x06)
#define BMP3_FIFO_SUBSAMPLING_128X        UINT8_C(0x07)

/**\name Over sampling macros */
#define BMP3_NO_OVERSAMPLING              UINT8_C(0x00)
#define BMP3_OVERSAMPLING_2X              UINT8_C(0x01)
#define BMP3_OVERSAMPLING_4X              UINT8_C(0x02)
#define BMP3_OVERSAMPLING_8X              UINT8_C(0x03)
#define BMP3_OVERSAMPLING_16X             UINT8_C(0x04)
#define BMP3_OVERSAMPLING_32X             UINT8_C(0x05)

/**\name Filter setting macros */
#define BMP3_IIR_FILTER_DISABLE           UINT8_C(0x00)
#define BMP3_IIR_FILTER_COEFF_1           UINT8_C(0x01)
#define BMP3_IIR_FILTER_COEFF_3           UINT8_C(0x02)
#define BMP3_IIR_FILTER_COEFF_7           UINT8_C(0x03)
#define BMP3_IIR_FILTER_COEFF_15          UINT8_C(0x04)
#define BMP3_IIR_FILTER_COEFF_31          UINT8_C(0x05)
#define BMP3_IIR_FILTER_COEFF_63          UINT8_C(0x06)
#define BMP3_IIR_FILTER_COEFF_127         UINT8_C(0x07)

/**\name Odr setting macros */
#define BMP3_ODR_200_HZ                   UINT8_C(0x00)
#define BMP3_ODR_100_HZ                   UINT8_C(0x01)
#define BMP3_ODR_50_HZ                    UINT8_C(0x02)
#define BMP3_ODR_25_HZ                    UINT8_C(0x03)
#define BMP3_ODR_12_5_HZ                  UINT8_C(0x04)
#define BMP3_ODR_6_25_HZ                  UINT8_C(0x05)
#define BMP3_ODR_3_1_HZ                   UINT8_C(0x06)
#define BMP3_ODR_1_5_HZ                   UINT8_C(0x07)
#define BMP3_ODR_0_78_HZ                  UINT8_C(0x08)
#define BMP3_ODR_0_39_HZ                  UINT8_C(0x09)
#define BMP3_ODR_0_2_HZ                   UINT8_C(0x0A)
#define BMP3_ODR_0_1_HZ                   UINT8_C(0x0B)
#define BMP3_ODR_0_05_HZ                  UINT8_C(0x0C)
#define BMP3_ODR_0_02_HZ                  UINT8_C(0x0D)
#define BMP3_ODR_0_01_HZ                  UINT8_C(0x0E)
#define BMP3_ODR_0_006_HZ                 UINT8_C(0x0F)
#define BMP3_ODR_0_003_HZ                 UINT8_C(0x10)
#define BMP3_ODR_0_001_HZ                 UINT8_C(0x11)

/**\name API success code */
#define BMP3_OK                           INT8_C(0)

/**\name API error codes */
#define BMP3_E_NULL_PTR                   INT8_C(-1)
#define BMP3_E_DEV_NOT_FOUND              INT8_C(-2)
#define BMP3_E_INVALID_ODR_OSR_SETTINGS   INT8_C(-3)
#define BMP3_E_CMD_EXEC_FAILED            INT8_C(-4)
#define BMP3_E_CONFIGURATION_ERR          INT8_C(-5)
#define BMP3_E_INVALID_LEN                INT8_C(-6)
#define BMP3_E_COMM_FAIL                  INT8_C(-7)
#define BMP3_E_FIFO_WATERMARK_NOT_REACHED INT8_C(-8)

/**\name API warning codes */
#define BMP3_W_SENSOR_NOT_ENABLED         UINT8_C(1)
#define BMP3_W_INVALID_FIFO_REQ_FRAME_CNT UINT8_C(2)

/**\name Macros to select the which sensor settings are to be set by the user.
 * These values are internal for API implementation. Don't relate this to
 * data sheet. */
#define BMP3_PRESS_EN_SEL                 UINT16_C(1 << 1)
#define BMP3_TEMP_EN_SEL                  UINT16_C(1 << 2)
#define BMP3_DRDY_EN_SEL                  UINT16_C(1 << 3)
#define BMP3_PRESS_OS_SEL                 UINT16_C(1 << 4)
#define BMP3_TEMP_OS_SEL                  UINT16_C(1 << 5)
#define BMP3_IIR_FILTER_SEL               UINT16_C(1 << 6)
#define BMP3_ODR_SEL                      UINT16_C(1 << 7)
#define BMP3_OUTPUT_MODE_SEL              UINT16_C(1 << 8)
#define BMP3_LEVEL_SEL                    UINT16_C(1 << 9)
#define BMP3_LATCH_SEL                    UINT16_C(1 << 10)
#define BMP3_I2C_WDT_EN_SEL               UINT16_C(1 << 11)
#define BMP3_I2C_WDT_SEL_SEL              UINT16_C(1 << 12)
#define BMP3_ALL_SETTINGS                 UINT16_C(0x7FF)

/**\name Macros to select the which FIFO settings are to be set by the user
 * These values are internal for API implementation. Don't relate this to
 * data sheet.*/
#define BMP3_FIFO_MODE_SEL                UINT16_C(1 << 1)
#define BMP3_FIFO_STOP_ON_FULL_EN_SEL     UINT16_C(1 << 2)
#define BMP3_FIFO_TIME_EN_SEL             UINT16_C(1 << 3)
#define BMP3_FIFO_PRESS_EN_SEL            UINT16_C(1 << 4)
#define BMP3_FIFO_TEMP_EN_SEL             UINT16_C(1 << 5)
#define BMP3_FIFO_DOWN_SAMPLING_SEL       UINT16_C(1 << 6)
#define BMP3_FIFO_FILTER_EN_SEL           UINT16_C(1 << 7)
#define BMP3_FIFO_FWTM_EN_SEL             UINT16_C(1 << 8)
#define BMP3_FIFO_FULL_EN_SEL             UINT16_C(1 << 9)
#define BMP3_FIFO_ALL_SETTINGS            UINT16_C(0x3FF)

/**\name Sensor component selection macros
 * These values are internal for API implementation. Don't relate this to
 * data sheet.*/
#define BMP3_PRESS                        UINT8_C(1)
#define BMP3_TEMP                         UINT8_C(1 << 1)
#define BMP3_ALL                          UINT8_C(0x03)

/**\name Macros for bit masking */
#define BMP3_ERR_FATAL_MSK                UINT8_C(0x01)

#define BMP3_ERR_CMD_MSK                  UINT8_C(0x02)
#define BMP3_ERR_CMD_POS                  UINT8_C(0x01)

#define BMP3_ERR_CONF_MSK                 UINT8_C(0x04)
#define BMP3_ERR_CONF_POS                 UINT8_C(0x02)

#define BMP3_STATUS_CMD_RDY_MSK           UINT8_C(0x10)
#define BMP3_STATUS_CMD_RDY_POS           UINT8_C(0x04)

#define BMP3_STATUS_DRDY_PRESS_MSK        UINT8_C(0x20)
#define BMP3_STATUS_DRDY_PRESS_POS        UINT8_C(0x05)

#define BMP3_STATUS_DRDY_TEMP_MSK         UINT8_C(0x40)
#define BMP3_STATUS_DRDY_TEMP_POS         UINT8_C(0x06)

#define BMP3_OP_MODE_MSK                  UINT8_C(0x30)
#define BMP3_OP_MODE_POS                  UINT8_C(0x04)

#define BMP3_PRESS_EN_MSK                 UINT8_C(0x01)

#define BMP3_TEMP_EN_MSK                  UINT8_C(0x02)
#define BMP3_TEMP_EN_POS                  UINT8_C(0x01)

#define BMP3_IIR_FILTER_MSK               UINT8_C(0x0E)
#define BMP3_IIR_FILTER_POS               UINT8_C(0x01)

#define BMP3_ODR_MSK                      UINT8_C(0x1F)

#define BMP3_PRESS_OS_MSK                 UINT8_C(0x07)

#define BMP3_TEMP_OS_MSK                  UINT8_C(0x38)
#define BMP3_TEMP_OS_POS                  UINT8_C(0x03)

#define BMP3_FIFO_MODE_MSK                UINT8_C(0x01)

#define BMP3_FIFO_STOP_ON_FULL_MSK        UINT8_C(0x02)
#define BMP3_FIFO_STOP_ON_FULL_POS        UINT8_C(0x01)

#define BMP3_FIFO_TIME_EN_MSK             UINT8_C(0x04)
#define BMP3_FIFO_TIME_EN_POS             UINT8_C(0x02)

#define BMP3_FIFO_PRESS_EN_MSK            UINT8_C(0x08)
#define BMP3_FIFO_PRESS_EN_POS            UINT8_C(0x03)

#define BMP3_FIFO_TEMP_EN_MSK             UINT8_C(0x10)
#define BMP3_FIFO_TEMP_EN_POS             UINT8_C(0x04)

#define BMP3_FIFO_FILTER_EN_MSK           UINT8_C(0x18)
#define BMP3_FIFO_FILTER_EN_POS           UINT8_C(0x03)

#define BMP3_FIFO_DOWN_SAMPLING_MSK       UINT8_C(0x07)

#define BMP3_FIFO_FWTM_EN_MSK             UINT8_C(0x08)
#define BMP3_FIFO_FWTM_EN_POS             UINT8_C(0x03)

#define BMP3_FIFO_FULL_EN_MSK             UINT8_C(0x10)
#define BMP3_FIFO_FULL_EN_POS             UINT8_C(0x04)

#define BMP3_INT_OUTPUT_MODE_MSK          UINT8_C(0x01)

#define BMP3_INT_LEVEL_MSK                UINT8_C(0x02)
#define BMP3_INT_LEVEL_POS                UINT8_C(0x01)

#define BMP3_INT_LATCH_MSK                UINT8_C(0x04)
#define BMP3_INT_LATCH_POS                UINT8_C(0x02)

#define BMP3_INT_DRDY_EN_MSK              UINT8_C(0x40)
#define BMP3_INT_DRDY_EN_POS              UINT8_C(0x06)

#define BMP3_I2C_WDT_EN_MSK               UINT8_C(0x02)
#define BMP3_I2C_WDT_EN_POS               UINT8_C(0x01)

#define BMP3_I2C_WDT_SEL_MSK              UINT8_C(0x04)
#define BMP3_I2C_WDT_SEL_POS              UINT8_C(0x02)

#define BMP3_INT_STATUS_FWTM_MSK          UINT8_C(0x01)

#define BMP3_INT_STATUS_FFULL_MSK         UINT8_C(0x02)
#define BMP3_INT_STATUS_FFULL_POS         UINT8_C(0x01)

#define BMP3_INT_STATUS_DRDY_MSK          UINT8_C(0x08)
#define BMP3_INT_STATUS_DRDY_POS          UINT8_C(0x03)

/**\name    UTILITY MACROS  */
#define BMP3_SET_LOW_BYTE                 UINT16_C(0x00FF)
#define BMP3_SET_HIGH_BYTE                UINT16_C(0xFF00)

/**\name Macro to combine two 8 bit data's to form a 16 bit data */
#define BMP3_CONCAT_BYTES(msb, lsb) (((uint16_t)msb << 8) | (uint16_t)lsb)

#define BMP3_SET_BITS(reg_data, bitname, data) \
    ((reg_data & ~(bitname##_MSK)) | \
     ((data << bitname##_POS) & bitname##_MSK))

/* Macro variant to handle the bitname position if it is zero */
#define BMP3_SET_BITS_POS_0(reg_data, bitname, data) \
    ((reg_data & ~(bitname##_MSK)) | \
     (data & bitname##_MSK))

#define BMP3_GET_BITS(reg_data, bitname)       ((reg_data & (bitname##_MSK)) >> \
                                                (bitname##_POS))

/* Macro variant to handle the bitname position if it is zero */
#define BMP3_GET_BITS_POS_0(reg_data, bitname) (reg_data & (bitname##_MSK))

#define BMP3_GET_LSB(var)                      (uint8_t)(var & BMP3_SET_LOW_BYTE)
#define BMP3_GET_MSB(var)                      (uint8_t)((var & BMP3_SET_HIGH_BYTE) >> 8)

/**\name Macros related to size */
#define BMP3_CALIB_DATA_LEN          UINT8_C(21)
#define BMP3_P_AND_T_HEADER_DATA_LEN UINT8_C(7)
#define BMP3_P_OR_T_HEADER_DATA_LEN  UINT8_C(4)
#define BMP3_P_T_DATA_LEN            UINT8_C(6)
#define BMP3_GEN_SETT_LEN            UINT8_C(7)
#define BMP3_P_DATA_LEN              UINT8_C(3)
#define BMP3_T_DATA_LEN              UINT8_C(3)
#define BMP3_SENSOR_TIME_LEN         UINT8_C(3)
#define BMP3_FIFO_MAX_FRAMES         UINT8_C(73)

/********************************************************/

/*!
 * @brief Interface selection Enums
 */
enum bmp3_intf {
    /*! SPI interface */
    BMP3_SPI_INTF,

    /*! I2C interface */
    BMP3_I2C_INTF
};

/********************************************************/
struct bmp3_dev;
/*!
 * @brief Type definitions
 */
typedef int8_t (*bmp3_com_fptr_t)(const struct bmp3_dev* dev, uint8_t reg_addr, uint8_t *data, uint16_t len);
typedef void (*bmp3_delay_fptr_t)(uint32_t period);

/********************************************************/

/*!
 * @brief Register Trim Variables
 */
struct bmp3_reg_calib_data
{
    /**
     * @ Trim Variables
     */

    /**@{*/
    uint16_t par_t1;
    uint16_t par_t2;
    int8_t par_t3;
    int16_t par_p1;
    int16_t par_p2;
    int8_t par_p3;
    int8_t par_p4;
    uint16_t par_p5;
    uint16_t par_p6;
    int8_t par_p7;
    int8_t par_p8;
    int16_t par_p9;
    int8_t par_p10;
    int8_t par_p11;
    int64_t t_lin;

    /**@}*/
};

/*!
 * @brief bmp3 advance settings
 */
struct bmp3_adv_settings
{
    /*! i2c watch dog enable */
    uint8_t i2c_wdt_en;

    /*! i2c watch dog select */
    uint8_t i2c_wdt_sel;
};

/*!
 * @brief bmp3 odr and filter settings
 */
struct bmp3_odr_filter_settings
{
    /*! Pressure oversampling */
    uint8_t press_os;

    /*! Temperature oversampling */
    uint8_t temp_os;

    /*! IIR filter */
    uint8_t iir_filter;

    /*! Output data rate */
    uint8_t odr;
};

/*!
 * @brief bmp3 sensor status flags
 */
struct bmp3_sens_status
{
    /*! Command ready status */
    uint8_t cmd_rdy;

    /*! Data ready for pressure */
    uint8_t drdy_press;

    /*! Data ready for temperature */
    uint8_t drdy_temp;
};

/*!
 * @brief bmp3 interrupt status flags
 */
struct bmp3_int_status
{
    /*! fifo watermark interrupt */
    uint8_t fifo_wm;

    /*! fifo full interrupt */
    uint8_t fifo_full;

    /*! data ready interrupt */
    uint8_t drdy;
};

/*!
 * @brief bmp3 error status flags
 */
struct bmp3_err_status
{
    /*! fatal error */
    uint8_t fatal;

    /*! command error */
    uint8_t cmd;

    /*! configuration error */
    uint8_t conf;
};

/*!
 * @brief bmp3 status flags
 */
struct bmp3_status
{
    /*! Interrupt status */
    struct bmp3_int_status intr;

    /*! Sensor status */
    struct bmp3_sens_status sensor;

    /*! Error status */
    struct bmp3_err_status err;

    /*! power on reset status */
    uint8_t pwr_on_rst;
};

/*!
 * @brief bmp3 interrupt pin settings
 */
struct bmp3_int_ctrl_settings
{
    /*! Output mode */
    uint8_t output_mode;

    /*! Active high/low */
    uint8_t level;

    /*! Latched or Non-latched */
    uint8_t latch;

    /*! Data ready interrupt */
    uint8_t drdy_en;
};

/*!
 * @brief bmp3 device settings
 */
struct bmp3_settings
{
    /*! Power mode which user wants to set */
    uint8_t op_mode;

    /*! Enable/Disable pressure sensor */
    uint8_t press_en;

    /*! Enable/Disable temperature sensor */
    uint8_t temp_en;

    /*! ODR and filter configuration */
    struct bmp3_odr_filter_settings odr_filter;

    /*! Interrupt configuration */
    struct bmp3_int_ctrl_settings int_settings;

    /*! Advance settings */
    struct bmp3_adv_settings adv_settings;
};

/*!
 * @brief bmp3 fifo frame
 */
struct bmp3_fifo_data
{
    /*! Data buffer of user defined length is to be mapped here
     * 512 + 4 */
    uint8_t buffer[516];

    /*! Number of bytes of data read from the fifo */
    uint16_t byte_count;

    /*! Number of frames to be read as specified by the user */
    uint8_t req_frames;

    /*! Will be equal to length when no more frames are there to parse */
    uint16_t start_idx;

    /*! Will contain the no of parsed data frames from fifo */
    uint8_t parsed_frames;

    /*! Configuration error */
    uint8_t config_err;

    /*! Sensor time */
    uint32_t sensor_time;

    /*! FIFO input configuration change */
    uint8_t config_change;

    /*! All available frames are parsed */
    uint8_t frame_not_available;
};

/*!
 * @brief bmp3 fifo configuration
 */
struct bmp3_fifo_settings
{
    /*! enable/disable */
    uint8_t mode;

    /*! stop on full enable/disable */
    uint8_t stop_on_full_en;

    /*! time enable/disable */
    uint8_t time_en;

    /*! pressure enable/disable */
    uint8_t press_en;

    /*! temperature enable/disable */
    uint8_t temp_en;

    /*! down sampling rate */
    uint8_t down_sampling;

    /*! filter enable/disable */
    uint8_t filter_en;

    /*! FIFO watermark enable/disable */
    uint8_t fwtm_en;

    /*! FIFO full enable/disable */
    uint8_t ffull_en;
};

/*!
 * @brief bmp3 bmp3 FIFO
 */
struct bmp3_fifo
{
    /*! FIFO frame structure */
    struct bmp3_fifo_data data;

    /*! FIFO config structure */
    struct bmp3_fifo_settings settings;
};

#ifdef BMP3_DOUBLE_PRECISION_COMPENSATION

/*!
 * @brief Quantized Trim Variables
 */
struct bmp3_quantized_calib_data
{
    /**
     * @ Quantized Trim Variables
     */

    /**@{*/
    double par_t1;
    double par_t2;
    double par_t3;
    double par_p1;
    double par_p2;
    double par_p3;
    double par_p4;
    double par_p5;
    double par_p6;
    double par_p7;
    double par_p8;
    double par_p9;
    double par_p10;
    double par_p11;
    double t_lin;

    /**@}*/
};

/*!
 * @brief Calibration data
 */
struct bmp3_calib_data
{
    /*! Quantized data */
    struct bmp3_quantized_calib_data quantized_calib_data;

    /*! Register data */
    struct bmp3_reg_calib_data reg_calib_data;
};

/*!
 * @brief bmp3 sensor structure which comprises of temperature and pressure
 * data.
 */
struct bmp3_data
{
    /*! Compensated temperature */
    double temperature;

    /*! Compensated pressure */
    double pressure;
};

#else

/*!
 * @brief bmp3 sensor structure which comprises of temperature and pressure
 * data.
 */
struct bmp3_data
{
    /*! Compensated temperature */
    int64_t temperature;

    /*! Compensated pressure */
    uint64_t pressure;
};

/*!
 * @brief Calibration data
 */
struct bmp3_calib_data
{
    /*! Register data */
    struct bmp3_reg_calib_data reg_calib_data;
};

#endif /* BMP3_DOUBLE_PRECISION_COMPENSATION */

/*!
 * @brief bmp3 sensor structure which comprises of un-compensated temperature
 * and pressure data.
 */
struct bmp3_uncomp_data
{
    /*! un-compensated pressure */
    uint32_t pressure;

    /*! un-compensated temperature */
    uint32_t temperature;
};

/*!
 * @brief bmp3 device structure
 */
struct bmp3_dev {
    /*! Chip Id */
    uint8_t chip_id;

    /*! Device Id (I2C slave address) */
    uint8_t dev_id;

    /*! SPI/I2C interface */
    enum bmp3_intf intf;

    /*! Decide SPI or I2C read mechanism */
    uint8_t dummy_byte;

    /*! Read function pointer */
    bmp3_com_fptr_t read;

    /*! Write function pointer */
    bmp3_com_fptr_t write;

    /*! Delay function pointer */
    bmp3_delay_fptr_t delay_ms;

    /*! Trim data */
    struct bmp3_calib_data calib_data;

    /*! Sensor Settings */
    struct bmp3_settings settings;

    /*! Sensor and interrupt status flags */
    struct bmp3_status status;

    /*! FIFO data and settings structure */
    struct bmp3_fifo *fifo;

    struct bmp3_dev *flink;          /* Supports a singly linked list of drivers */
    int freq;                        /* BMP3 Frequency    */
};

/*!
 *  @brief This API is the entry point.
 *  It performs the selection of I2C/SPI read mechanism according to the
 *  selected interface and reads the chip-id and calibration data of the sensor.
 *
 *  @param[in,out] dev : Structure instance of bmp3_dev
 *
 *  @return Result of API execution status
 *  @retval zero -> Success / +ve value -> Warning / -ve value -> Error
 */
int8_t bmp3_init(struct bmp3_dev *dev);

/*!
 * @brief This API performs the soft reset of the sensor.
 *
 * @param[in] dev : Structure instance of bmp3_dev.
 *
 * @return Result of API execution status
 * @retval zero -> Success / +ve value -> Warning / -ve value -> Error.
 */
int8_t bmp3_soft_reset(const struct bmp3_dev *dev);

/*!
 * @brief This API sets the power control(pressure enable and
 * temperature enable), over sampling, odr and filter
 * settings in the sensor.
 *
 * @param[in] dev : Structure instance of bmp3_dev.
 * @param[in] desired_settings : Variable used to select the settings which
 * are to be set in the sensor.
 *
 * @note : Below are the macros to be used by the user for selecting the
 * desired settings. User can do OR operation of these macros for configuring
 * multiple settings.
 *
 * Macros         |   Functionality
 * -----------------------|----------------------------------------------
 * BMP3_PRESS_EN_SEL    |   Enable/Disable pressure.
 * BMP3_TEMP_EN_SEL     |   Enable/Disable temperature.
 * BMP3_PRESS_OS_SEL    |   Set pressure oversampling.
 * BMP3_TEMP_OS_SEL     |   Set temperature oversampling.
 * BMP3_IIR_FILTER_SEL  |   Set IIR filter.
 * BMP3_ODR_SEL         |   Set ODR.
 * BMP3_OUTPUT_MODE_SEL |   Set either open drain or push pull
 * BMP3_LEVEL_SEL       |   Set interrupt pad to be active high or low
 * BMP3_LATCH_SEL       |   Set interrupt pad to be latched or nonlatched.
 * BMP3_DRDY_EN_SEL     |   Map/Unmap the drdy interrupt to interrupt pad.
 * BMP3_I2C_WDT_EN_SEL  |   Enable/Disable I2C internal watch dog.
 * BMP3_I2C_WDT_SEL_SEL |   Set I2C watch dog timeout delay.
 *
 * @return Result of API execution status
 * @retval zero -> Success / +ve value -> Warning / -ve value -> Error.
 */
int8_t bmp3_set_sensor_settings(uint32_t desired_settings, struct bmp3_dev *dev);

/*!
 * @brief This API gets the power control(power mode, pressure enable and
 * temperature enable), over sampling, odr, filter, interrupt control and
 * advance settings from the sensor.
 *
 * @param[in,out] dev : Structure instance of bmp3_dev.
 *
 * @return Result of API execution status
 * @retval zero -> Success / +ve value -> Warning / -ve value -> Error.
 */
int8_t bmp3_get_sensor_settings(struct bmp3_dev *dev);

/*!
 * @brief This API sets the power mode of the sensor.
 *
 * @param[in] dev : Structure instance of bmp3_dev.
 *
 * dev->settings.op_mode |   Macros
 * ----------------------|-------------------
 *     0                 | BMP3_SLEEP_MODE
 *     1                 | BMP3_FORCED_MODE
 *     3                 | BMP3_NORMAL_MODE
 *
 *
 * @note : Before setting normal mode, valid odr and osr settings should be set
 * in the sensor by using 'bmp3_set_sensor_settings' function.
 *
 * @return Result of API execution status
 * @retval zero -> Success / +ve value -> Warning / -ve value -> Error
 */
int8_t bmp3_set_op_mode(struct bmp3_dev *dev);

/*!
 * @brief This API gets the power mode of the sensor.
 *
 * @param[in] dev : Structure instance of bmp3_dev.
 * @param[out] op_mode : Pointer variable to store the op-mode.
 *
 *   op_mode             |   Macros
 * ----------------------|-------------------
 *     0                 | BMP3_SLEEP_MODE
 *     1                 | BMP3_FORCED_MODE
 *     3                 | BMP3_NORMAL_MODE
 *
 * @return Result of API execution status
 * @retval zero -> Success / +ve value -> Warning / -ve value -> Error
 */
int8_t bmp3_get_op_mode(uint8_t *op_mode, const struct bmp3_dev *dev);

/*!
 * @brief This API reads the pressure, temperature or both data from the
 * sensor, compensates the data and store it in the bmp3_data structure
 * instance passed by the user.
 *
 * @param[in] sensor_comp : Variable which selects which data to be read from
 * the sensor.
 *
 * sensor_comp |   Macros
 * ------------|-------------------
 *     1       | BMP3_PRESS
 *     2       | BMP3_TEMP
 *     3       | BMP3_ALL
 *
 * @param[out] data : Structure instance of bmp3_data.
 * @param[in] dev : Structure instance of bmp3_dev.
 *
 * @note : for fixed point the compensated temperature and pressure has a multiplication factor of 100.
 *          units are degree celsius and Pascal respectively.
 *          ie if temp is 2426 then temp is 24.26 deg C
 *          if press is 9528709 it is 95287.09 Pascal.
 *
 * @return Result of API execution status
 * @retval zero -> Success / +ve value -> Warning / -ve value -> Error
 */
int8_t bmp3_get_sensor_data(uint8_t sensor_comp, struct bmp3_data *data, struct bmp3_dev *dev);

/*!
 * @brief This API writes the given data to the register address
 * of the sensor.
 *
 * @param[in] reg_addr : Register address from where the data to be written.
 * @param[in] reg_data : Pointer to data buffer which is to be written
 * in the sensor.
 * @param[in] len : No of bytes of data to write..
 * @param[in] dev : Structure instance of bmp3_dev.
 *
 * @return Result of API execution status
 * @retval zero -> Success / +ve value -> Warning / -ve value -> Error
 */
int8_t bmp3_set_regs(uint8_t *reg_addr, const uint8_t *reg_data, uint8_t len, const struct bmp3_dev *dev);

/*!
 * @brief This API reads the data from the given register address of the sensor.
 *
 * @param[in] reg_addr : Register address from where the data to be read
 * @param[out] reg_data : Pointer to data buffer to store the read data.
 * @param[in] length : No of bytes of data to be read.
 * @param[in] dev : Structure instance of bmp3_dev.
 *
 * @return Result of API execution status
 * @retval zero -> Success / +ve value -> Warning / -ve value -> Error
 */
int8_t bmp3_get_regs(uint8_t reg_addr, uint8_t *reg_data, uint16_t length, const struct bmp3_dev *dev);

/*!
 * @brief This API sets the fifo_config_1(fifo_mode,
 * fifo_stop_on_full, fifo_time_en, fifo_press_en, fifo_temp_en),
 * fifo_config_2(fifo_subsampling, data_select) and int_ctrl(fwtm_en, ffull_en)
 * settings in the sensor.
 *
 * @param[in] dev : Structure instance of bmp3_dev.
 * @param[in] desired_settings : Variable used to select the FIFO settings which
 * are to be set in the sensor.
 *
 * @note : Below are the macros to be used by the user for selecting the
 * desired settings. User can do OR operation of these macros for configuring
 * multiple settings.
 *
 * Macros                          |  Functionality
 * --------------------------------|----------------------------
 * BMP3_FIFO_MODE_SEL            |  Enable/Disable FIFO
 * BMP3_FIFO_STOP_ON_FULL_EN_SEL |  Set FIFO stop on full interrupt
 * BMP3_FIFO_TIME_EN_SEL         |  Enable/Disable FIFO time
 * BMP3_FIFO_PRESS_EN_SEL        |  Enable/Disable pressure
 * BMP3_FIFO_TEMP_EN_SEL         |  Enable/Disable temperature
 * BMP3_FIFO_DOWN_SAMPLING_SEL   |  Set FIFO downsampling
 * BMP3_FIFO_FILTER_EN_SEL       |  Enable/Disable FIFO filter
 * BMP3_FIFO_FWTM_EN_SEL         |  Enable/Disable FIFO watermark interrupt
 * BMP3_FIFO_FFULL_EN_SEL        |  Enable/Disable FIFO full interrupt
 *
 * @return Result of API execution status
 * @retval zero -> Success / +ve value -> Warning / -ve value -> Error.
 */
int8_t bmp3_set_fifo_settings(uint16_t desired_settings, const struct bmp3_dev *dev);

/*!
 * @brief This API gets the fifo_config_1(fifo_mode,
 * fifo_stop_on_full, fifo_time_en, fifo_press_en, fifo_temp_en),
 * fifo_config_2(fifo_subsampling, data_select) and int_ctrl(fwtm_en, ffull_en)
 * settings from the sensor.
 *
 * @param[in,out] dev : Structure instance of bmp3_dev.
 *
 * @return Result of API execution status
 * @retval zero -> Success / +ve value -> Warning / -ve value -> Error.
 */
int8_t bmp3_get_fifo_settings(const struct bmp3_dev *dev);

/*!
 * @brief This API gets the fifo data from the sensor.
 *
 * @param[in,out] dev : Structure instance of bmp3 device, where the fifo
 * data will be stored in fifo buffer.
 *
 * @return Result of API execution status.
 * @retval zero -> Success / +ve value -> Warning / -ve value -> Error.
 */
int8_t bmp3_get_fifo_data(const struct bmp3_dev *dev);

/*!
 * @brief This API gets the fifo length from the sensor.
 *
 * @param[out] fifo_length : Variable used to store the fifo length.
 * @param[in] dev : Structure instance of bmp3_dev.
 *
 * @return Result of API execution status.
 * @retval zero -> Success / +ve value -> Warning / -ve value -> Error.
 */
int8_t bmp3_get_fifo_length(uint16_t *fifo_length, const struct bmp3_dev *dev);

/*!
 * @brief This API extracts the temperature and/or pressure data from the FIFO
 * data which is already read from the fifo.
 *
 * @param[out] data : Array of bmp3_data structures where the temperature
 * and pressure frames will be stored.
 * @param[in,out] dev : Structure instance of bmp3_dev which contains the
 * fifo buffer to parse the temperature and pressure frames.
 *
 * @return Result of API execution status.
 * @retval zero -> Success / -ve value -> Error.
 */
int8_t bmp3_extract_fifo_data(struct bmp3_data *data, struct bmp3_dev *dev);

/*!
 * @brief This API gets the command ready, data ready for pressure and
 * temperature and interrupt (fifo watermark, fifo full, data ready) and
 * error status from the sensor.
 *
 * @param[in,out] dev : Structure instance of bmp3_dev
 *
 * @return Result of API execution status.
 * @retval zero -> Success / -ve value -> Error.
 */
int8_t bmp3_get_status(struct bmp3_dev *dev);

/*!
 * @brief This API sets the fifo watermark length according to the frames count
 * set by the user in the device structure. Refer below for usage.
 *
 * @note: dev->fifo->data.req_frames = 50;
 *
 * @param[in] dev : Structure instance of bmp3_dev
 *
 * @return Result of API execution status.
 * @retval zero -> Success / -ve value -> Error.
 */
int8_t bmp3_set_fifo_watermark(const struct bmp3_dev *dev);

#if defined(CONFIG_I2C) && defined(CONFIG_SENSORS_BMP180)
  
/*!
 * @brief register the driver with NuttX kernel.
 *
 * @note: dev->fifo->data.req_frames = 50;
 *
 * @param[in] devpath : device path in /dev
 * @param[in] dev : Structure instance of i2c_master_s
 *
 * @return Result of API execution status.
 * @retval zero -> Success / -ve value -> Error.
 * 
 */
int bmp3_register(FAR const char *devpath, FAR struct i2c_master_s *i2c);
#endif /* !defined(CONFIG_I2C) && defined(CONFIG_SENSORS_BMP180) */
  
#ifdef __cplusplus
}
#endif /* End of CPP guard */

#endif /* BMP3_H_ */
/** @}*/

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
* @file       bmp3_selftest.c
* @date       2020-07-20
* @version    v2.0.1
*
*/

#include "bmp3_selftest.h"

#ifndef BMP3_DOUBLE_PRECISION_COMPENSATION

/* 0 degree celsius */
#define BMP3_MIN_TEMPERATURE  INT16_C(0)

/* 40 degree celsius */
#define BMP3_MAX_TEMPERATURE  INT16_C(4000)

/* 900 hecto Pascals */
#define BMP3_MIN_PRESSURE     UINT32_C(90000)

/* 1100 hecto Pascals */
#define BMP3_MAX_PRESSURE     UINT32_C(110000)

#else

/* 0 degree celsius */
#define BMP3_MIN_TEMPERATURE  (0.0f)

/* 40 degree celsius */
#define BMP3_MAX_TEMPERATURE  (40.0f)

/* 900 hecto Pascals */
#define BMP3_MIN_PRESSURE     (900.0f)

/* 1100 hecto Pascals */
#define BMP3_MAX_PRESSURE     (1100.0f)
#endif

/*!
 * @brief       Function to analyze the sensor data
 *
 * @param[in]   data   Structure instance of bmp3_data(compensated temp & press values)
 *
 * @return      Error code
 * @retval      0   Success
 */
static int8_t analyze_sensor_data(const struct bmp3_data *data);

/*!
 * @brief       Function to calculate the CRC of the trimming parameters.
 *
 * @param[in]   seed   CRC of each register
 * @param[in]   data   register data.
 *
 * @return      calculated CRC
 */
static int8_t cal_crc(uint8_t seed, uint8_t data);

/*!
 * @brief Function to validate the trimming parameters
 *
 * @param [in] dev Structure instance of bmp3_dev structure.
 *
 * @return      Error code
 * @retval      0   Success
 *
 */
static int8_t validate_trimming_param(struct bmp3_dev *dev);

/*!
 * @brief       Self-test API for the BMP38X
 */
int8_t bmp3_selftest_check(struct bmp3_dev *dev)
{
    int8_t rslt;

    /* Variable used to select the sensor component */
    uint8_t sensor_comp;

    /* Variable used to store the compensated data */
    struct bmp3_data data = { 0 };

    /* Used to select the settings user needs to change */
    uint16_t settings_sel;

    /* Reset the sensor */
    rslt = bmp3_soft_reset(dev);
    if (rslt == BMP3_SENSOR_OK)
    {
        rslt = bmp3_init(dev);

        if (rslt == BMP3_E_COMM_FAIL || rslt == BMP3_E_DEV_NOT_FOUND)
        {
            rslt = BMP3_COMMUNICATION_ERROR_OR_WRONG_DEVICE;
        }

        if (rslt == BMP3_SENSOR_OK)
        {
            rslt = validate_trimming_param(dev);
        }

        if (rslt == BMP3_SENSOR_OK)
        {
            /* Select the pressure and temperature sensor to be enabled */
            dev->settings.press_en = BMP3_ENABLE;
            dev->settings.temp_en = BMP3_ENABLE;

            /* Select the output data rate and over sampling settings for pressure and temperature */
            dev->settings.odr_filter.press_os = BMP3_NO_OVERSAMPLING;
            dev->settings.odr_filter.temp_os = BMP3_NO_OVERSAMPLING;
            dev->settings.odr_filter.odr = BMP3_ODR_25_HZ;

            /* Assign the settings which needs to be set in the sensor */
            settings_sel = BMP3_SEL_PRESS_EN | BMP3_SEL_TEMP_EN | BMP3_SEL_PRESS_OS | BMP3_SEL_TEMP_OS | BMP3_SEL_ODR;
            rslt = bmp3_set_sensor_settings(settings_sel, dev);
            if (rslt == BMP3_SENSOR_OK)
            {
                dev->settings.op_mode = BMP3_MODE_NORMAL;
                rslt = bmp3_set_op_mode(dev);
                if (rslt == BMP3_SENSOR_OK)
                {
                    dev->delay_us(40000, dev->intf_ptr);

                    /* Sensor component selection */
                    sensor_comp = BMP3_PRESS | BMP3_TEMP;

                    /* Temperature and Pressure data are read and stored in the bmp3_data instance */
                    rslt = bmp3_get_sensor_data(sensor_comp, &data, dev);
                }
            }
        }

        if (rslt == BMP3_SENSOR_OK)
        {
            rslt = analyze_sensor_data(&data);

            /* Set the power mode to sleep mode */
            if (rslt == BMP3_SENSOR_OK)
            {
                dev->settings.op_mode = BMP3_MODE_SLEEP;
                rslt = bmp3_set_op_mode(dev);
            }
        }
    }

    return rslt;
}

/*!
 * @brief  Function to analyze the sensor data
 */
static int8_t analyze_sensor_data(const struct bmp3_data *sens_data)
{
    int8_t rslt = BMP3_SENSOR_OK;

    if ((sens_data->temperature < BMP3_MIN_TEMPERATURE) || (sens_data->temperature > BMP3_MAX_TEMPERATURE))
    {
        rslt = BMP3_IMPLAUSIBLE_TEMPERATURE;
    }

    if (rslt == BMP3_SENSOR_OK)
    {
        if ((sens_data->pressure / 100 < BMP3_MIN_PRESSURE) || (sens_data->pressure / 100 > BMP3_MAX_PRESSURE))
        {
            rslt = BMP3_IMPLAUSIBLE_PRESSURE;
        }
    }

    return rslt;
}

/*
 * @brief Function to verify the trimming parameters
 * */
static int8_t validate_trimming_param(struct bmp3_dev *dev)
{
    int8_t rslt;
    uint8_t crc = 0xFF;
    uint8_t stored_crc;
    uint8_t trim_param[21];
    uint8_t i;

    rslt = bmp3_get_regs(BMP3_REG_CALIB_DATA, trim_param, 21, dev);
    if (rslt == BMP3_SENSOR_OK)
    {
        for (i = 0; i < 21; i++)
        {
            crc = (uint8_t)cal_crc(crc, trim_param[i]);
        }

        crc = (crc ^ 0xFF);
        rslt = bmp3_get_regs(0x30, &stored_crc, 1, dev);
        if (stored_crc != crc)
        {
            rslt = BMP3_TRIMMING_DATA_OUT_OF_BOUND;
        }
    }

    return rslt;

}

/*
 * @brief function to calculate CRC for the trimming parameters
 * */
static int8_t cal_crc(uint8_t seed, uint8_t data)
{
    int8_t poly = 0x1D;
    int8_t var2;
    uint8_t i;

    for (i = 0; i < 8; i++)
    {
        if ((seed & 0x80) ^ (data & 0x80))
        {
            var2 = 1;
        }
        else
        {
            var2 = 0;
        }

        seed = (seed & 0x7F) << 1;
        data = (data & 0x7F) << 1;
        seed = seed ^ (uint8_t)(poly * var2);
    }

    return (int8_t)seed;
}

/** @}*/

/**\mainpage
 * Copyright (C) 2018 - 2019 Bosch Sensortec GmbH
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution.
 *
 * Neither the name of the copyright holder nor the names of the
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 * CONTRIBUTORS "AS IS" AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDER
 * OR CONTRIBUTORS BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY,
 * OR CONSEQUENTIAL DAMAGES(INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE
 *
 * The information provided is believed to be accurate and reliable.
 * The copyright holder assumes no responsibility
 * for the consequences of use
 * of such information nor for any infringement of patents or
 * other rights of third parties which may result from its use.
 * No license is granted by implication or otherwise under any patent or
 * patent rights of the copyright holder.
 *
 * @File     bmp38x_selftest.c
 * @date    01 July 2019
 * @version 1.1.3
 *
 */

#include "bmp3_selftest.h"

/* 0 degree celsius */
#define BMP3_MIN_TEMPERATURE INT16_C(0)

/* 40 degree celsius */
#define BMP3_MAX_TEMPERATURE INT16_C(4000)

/* 900 hecto Pascals */
#define BMP3_MIN_PRESSURE    UINT32_C(90000)

/* 1100 hecto Pascals */
#define BMP3_MAX_PRESSURE    UINT32_C(110000)

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
static int8_t validate_trimming_param(const struct bmp3_dev *dev);

/*!
 * @brief       Self-test API for the BMP38X
 */
int8_t bmp38x_self_test(const struct bmp3_dev *dev)
{
    int8_t rslt;

    /* Variable used to select the sensor component */
    uint8_t sensor_comp;

    /* Variable used to store the compensated data */
    struct bmp3_data data = { 0 };

    /* Used to select the settings user needs to change */
    uint16_t settings_sel;
    struct bmp3_dev t_dev;

    t_dev.dev_id = dev->dev_id;
    t_dev.read = dev->read;
    t_dev.write = dev->write;
    t_dev.intf = dev->intf;
    t_dev.delay_ms = dev->delay_ms;

    /* Reset the sensor */
    rslt = bmp3_soft_reset(dev);
    if (rslt == BMP3_SENSOR_OK)
    {
        rslt = bmp3_init(&t_dev);

        if (rslt == BMP3_E_COMM_FAIL || rslt == BMP3_E_DEV_NOT_FOUND)
        {
            rslt = BMP3_COMMUNICATION_ERROR_OR_WRONG_DEVICE;
        }

        if (rslt == BMP3_SENSOR_OK)
        {
            rslt = validate_trimming_param(&t_dev);
        }

        if (rslt == BMP3_SENSOR_OK)
        {
            /* Select the pressure and temperature sensor to be enabled */
            t_dev.settings.press_en = BMP3_ENABLE;
            t_dev.settings.temp_en = BMP3_ENABLE;

            /* Select the output data rate and over sampling settings for pressure and temperature */
            t_dev.settings.odr_filter.press_os = BMP3_NO_OVERSAMPLING;
            t_dev.settings.odr_filter.temp_os = BMP3_NO_OVERSAMPLING;
            t_dev.settings.odr_filter.odr = BMP3_ODR_25_HZ;

            /* Assign the settings which needs to be set in the sensor */
            settings_sel = BMP3_PRESS_EN_SEL | BMP3_TEMP_EN_SEL | BMP3_PRESS_OS_SEL | BMP3_TEMP_OS_SEL | BMP3_ODR_SEL;
            rslt = bmp3_set_sensor_settings(settings_sel, &t_dev);
            if (rslt == BMP3_SENSOR_OK)
            {
                t_dev.settings.op_mode = BMP3_NORMAL_MODE;
                rslt = bmp3_set_op_mode(&t_dev);
                if (rslt == BMP3_SENSOR_OK)
                {
                    t_dev.delay_ms(40);

                    /* Sensor component selection */
                    sensor_comp = BMP3_PRESS | BMP3_TEMP;

                    /* Temperature and Pressure data are read and stored in the bmp3_data instance */
                    rslt = bmp3_get_sensor_data(sensor_comp, &data, &t_dev);
                }
            }

            if (rslt == BMP3_SENSOR_OK)
            {
                rslt = analyze_sensor_data(&data);

                /* Set the power mode to sleep mode */
                if (rslt == BMP3_SENSOR_OK)
                {

                    t_dev.settings.op_mode = BMP3_SLEEP_MODE;
                    rslt = bmp3_set_op_mode(&t_dev);

                }
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
static int8_t validate_trimming_param(const struct bmp3_dev *dev)
{
    int8_t rslt;
    uint8_t crc = 0xFF;
    uint8_t stored_crc;
    uint8_t trim_param[21];
    uint8_t i;

    rslt = bmp3_get_regs(BMP3_CALIB_DATA_ADDR, trim_param, 21, dev);
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

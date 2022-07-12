/**\
 * Copyright (c) 2022 Bosch Sensortec GmbH. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 **/

#include <stdio.h>

#include "bmp3.h"
#include "common.h"

/************************************************************************/
/*********                     Macros                              ******/
/************************************************************************/
/* Iteration count to run example code */
#define ITERATION  UINT8_C(100)

/************************************************************************/
/*********                     Test code                           ******/
/************************************************************************/

int main(void)
{
    int8_t rslt;
    uint8_t loop = 0;
    uint16_t settings_sel;
    struct bmp3_dev dev;
    struct bmp3_data data = { 0 };
    struct bmp3_settings settings = { 0 };
    struct bmp3_status status = { { 0 } };

    /* Interface reference is given as a parameter
     *         For I2C : BMP3_I2C_INTF
     *         For SPI : BMP3_SPI_INTF
     */
    rslt = bmp3_interface_init(&dev, BMP3_I2C_INTF);
    bmp3_check_rslt("bmp3_interface_init", rslt);

    rslt = bmp3_init(&dev);
    bmp3_check_rslt("bmp3_init", rslt);

    settings.int_settings.drdy_en = BMP3_ENABLE;
    settings.press_en = BMP3_ENABLE;
    settings.temp_en = BMP3_ENABLE;

    settings.odr_filter.press_os = BMP3_OVERSAMPLING_2X;
    settings.odr_filter.temp_os = BMP3_OVERSAMPLING_2X;
    settings.odr_filter.odr = BMP3_ODR_100_HZ;

    settings_sel = BMP3_SEL_PRESS_EN | BMP3_SEL_TEMP_EN | BMP3_SEL_PRESS_OS | BMP3_SEL_TEMP_OS | BMP3_SEL_ODR |
                   BMP3_SEL_DRDY_EN;

    rslt = bmp3_set_sensor_settings(settings_sel, &settings, &dev);
    bmp3_check_rslt("bmp3_set_sensor_settings", rslt);

    settings.op_mode = BMP3_MODE_NORMAL;
    rslt = bmp3_set_op_mode(&settings, &dev);
    bmp3_check_rslt("bmp3_set_op_mode", rslt);

    while (loop < ITERATION)
    {
        rslt = bmp3_get_status(&status, &dev);
        bmp3_check_rslt("bmp3_get_status", rslt);

        /* Read temperature and pressure data iteratively based on data ready interrupt */
        if ((rslt == BMP3_OK) && (status.intr.drdy == BMP3_ENABLE))
        {
            /*
             * First parameter indicates the type of data to be read
             * BMP3_PRESS_TEMP : To read pressure and temperature data
             * BMP3_TEMP       : To read only temperature data
             * BMP3_PRESS      : To read only pressure data
             */
            rslt = bmp3_get_sensor_data(BMP3_PRESS_TEMP, &data, &dev);
            bmp3_check_rslt("bmp3_get_sensor_data", rslt);

            /* NOTE : Read status register again to clear data ready interrupt status */
            rslt = bmp3_get_status(&status, &dev);
            bmp3_check_rslt("bmp3_get_status", rslt);

            #ifdef BMP3_FLOAT_COMPENSATION
            printf("Data[%d]  T: %.2f deg C, P: %.2f Pa\n", loop, (data.temperature), (data.pressure));
            #else
            printf("Data[%d]  T: %ld deg C, P: %lu Pa\n", loop, (long int)(int32_t)(data.temperature / 100),
                   (long unsigned int)(uint32_t)(data.pressure / 100));
            #endif

            loop = loop + 1;
        }
    }

    bmp3_coines_deinit();

    return rslt;
}

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

/* Defines frame count requested
 * As, only Temperature is enabled in this example,
 * Total byte count requested : FIFO_FRAME_COUNT * BMP3_LEN_P_OR_T_HEADER_DATA
 */
#define FIFO_FRAME_COUNT  UINT8_C(50)

/* Maximum FIFO size
 * FIFO size is 512 bytes
 * Additional bytes given to read sensortime frame
 */
#define FIFO_MAX_SIZE     UINT16_C(550)

/* Iteration count to run example code */
#define ITERATION         UINT8_C(10)

/************************************************************************/
/*********                     Test code                           ******/
/************************************************************************/

int main(void)
{
    struct bmp3_dev dev;
    int8_t rslt;

    uint8_t try = 1;
    uint8_t index = 0;
    uint16_t settings_sel;
    uint16_t settings_fifo;
    uint16_t fifo_length = 0;
    uint8_t fifo_data[FIFO_MAX_SIZE];
    struct bmp3_data fifo_p_t_data[FIFO_MAX_SIZE];
    struct bmp3_fifo_settings fifo_settings = { 0 };
    struct bmp3_settings settings = { 0 };
    struct bmp3_fifo_data fifo = { 0 };
    struct bmp3_status status = { { 0 } };

    /* Interface reference is given as a parameter
     *         For I2C : BMP3_I2C_INTF
     *         For SPI : BMP3_SPI_INTF
     */
    rslt = bmp3_interface_init(&dev, BMP3_I2C_INTF);
    bmp3_check_rslt("bmp3_interface_init", rslt);

    rslt = bmp3_init(&dev);
    bmp3_check_rslt("bmp3_init", rslt);

    fifo_settings.mode = BMP3_ENABLE;
    fifo_settings.temp_en = BMP3_ENABLE;
    fifo_settings.filter_en = BMP3_ENABLE;
    fifo_settings.down_sampling = BMP3_FIFO_NO_SUBSAMPLING;
    fifo_settings.ffull_en = BMP3_ENABLE;
    fifo_settings.time_en = BMP3_ENABLE;

    fifo.buffer = fifo_data;
    fifo.req_frames = FIFO_FRAME_COUNT;

    settings.temp_en = BMP3_ENABLE;
    settings.odr_filter.temp_os = BMP3_NO_OVERSAMPLING;
    settings.odr_filter.odr = BMP3_ODR_12_5_HZ;

    settings_sel = BMP3_SEL_TEMP_EN | BMP3_SEL_TEMP_OS | BMP3_SEL_ODR;

    settings_fifo = BMP3_SEL_FIFO_MODE | BMP3_SEL_FIFO_TEMP_EN | BMP3_SEL_FIFO_FULL_EN | BMP3_SEL_FIFO_TIME_EN |
                    BMP3_SEL_FIFO_DOWN_SAMPLING | BMP3_SEL_FIFO_FILTER_EN;

    rslt = bmp3_set_sensor_settings(settings_sel, &settings, &dev);
    bmp3_check_rslt("bmp3_set_sensor_settings", rslt);

    settings.op_mode = BMP3_MODE_NORMAL;
    rslt = bmp3_set_op_mode(&settings, &dev);
    bmp3_check_rslt("bmp3_set_op_mode", rslt);

    rslt = bmp3_set_fifo_settings(settings_fifo, &fifo_settings, &dev);
    bmp3_check_rslt("bmp3_set_fifo_settings", rslt);

    rslt = bmp3_get_fifo_settings(&fifo_settings, &dev);
    bmp3_check_rslt("bmp3_get_fifo_settings", rslt);

    printf("Read Fifo full interrupt data with sensor time\n");

    while (try <= ITERATION)
    {
        rslt = bmp3_get_status(&status, &dev);
        bmp3_check_rslt("bmp3_get_status", rslt);

        if ((rslt == BMP3_OK) && (status.intr.fifo_full == BMP3_ENABLE))
        {
            rslt = bmp3_get_fifo_length(&fifo_length, &dev);
            bmp3_check_rslt("bmp3_get_fifo_length", rslt);

            rslt = bmp3_get_fifo_data(&fifo, &fifo_settings, &dev);
            bmp3_check_rslt("bmp3_get_fifo_data", rslt);

            /* NOTE : Read status register again to clear FIFO Full interrupt status */
            rslt = bmp3_get_status(&status, &dev);
            bmp3_check_rslt("bmp3_get_status", rslt);

            if (rslt == BMP3_OK)
            {
                rslt = bmp3_extract_fifo_data(fifo_p_t_data, &fifo, &dev);
                bmp3_check_rslt("bmp3_extract_fifo_data", rslt);

                printf("\nIteration : %d\n", try);
                printf("Available fifo length : %d\n", fifo_length);
                printf("Fifo byte count from fifo structure : %d\n", fifo.byte_count);

                printf("FIFO compensation Pressure and Temperature frames requested : %d\n", fifo.req_frames);

                printf("FIFO frames extracted : %d\n", fifo.parsed_frames);

                for (index = 0; index < fifo.parsed_frames; index++)
                {
                    #ifdef BMP3_FLOAT_COMPENSATION
                    printf("Frame[%d]  T: %.2f deg C, P: %.2f Pa\n", index, (fifo_p_t_data[index].temperature),
                           (fifo_p_t_data[index].pressure));
                    #else
                    printf("Frame[%d]  T: %ld deg C, P: %lu Pa\n", index,
                           (long int)(int32_t)(fifo_p_t_data[index].temperature / 100),
                           (long unsigned int)(uint32_t)(fifo_p_t_data[index].pressure / 100));
                    #endif
                }

                printf("Sensor time : %lu\n", (long unsigned int)fifo.sensor_time);

                try++;
            }
        }
    }

    bmp3_coines_deinit();

    return rslt;
}

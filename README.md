# BMP3 sensor API

## Introduction

This package contains the Bosch Sensortec's BMP3 pressure sensor driver (sensor API)

The sensor driver package includes bmp3.h, bmp3.c and bmp3_defs.h files

## Version

File        | Version | Date
------------|---------|-----
bmp3.c      |  1.1.0  | 05 Apr 2018
bmp3.h      |  1.1.0  | 05 Apr 2018
bmp3_defs.h |  1.1.0  | 05 Apr 2018

## Integration details

- Integrate bmp3.h, bmp3_defs.h and bmp3.c file in to your project.
- Include the bmp3.h file in your code like below.

```c
#include "bmp3.h"
```

## File information

- bmp3_defs.h : This header file has the constants, macros and datatype declarations.
- bmp3.h : This header file contains the declarations of the sensor driver APIs.
- bmp3.c : This source file contains the definitions of the sensor driver APIs.

## Supported sensor interfaces

- SPI 4-wire
- I2C

## Usage guide

### Initializing the sensor

To initialize the sensor, you will first need to create a device structure. You 
can do this by creating an instance of the structure bmp3_dev. Then go on to
fill in the various parameters as shown below.

Regarding Compensation functions for temperature and pressure, we have two implementations.
1) Double precision floating point version
2) Integer version

If you want to use the floating point version, define the BMP3_DOUBLE_PRECISION_COMPENSATION macro in your makefile or uncomment the relevant line in the bmp3_defs.h file. By default, the integer version will be used. Below example code uses floating point representation.

#### Example for SPI 4-Wire

```c
struct bmp3_dev dev;
int8_t rslt = BMP3_OK;

/* Sensor_0 interface over SPI with native chip select line */
dev.dev_id = 0;
dev.intf = BMP3_SPI_INTF;
dev.read = user_spi_read;
dev.write = user_spi_write;
dev.delay_ms = user_delay_ms;

rslt = bmp3_init(&dev);
```

#### Example for I2C

```c
struct bmp3_dev dev;
int8_t rslt = BMP3_OK;

dev.dev_id = BMP3_I2C_ADDR_PRIM;
dev.intf = BMP3_I2C_INTF;
dev.read = user_i2c_read;
dev.write = user_i2c_write;
dev.delay_ms = user_delay_ms;

rslt = bmp3_init(&dev);
```

### Configuring the sensor

#### Forced mode

##### Example for configuring the sensor without oversampling settings

```c
int8_t set_forced_mode(struct bmp3_dev *dev)
{
    int8_t rslt;
    /* Used to select the settings user needs to change */
    uint16_t settings_sel;

    /* Select the pressure and temperature sensor to be enabled */
    dev->settings.press_en = BMP3_ENABLE;
    dev->settings.temp_en = BMP3_ENABLE;
    /* Assign the settings which needs to be set in the sensor */
    settings_sel = BMP3_PRESS_EN_SEL | BMP3_TEMP_EN_SEL;
    /* Write the settings in the sensor */
    rslt = bmp3_set_sensor_settings(settings_sel, dev);

    /* Select the power mode */
    dev->settings.op_mode = BMP3_FORCED_MODE;
    /* Set the power mode in the sensor */
    rslt = bmp3_set_op_mode(dev);

    return rslt;
}
```

##### Example for configuring the sensor with oversampling settings

```c
int8_t set_forced_mode_with_osr(struct bmp3_dev *dev)
{
    int8_t rslt;
    /* Used to select the settings user needs to change */
    uint16_t settings_sel;

    /* Select the pressure and temperature sensor to be enabled */
    dev->settings.press_en = BMP3_ENABLE;
    dev->settings.temp_en = BMP3_ENABLE;
    /* Select the oversampling settings for pressure and temperature */
    dev->settings.odr_filter.press_os = BMP3_OVERSAMPLING_2X;
    dev->settings.odr_filter.temp_os = BMP3_OVERSAMPLING_2X;
    /* Assign the settings which needs to be set in the sensor */
    settings_sel = BMP3_PRESS_EN_SEL | BMP3_TEMP_EN_SEL | BMP3_PRESS_OS_SEL | BMP3_TEMP_OS_SEL;
    /* Write the settings in the sensor */
    rslt = bmp3_set_sensor_settings(settings_sel, dev);

    /* Select the power mode */
    dev->settings.op_mode = BMP3_FORCED_MODE;
    /* Set the power mode in the sensor */
    rslt = bmp3_set_op_mode(dev);

    return rslt;
}
```

#### Normal mode

##### Example for configuring the sensor with output data rate and oversampling settings

```c
int8_t set_normal_mode(struct bmp3_dev *dev)
{
    int8_t rslt;
    /* Used to select the settings user needs to change */
    uint16_t settings_sel;

    /* Select the pressure and temperature sensor to be enabled */
    dev->settings.press_en = BMP3_ENABLE;
    dev->settings.temp_en = BMP3_ENABLE;
    /* Select the output data rate and oversampling settings for pressure and temperature */
    dev->settings.odr_filter.press_os = BMP3_NO_OVERSAMPLING;
    dev->settings.odr_filter.temp_os = BMP3_NO_OVERSAMPLING;
    dev->settings.odr_filter.odr = BMP3_ODR_200_HZ;
    /* Assign the settings which needs to be set in the sensor */
    settings_sel = BMP3_PRESS_EN_SEL | BMP3_TEMP_EN_SEL | BMP3_PRESS_OS_SEL | BMP3_TEMP_OS_SEL | BMP3_ODR_SEL;
    rslt = bmp3_set_sensor_settings(settings_sel, dev);

    /* Set the power mode to normal mode */
    dev->settings.op_mode = BMP3_NORMAL_MODE;
    rslt = bmp3_set_op_mode(dev);

    return rslt;
}
```

### Reading sensor data

#### Example for reading all sensor data

```c
int8_t get_sensor_data(struct bmp3_dev *dev)
{
    int8_t rslt;
    /* Variable used to select the sensor component */
    uint8_t sensor_comp;
    /* Variable used to store the compensated data */
    struct bmp3_data data;

    /* Sensor component selection */
    sensor_comp = BMP3_PRESS | BMP3_TEMP;
    /* Temperature and Pressure data are read and stored in the bmp3_data instance */
    rslt = bmp3_get_sensor_data(sensor_comp, &data, dev);

    /* Print the temperature and pressure data */
    printf("Temperature\t Pressure\t\n");
    printf("%0.2f\t\t %0.2f\t\t\n",data.temperature, data.pressure);

    return rslt;
}
```

### Configure and read FIFO data

#### Example for configuring and reading the FIFO data.

```c
int8_t configure_and_get_fifo_data(struct bmp3_dev *dev)
{
    int8_t rslt;
    /* Loop Variable */
    uint8_t i;
    /* FIFO object to be assigned to device structure */
    struct bmp3_fifo fifo;
    /* Pressure and temperature array of structures with maximum frame size */
    struct bmp3_data sensor_data[73] = {0};
    /* Used to select the settings user needs to change */
    uint16_t settings_sel;
    /* try count for polling the watermark interrupt status */
    uint8_t try_count;

    /* Enable fifo */
    fifo.settings.mode = BMP3_ENABLE;
    /* Enable Pressure sensor for fifo */
    fifo.settings.press_en = BMP3_ENABLE;
    /* Enable temperature sensor for fifo */
    fifo.settings.temp_en = BMP3_ENABLE;
    /* Enable fifo time */
    fifo.settings.time_en = BMP3_ENABLE;
    /* No subsampling for FIFO */
    fifo.settings.down_sampling = BMP3_FIFO_NO_SUBSAMPLING;
    /* FIFO watemrmark interrupt enable */
    fifo.settings.fwtm_en = BMP3_ENABLE;

     /* Link the fifo object to device structure */
    dev->fifo = &fifo;
    /* Select the settings required for fifo */
    settings_sel = BMP3_FIFO_MODE_SEL | BMP3_FIFO_TIME_EN_SEL | BMP3_FIFO_TEMP_EN_SEL |
            BMP3_FIFO_PRESS_EN_SEL | BMP3_FIFO_DOWN_SAMPLING_SEL | BMP3_FIFO_FWTM_EN_SEL;
    /* Set the selected settings in fifo */
    rslt = bmp3_set_fifo_settings(settings_sel, dev);

    /* Set the number of frames to be read so as to set the watermark length in the sensor */
    dev->fifo->data.req_frames = 50;
    rslt = bmp3_set_fifo_watermark(dev);

    /* Set the power mode to normal */
    rslt = set_normal_mode(dev);

    /* TODO : To calculate the exact time for try count variable */
    try_count = 0xFFFF;
    /* Poll till watermark level is reached in fifo */
    do {
        rslt = bmp3_get_status(dev);
        try_count--;
    } while ((dev->status.intr.fifo_wm == 0) && (try_count > 0));

    if (try_count > 0) {
        rslt = bmp3_get_fifo_data(dev);
        rslt = bmp3_extract_fifo_data(sensor_data, dev);
        printf("FIFO data\n");
        printf("Temp\tPress\n");
        /* Print the fifo data */
        for (i = 0; i < dev->fifo->data.req_frames; i++) {
#ifdef FLOATING_POINT_COMPENSATION
            printf("%0.2f\t%0.2f\n", sensor_data[i].temperature,sensor_data[i].pressure);
#else
            printf("%lld\t%lld\n", sensor_data[i].temperature,sensor_data[i].pressure);
#endif
        }
    } else {
        rslt = BMP3_E_FIFO_WATERMARK_NOT_REACHED;
    }

    return rslt;
}
```

## Copyright (C) 2017 - 2018 Bosch Sensortec GmbH
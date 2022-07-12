# BMP3 sensor API

## Introduction

This package contains the Bosch Sensortec's BMP3 pressure sensor driver (sensor API)

The sensor driver package includes bmp3.h, bmp3.c and bmp3_defs.h files

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

If you want to use the floating point version, define the BMP3_FLOAT_COMPENSATION macro in your makefile or uncomment the relevant line in the bmp3_defs.h file. By default, the integer version will be used. Below example code uses floating point representation.

### Important links

- [BMP388 product page](https://www.bosch-sensortec.com/products/environmental-sensors/pressure-sensors/bmp388/)
- [BMP390L product page](https://www.bosch-sensortec.com/products/environmental-sensors/pressure-sensors/bmp390l/)
- [BMP388 datasheet](https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bmp388-ds001.pdf)
- [BMP390L datasheet](https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bmp390l-ds001.pdf)
- [BMP3xy shuttle board flyer](https://www.bosch-sensortec.com/media/boschsensortec/downloads/shuttle_board_flyer/bst-mhd-fl001.pdf)
- [Community support page](https://community.bosch-sensortec.com)
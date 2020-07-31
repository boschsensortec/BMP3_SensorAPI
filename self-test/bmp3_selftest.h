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
* @file       bmp3_selftest.h
* @date       2020-07-20
* @version    v2.0.1
*
*/

#ifndef BMP38X_SELFTEST_H_
#define BMP38X_SELFTEST_H_

/*! CPP guard */
#ifdef __cplusplus
extern "C" {
#endif

#include "bmp3.h"

/*Error codes for self test  */

#define BMP3_SENSOR_OK                                      UINT8_C(0)
#define BMP3_COMMUNICATION_ERROR_OR_WRONG_DEVICE            UINT8_C(10)
#define BMP3_TRIMMING_DATA_OUT_OF_BOUND                     UINT8_C(20)
#define BMP3_TEMPERATURE_BOUND_WIRE_FAILURE_OR_MEMS_DEFECT  UINT8_C(30)
#define BMP3_PRESSURE_BOUND_WIRE_FAILURE_OR_MEMS_DEFECT     UINT8_C(31)
#define BMP3_IMPLAUSIBLE_TEMPERATURE                        UINT8_C(40)
#define BMP3_IMPLAUSIBLE_PRESSURE                           UINT8_C(41)

/**
 * \ingroup bmp3
 * \defgroup bmp3ApiSelftest Self test
 * @brief Perform self test of sensor
 */

/*!
 * \ingroup bmp3ApiSelftest
 * \page bmp3_api_bmp3_selftest_check bmp3_selftest_check
 * \code
 * int8_t bmp3_selftest_check(const struct bmp3_dev *dev);
 * \endcode
 * @details Self-test API for the BMP38X
 *
 * @param[in]   dev Device structure containing relevant information on how
 *                  to communicate with the sensor
 *
 * @return      Error code
 * @retval      0   Success
 * @retval      > 0 Error
 */
int8_t bmp3_selftest_check(struct bmp3_dev *dev);

/*! CPP guard */
#ifdef __cplusplus
}
#endif

#endif /* BMP38X_SELFTEST_H_ */
/** @}*/

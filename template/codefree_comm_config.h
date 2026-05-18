/**
 * @file The codefree_comm module template definitions.
 * @internal
 *
 * @copyright (C) 2025 Melexis N.V.
 *
 * Melexis N.V. is supplying this code for use with Melexis N.V. processor based
 * microcontrollers only.
 *
 * THIS SOFTWARE IS PROVIDED "AS IS".  NO WARRANTIES, WHETHER EXPRESS, IMPLIED
 * OR STATUTORY, INCLUDING, BUT NOT LIMITED TO, IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE.
 * MELEXIS N.V. SHALL NOT IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL,
 * INCIDENTAL, OR CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
 *
 * @endinternal
 *
 * @ingroup application
 *
 * @details
 * This file is to be moved to the application inc/ directory.
 * It is used to set configuration parameters for the codefree communication
 * lib.
 */

#ifndef CODEFREE_COMM_CONFIG_H
#define CODEFREE_COMM_CONFIG_H

/** Common configuration */
#define CODEFREE_COMM_RECEIVE_TIMEOUT_MS 1

/** UART configuration */
/** USART driver instance, used by CMSIS uart driver */
#define CODEFREE_COMM_USART_DRIVER_INSTANCE 0

/** I2C configuration */
/** I2C driver instance, used by CMSIS i2c driver */
#define CODEFREE_COMM_I2C_DRIVER_INSTANCE 0

#endif /* CODEFREE_COMM_CONFIG_H */
/* EOF */

/**
 * @file The codefree_comm module definitions.
 * @internal
 *
 * @copyright (C) 2025 Melexis N.V.
 *
 * Melexis N.V. is supplying this code for use with Melexis N.V. processor based microcontrollers only.
 *
 * THIS SOFTWARE IS PROVIDED "AS IS".  NO WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY,
 * INCLUDING, BUT NOT LIMITED TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE.  MELEXIS N.V. SHALL NOT IN ANY CIRCUMSTANCES,
 * BE LIABLE FOR SPECIAL, INCIDENTAL, OR CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
 *
 * @endinternal
 *
 * @ingroup application
 *
 * @details
 * This file contains the external definitions of the codefree_comm module.
 * The definitions in this file are to be implemented by the application.
 */

 #ifndef CODEFREE_COMM_EXTERNAL_H_
    #define CODEFREE_COMM_EXTERNAL_H_

#include <stdint.h>
#include <stdbool.h>

#include "codefree_comm.h"

/** Application-level hardware initialization hook.
 *
 * This function is called by the library ONLY to initialise the gpio pins.
 * It should put them in the correct mode (push-pull, open-drain)
 * It should connect them to the correct block (I2C, UART)
 */
extern void codefree_comm_ext_hw_init(void);

/** Application-level time hook for timeouts.
 *
 * This function is called by the library only to check for timeout on receive message.
 * It should return the current system time [ms]
 */
extern uint32_t codefree_comm_ext_getTimeMs(void);

/**
 * Initialize the underlying driver for communication with the 80339 device.
 *
 * @return bool true if initialization was successful, false otherwise
 */
static inline bool codefree_comm_ext_initDriver(void);

/**
 * Send the write message to the specified slave device.
 *
 * @param msg message structure containing the write data
 * @retval true write message sent successfully
 * @retval false write message sending failed
 */
static inline bool codefree_comm_ext_sendWriteMessage(CodefreeCommWriteMessage_t* msg);

/**
 * Send a read request to the specified slave device.
 *
 * @param readMsgStruct pointer to the read message struct to be filled up with incoming message
 * @note the slave address in the data structure needs to be set to the slave address you want to read.
 * @retval true read request sent successfully
 * @retval false read request sending failed
 */
static inline bool codefree_comm_ext_sendReadRequest(CodefreeCommReadMessage_t* readMsgStruct);

#endif /* CODEFREE_COMM_EXTERNAL_H_ */


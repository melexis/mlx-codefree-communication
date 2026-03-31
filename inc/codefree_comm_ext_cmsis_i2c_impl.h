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
 * This file contains the definitions of the codefree_comm module.
 * This module can be used to send commands to the codefree 80339 product via I2C.
 */

#ifndef CODEFREE_COMM_EXT_CMSIS_IMPL_H_
#define CODEFREE_COMM_EXT_CMSIS_IMPL_H_

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include "Driver_I2C.h"
#include "codefree_comm.h"
#include "codefree_comm_external.h"

/* ----------------------------------------------------------------------------
 * Configuration & Macros
 * ------------------------------------------------------------------------- */

/* Define default driver instance if not provided by build system */
#ifndef CODEFREE_COMM_I2C_DRIVER_INSTANCE
#define CODEFREE_COMM_I2C_DRIVER_INSTANCE 0
#endif

extern ARM_DRIVER_I2C ARM_Driver_I2C_(CODEFREE_COMM_I2C_DRIVER_INSTANCE);

/* Define standard fast mode by default (400kHz) */
#ifndef CODEFREE_COMM_I2C_BUS_SPEED
#define CODEFREE_COMM_I2C_BUS_SPEED ARM_I2C_BUS_SPEED_FAST
#endif

/* ----------------------------------------------------------------------------
 * Local function prototypes
 * ------------------------------------------------------------------------- */
static inline void codefree_comm_ext_callback(uint32_t event);

/* ----------------------------------------------------------------------------
 * Public functions
 * ------------------------------------------------------------------------- */

static inline bool codefree_comm_ext_initDriver(void)
{
    bool retVal = true;
    ARM_DRIVER_I2C* i2cDrv = &ARM_Driver_I2C_(CODEFREE_COMM_I2C_DRIVER_INSTANCE);

    /* 1. Hardware Init */
    codefree_comm_ext_hw_init();

    /* 2. CMSIS Initialization */
    retVal = (i2cDrv->Initialize(codefree_comm_ext_callback) == ARM_DRIVER_OK);
    if (retVal) {
        retVal = (i2cDrv->PowerControl(ARM_POWER_FULL) == ARM_DRIVER_OK);
    }

    if (retVal) {
        retVal = (i2cDrv->Control(ARM_I2C_BUS_SPEED, CODEFREE_COMM_I2C_BUS_SPEED) == ARM_DRIVER_OK);
    }

    return retVal;
}

static inline bool codefree_comm_ext_sendWriteMessage(CodefreeCommWriteMessage_t* msg)
{
    ARM_DRIVER_I2C* i2cDrv = &ARM_Driver_I2C_(CODEFREE_COMM_I2C_DRIVER_INSTANCE);
    bool retVal = true;

    if (i2cDrv->GetStatus().busy) {
        retVal = false;
    } else {
        /** Frame Format: [0x00][Cmd][Data0..3][CRC]
         * Length: 1 + 1 + 4 + 1 = 7 bytes
         */
        static uint8_t messageBuffer[7];
        messageBuffer[0] = 0;
        messageBuffer[1] = msg->commandId;
        memcpy(&messageBuffer[2], msg->data, 4);
        messageBuffer[6] = msg->crc;

        if (i2cDrv->MasterTransmit(msg->slaveAddress, messageBuffer, 7, false) != ARM_DRIVER_OK) {
            retVal = false;
        }
    }

    return retVal;
}

static inline bool codefree_comm_ext_sendReadRequest(CodefreeCommReadMessage_t* readMsgStruct)
{
    ARM_DRIVER_I2C* i2cDrv = &ARM_Driver_I2C_(CODEFREE_COMM_I2C_DRIVER_INSTANCE);
    bool retVal = true;

    /* Non-blocking check: Do not execute if hardware is still transmitting/receiving */
    if (i2cDrv->GetStatus().busy) {
        retVal = false;
    } else {
        if (i2cDrv->MasterReceive(readMsgStruct->slaveAddress, readMsgStruct->readResponse, 8,
                                  false) != ARM_DRIVER_OK) {
            retVal = false;
        }
    }

    return retVal;
}

/* ----------------------------------------------------------------------------
 * Local function implementation
 * ------------------------------------------------------------------------- */

static inline void codefree_comm_ext_callback(uint32_t event)
{
    ARM_DRIVER_I2C* i2cDrv = &ARM_Driver_I2C_(CODEFREE_COMM_I2C_DRIVER_INSTANCE);

    /* 1. Transfer successfully finished (both TX and RX complete trigger this in I2C) */
    if (event & ARM_I2C_EVENT_TRANSFER_DONE) {
        if (codefree_comm_getStatus() == CODEFREE_COMM_STATUS_SENDING_READ) {
            codefree_comm_handleRxComplete();
            /* Note: Ensure handleRxComplete updates the status to IDLE or appropriate next state */
        } else if (codefree_comm_getStatus() == CODEFREE_COMM_STATUS_SENDING_WRITE) {
            codefree_comm_setStatus(CODEFREE_COMM_STATUS_IDLE);
        }
    }

    /* 2. Error Detection */
    uint32_t errorMask = ARM_I2C_EVENT_TRANSFER_INCOMPLETE |
                         ARM_I2C_EVENT_ADDRESS_NACK |
                         ARM_I2C_EVENT_ARBITRATION_LOST |
                         ARM_I2C_EVENT_BUS_ERROR;

    if ((event & errorMask) != 0) {
        /** Abort the current transfer and reset state */
        i2cDrv->Control(ARM_I2C_ABORT_TRANSFER, 0);
        codefree_comm_setStatus(CODEFREE_COMM_STATUS_IDLE);
    }
}

#endif /* CODEFREE_COMM_EXT_CMSIS_IMPL_H_ */


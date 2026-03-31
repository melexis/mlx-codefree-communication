/**
 * @file
 * @brief The codefree_comm module.
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
 * @details This file contains the implementations of the codefree_comm module.
 */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "codefree_comm.h"
#include "codefree_comm_config.h"
#include "codefree_comm_external.h"

#ifdef USE_CODEFREE_COMM_I2C
    #include "codefree_comm_ext_cmsis_i2c_impl.h"
#else
    #ifdef USE_CODEFREE_COMM_UART
    #include "codefree_comm_ext_cmsis_uart_impl.h"
    #else
    #error Codefree communication should have a communication method selected in codefree_config.h
    #endif
#endif

/* ---------------------------------------------
 * Variables
 * --------------------------------------------- */
CodefreeCommWriteMessage_t writeInterface;
CodefreeCommReadMessage_t* readInterfaceApplication;
CodefreeCommReadMessage_t readInterfaceIO;
CodefreeCommStatus_t codefree_commStatus = CODEFREE_COMM_STATUS_INACTIVE;

static CodefreeComm_ReadCallback_t codefree_comm_appCallback = NULL;
static uint32_t readStartTime = 0;

/* ---------------------------------------------
 * Private Function prototypes
 * --------------------------------------------- */
static void codefree_comm_calcWriteCrc(CodefreeCommWriteMessage_t* msg);
static void codefree_comm_clearWriteData(CodefreeCommWriteMessage_t* msg);
static void codefree_comm_startWriteTransmit(void);
static inline uint8_t codefree_comm_getMSB(uint16_t value);
static inline uint8_t codefree_comm_getLSB(uint16_t value);

/* ---------------------------------------------
 * Public Function implementation
 * --------------------------------------------- */
/** --- General functions --- */
bool codefree_comm_init(CodefreeComm_ReadCallback_t appCallback)
{
    bool retVal = false;
    codefree_comm_appCallback = appCallback;
    retVal = codefree_comm_ext_initDriver();
    if (retVal) {
        codefree_commStatus = CODEFREE_COMM_STATUS_IDLE;
    }

    return retVal;
}

/* --- Write Commands --- */

void codefree_comm_SetSpeedRelative(uint8_t slvAddr, uint8_t speedRel)
{
    codefree_comm_clearWriteData(&writeInterface);
    writeInterface.slaveAddress = slvAddr;
    writeInterface.commandId = CODEFREE_COMM_COMMAND_SETSPEEDRELATIVE;
    writeInterface.data[0] = speedRel;
    codefree_comm_startWriteTransmit();
}

void codefree_comm_SetSpeedAbsolute(uint8_t slvAddr, uint16_t speedAbs)
{
    codefree_comm_clearWriteData(&writeInterface);
    writeInterface.slaveAddress = slvAddr;
    writeInterface.commandId = CODEFREE_COMM_COMMAND_SETSPEEDABSOLUTE;
    writeInterface.data[0] = codefree_comm_getMSB(speedAbs);
    writeInterface.data[1] = codefree_comm_getLSB(speedAbs);
    codefree_comm_startWriteTransmit();
}

void codefree_comm_SetPositionRelative(uint8_t slvAddr, uint8_t posRel, uint8_t speedTarget)
{
    codefree_comm_clearWriteData(&writeInterface);
    writeInterface.slaveAddress = slvAddr;
    writeInterface.commandId = CODEFREE_COMM_COMMAND_SETPOSITIONRELATIVE;
    writeInterface.data[0] = posRel;
    writeInterface.data[2] = speedTarget;
    codefree_comm_startWriteTransmit();
}

void codefree_comm_SetPositionAbsolute(uint8_t slvAddr, uint16_t posAbs, uint8_t speedTarget)
{
    codefree_comm_clearWriteData(&writeInterface);
    writeInterface.slaveAddress = slvAddr;
    writeInterface.commandId = CODEFREE_COMM_COMMAND_SETPOSITIONABSOLUTE;
    writeInterface.data[0] = codefree_comm_getMSB(posAbs);
    writeInterface.data[1] = codefree_comm_getLSB(posAbs);
    writeInterface.data[2] = speedTarget;
    codefree_comm_startWriteTransmit();
}

void codefree_comm_SetActiveCommand(uint8_t slvAddr, CodefreeCommSupportedCommands_t activeCommandId)
{
    codefree_comm_clearWriteData(&writeInterface);
    writeInterface.slaveAddress = slvAddr;
    writeInterface.commandId = CODEFREE_COMM_COMMAND_SETACTIVECOMMAND;
    writeInterface.data[0] = 0x45;
    writeInterface.data[1] = 0x67;
    writeInterface.data[2] = 0x78;
    writeInterface.data[3] = activeCommandId;
    codefree_comm_startWriteTransmit();
}

/* --- Read Commands --- */

bool codefree_comm_executeRead(CodefreeCommReadMessage_t* readMsgStruct)
{
    bool retVal = false;
    if (codefree_comm_getStatus() == CODEFREE_COMM_STATUS_IDLE) {
        readInterfaceApplication = readMsgStruct;
        readInterfaceIO.slaveAddress = readInterfaceApplication->slaveAddress;
        readInterfaceIO.crcCorrect = false;
        codefree_comm_setStatus(CODEFREE_COMM_STATUS_SENDING_READ);
        readStartTime = codefree_comm_ext_getTimeMs();
        retVal = codefree_comm_ext_sendReadRequest(&readInterfaceIO);

        /* If the hardware driver rejected the request,
         * abort the timeout and revert the state machine. */
        if (!retVal) {
            codefree_comm_setStatus(CODEFREE_COMM_STATUS_IDLE);
        }
    }

    return retVal;
}

bool codefree_comm_checkReadCrc(CodefreeCommReadMessage_t* msg)
{
    uint16_t u16CRC = 0;

    u16CRC += ((uint16_t)msg->slaveAddress << 1) | 0x01;

    for (uint8_t i = 0; i < 8; i++) {
        u16CRC += msg->readResponse[i];
    }

    while ((u16CRC & 0xFF00u) != 0u) {
        u16CRC = (u16CRC >> 8u) + (u16CRC & 0x00FFu);
    }

    msg->crcCorrect = (u16CRC == 0xFFu);

    return msg->crcCorrect;
}

void codefree_comm_handleRxComplete(void)
{
    readInterfaceIO = codefree_comm_getReadInterfaceIO();
    memcpy(&readInterfaceApplication->payload.raw[0], &readInterfaceIO.payload.raw[0], 8);
    readInterfaceApplication->crcCorrect = codefree_comm_checkReadCrc(readInterfaceApplication);
    codefree_comm_setStatus(CODEFREE_COMM_STATUS_IDLE);

    if (codefree_comm_appCallback != NULL) {
        codefree_comm_appCallback(readInterfaceApplication, CODEFREE_COMM_EVENT_READ_COMPLETE);
    }
}

void codefree_comm_backgroundHandler(void)
{
    if (codefree_comm_getStatus() == CODEFREE_COMM_STATUS_SENDING_READ) {
        if ((codefree_comm_ext_getTimeMs() - readStartTime) >= CODEFREE_COMM_RECEIVE_TIMEOUT_MS) {
            /* Timeout occurred! */
            codefree_comm_setStatus(CODEFREE_COMM_STATUS_IDLE);
        }
    }
}

/* ---------------------------------------------
 * Private Function implementation
 * --------------------------------------------- */

static void codefree_comm_calcWriteCrc(CodefreeCommWriteMessage_t* msg)
{
    uint16_t u16CRC = 0;

    /* Byte 1: Address (shifted for write) */
    u16CRC += ((uint16_t)msg->slaveAddress << 1) | 0x00;

    /* Byte 2: 0x00 (no effect) */

    /* Byte 3: Command ID */
    u16CRC += msg->commandId;

    /* Bytes 4-7: Data */
    for (uint8_t i = 0; i < 4; i++) {
        u16CRC += msg->data[i];
    }

    while ((u16CRC & 0xFF00u) != 0u) {
        u16CRC = (u16CRC >> 8u) + (u16CRC & 0x00FFu);
    }

    msg->crc = (uint8_t)(0xFF - u16CRC);
}

static void codefree_comm_clearWriteData(CodefreeCommWriteMessage_t* msg)
{
    msg->data[0] = 0;
    msg->data[1] = 0;
    msg->data[2] = 0;
    msg->data[3] = 0;
}

static void codefree_comm_startWriteTransmit(void)
{
    codefree_comm_calcWriteCrc(&writeInterface);
    if (codefree_comm_getStatus() == CODEFREE_COMM_STATUS_IDLE) {
        codefree_comm_setStatus(CODEFREE_COMM_STATUS_SENDING_WRITE);
        if (!codefree_comm_ext_sendWriteMessage(&writeInterface)) {
            codefree_comm_setStatus(CODEFREE_COMM_STATUS_IDLE);
        }
    }
}

static inline uint8_t codefree_comm_getMSB(uint16_t value)
{
    return((value & 0xFF00u) >> 8);
}

static inline uint8_t codefree_comm_getLSB(uint16_t value)
{
    return(value & 0xFFu);
}


/**
 * @file
 * @brief The codefree_comm module.
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
 * @details This file contains the implementations of the codefree_comm module.
 *
 * Frame scheduling:
 * Write commands and read requests are not sent directly when the bus is busy
 * or when the minimum inter-frame gap (CODEFREE_COMM_MIN_FRAME_GAP_MS) has not
 * elapsed yet. Instead, one write (latest command wins) and one read can be
 * pending; they are sent as soon as allowed, either directly from the API call
 * or from codefree_comm_backgroundHandler(). When both are pending they
 * alternate (a write goes first unless the previous frame was a write), so a
 * stream of write commands cannot starve a read and vice versa.
 */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "codefree_comm.h"
#include "codefree_comm_config.h"
#include "codefree_comm_external.h"

/* ---------------------------------------------
 * Configuration defaults
 * --------------------------------------------- */

#ifndef CODEFREE_COMM_RECEIVE_TIMEOUT_MS
#define CODEFREE_COMM_RECEIVE_TIMEOUT_MS 10u
#endif

/* Minimum idle time between the end of one frame and the start of the next.
 * The MLX80339 computes the CRC of a read response over the previous (write)
 * address byte when a read follows a write too quickly. Measured: back-to-back
 * fails, >= 1 ms works. Time base is codefree_comm_ext_getTimeMs(), so the
 * effective gap is (N-1)..N ms. Set to 0 to disable. */
#ifndef CODEFREE_COMM_MIN_FRAME_GAP_MS
#define CODEFREE_COMM_MIN_FRAME_GAP_MS 2u
#endif

/* ---------------------------------------------
 * Variables
 * --------------------------------------------- */
CodefreeCommWriteMessage_t writeInterface;
CodefreeCommReadMessage_t *readInterfaceApplication;
CodefreeCommReadMessage_t readInterfaceIO;
volatile CodefreeCommStatus_t codefree_commStatus =
    CODEFREE_COMM_STATUS_INACTIVE;

static CodefreeComm_ReadCallback_t codefree_comm_appCallback = NULL;
static uint32_t readStartTime = 0;

static bool writePending = false;
static bool readPending = false;
static bool lastSentWasWrite = false;
static volatile bool frameEndValid = false;
static volatile uint32_t lastFrameEndTime = 0;

/* ---------------------------------------------
 * Private Function prototypes
 * --------------------------------------------- */
static void codefree_comm_calcWriteCrc(CodefreeCommWriteMessage_t *msg);
static void codefree_comm_clearWriteData(CodefreeCommWriteMessage_t *msg);
static void codefree_comm_queueWrite(void);
static void codefree_comm_processPending(void);
static bool codefree_comm_frameGapElapsed(void);
static bool codefree_comm_readInProgress(void);
static inline uint8_t codefree_comm_getMSB(uint16_t value);
static inline uint8_t codefree_comm_getLSB(uint16_t value);

/* ---------------------------------------------
 * Public Function implementation
 * --------------------------------------------- */
/** --- General functions --- */
bool codefree_comm_init(CodefreeComm_ReadCallback_t appCallback) {
  bool retVal = false;
  codefree_comm_appCallback = appCallback;
  writePending = false;
  readPending = false;
  frameEndValid = false;
  lastSentWasWrite = false;
  retVal = codefree_comm_ext_initDriver();
  if (retVal) {
    codefree_commStatus = CODEFREE_COMM_STATUS_IDLE;
  }

  return retVal;
}

void codefree_comm_setStatus(CodefreeCommStatus_t status) {
  /* Record the end of a frame (busy -> IDLE) for the inter-frame gap. This is
   * also the path used by the ports when a transfer completes or fails. */
  if ((status == CODEFREE_COMM_STATUS_IDLE) &&
      (codefree_commStatus != CODEFREE_COMM_STATUS_IDLE) &&
      (codefree_commStatus != CODEFREE_COMM_STATUS_INACTIVE)) {
    lastFrameEndTime = codefree_comm_ext_getTimeMs();
    frameEndValid = true;
  }
  codefree_commStatus = status;
}

/* --- Write Commands --- */

void codefree_comm_SetSpeedRelative(uint8_t slvAddr, uint8_t speedRel) {
  codefree_comm_clearWriteData(&writeInterface);
  writeInterface.slaveAddress = slvAddr;
  writeInterface.commandId = CODEFREE_COMM_COMMAND_SETSPEEDRELATIVE;
  writeInterface.data[0] = speedRel;
  codefree_comm_queueWrite();
}

void codefree_comm_SetSpeedAbsolute(uint8_t slvAddr, uint16_t speedAbs) {
  codefree_comm_clearWriteData(&writeInterface);
  writeInterface.slaveAddress = slvAddr;
  writeInterface.commandId = CODEFREE_COMM_COMMAND_SETSPEEDABSOLUTE;
  writeInterface.data[0] = codefree_comm_getMSB(speedAbs);
  writeInterface.data[1] = codefree_comm_getLSB(speedAbs);
  codefree_comm_queueWrite();
}

void codefree_comm_SetPositionRelative(uint8_t slvAddr, uint8_t posRel,
                                       uint8_t speedTarget) {
  codefree_comm_clearWriteData(&writeInterface);
  writeInterface.slaveAddress = slvAddr;
  writeInterface.commandId = CODEFREE_COMM_COMMAND_SETPOSITIONRELATIVE;
  writeInterface.data[0] = posRel;
  writeInterface.data[2] = speedTarget;
  codefree_comm_queueWrite();
}

void codefree_comm_SetPositionAbsolute(uint8_t slvAddr, uint16_t posAbs,
                                       uint8_t speedTarget) {
  codefree_comm_clearWriteData(&writeInterface);
  writeInterface.slaveAddress = slvAddr;
  writeInterface.commandId = CODEFREE_COMM_COMMAND_SETPOSITIONABSOLUTE;
  writeInterface.data[0] = codefree_comm_getMSB(posAbs);
  writeInterface.data[1] = codefree_comm_getLSB(posAbs);
  writeInterface.data[2] = speedTarget;
  codefree_comm_queueWrite();
}

void codefree_comm_SetActiveCommand(
    uint8_t slvAddr, CodefreeCommSupportedCommands_t activeCommandId) {
  codefree_comm_clearWriteData(&writeInterface);
  writeInterface.slaveAddress = slvAddr;
  writeInterface.commandId = CODEFREE_COMM_COMMAND_SETACTIVECOMMAND;
  writeInterface.data[0] = 0x45;
  writeInterface.data[1] = 0x67;
  writeInterface.data[2] = 0x78;
  writeInterface.data[3] = activeCommandId;
  codefree_comm_queueWrite();
}

bool codefree_comm_isWritePending(void) { return writePending; }

/* --- Read Commands --- */

bool codefree_comm_executeRead(CodefreeCommReadMessage_t *readMsgStruct) {
  bool retVal = false;

  if ((readMsgStruct != NULL) &&
      (codefree_comm_getStatus() != CODEFREE_COMM_STATUS_INACTIVE) &&
      !readPending && !codefree_comm_readInProgress()) {
    readInterfaceApplication = readMsgStruct;
    readPending = true;
    retVal = true;
    codefree_comm_processPending();
  }

  return retVal;
}

bool codefree_comm_checkReadCrc(CodefreeCommReadMessage_t *msg) {
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

void codefree_comm_handleRxComplete(void) {
  /* Ignore late or unsolicited completions, e.g. a response that arrives
   * after the read already timed out. */
  if ((readInterfaceApplication != NULL) && codefree_comm_readInProgress()) {
    memcpy(readInterfaceApplication->readResponse, readInterfaceIO.readResponse,
           sizeof(readInterfaceIO.readResponse));
    readInterfaceApplication->crcCorrect =
        codefree_comm_checkReadCrc(readInterfaceApplication);
    codefree_comm_setStatus(CODEFREE_COMM_STATUS_IDLE);

    if (codefree_comm_appCallback != NULL) {
      codefree_comm_appCallback(readInterfaceApplication,
                                CODEFREE_COMM_EVENT_READ_COMPLETE);
    }
  }
}

void codefree_comm_backgroundHandler(void) {
  /* Response timeout: covers both the request phase (SENDING_READ) and the
   * receive phase (RECEIVING, used by the UART port). */
  if (codefree_comm_readInProgress()) {
    if ((codefree_comm_ext_getTimeMs() - readStartTime) >=
        CODEFREE_COMM_RECEIVE_TIMEOUT_MS) {
      codefree_comm_ext_abortTransfer();
      codefree_comm_setStatus(CODEFREE_COMM_STATUS_IDLE);

      if ((codefree_comm_appCallback != NULL) &&
          (readInterfaceApplication != NULL)) {
        readInterfaceApplication->crcCorrect = false;
        codefree_comm_appCallback(readInterfaceApplication,
                                  CODEFREE_COMM_EVENT_TIMEOUT);
      }
    }
  }

  codefree_comm_processPending();
}

/* ---------------------------------------------
 * Private Function implementation
 * --------------------------------------------- */

static void codefree_comm_calcWriteCrc(CodefreeCommWriteMessage_t *msg) {
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

static void codefree_comm_clearWriteData(CodefreeCommWriteMessage_t *msg) {
  msg->data[0] = 0;
  msg->data[1] = 0;
  msg->data[2] = 0;
  msg->data[3] = 0;
}

/** Finalize the write frame and mark it pending. A previously pending (not yet
 * sent) write is replaced: for setpoint commands the latest value wins. The
 * ports copy the frame into their own buffer on send, so writeInterface may be
 * modified while a previous write is still on the bus. */
static void codefree_comm_queueWrite(void) {
  codefree_comm_calcWriteCrc(&writeInterface);
  if (codefree_comm_getStatus() != CODEFREE_COMM_STATUS_INACTIVE) {
    writePending = true;
    codefree_comm_processPending();
  }
}

/** Send the next pending frame if the bus is idle and the gap has elapsed. */
static void codefree_comm_processPending(void) {
  if ((codefree_comm_getStatus() != CODEFREE_COMM_STATUS_IDLE) ||
      !codefree_comm_frameGapElapsed()) {
    return;
  }

  bool sendRead = readPending && (!writePending || lastSentWasWrite);
  bool prevLastSentWasWrite = lastSentWasWrite;

  /* If the port rejects a frame (e.g. the driver is still busy finishing an
   * aborted transfer), the frame stays pending and is retried after the gap:
   * going back to IDLE marks a frame end. A write is not lost, and an accepted
   * read still ends with READ_COMPLETE or TIMEOUT. */
  if (writePending && !sendRead) {
    writePending = false;
    lastSentWasWrite = true;
    codefree_comm_setStatus(CODEFREE_COMM_STATUS_SENDING_WRITE);
    if (!codefree_comm_ext_sendWriteMessage(&writeInterface)) {
      writePending = true;
      lastSentWasWrite = prevLastSentWasWrite;
      codefree_comm_setStatus(CODEFREE_COMM_STATUS_IDLE);
    }
  } else if (sendRead) {
    readPending = false;
    lastSentWasWrite = false;
    readInterfaceIO.slaveAddress = readInterfaceApplication->slaveAddress;
    readInterfaceIO.crcCorrect = false;
    codefree_comm_setStatus(CODEFREE_COMM_STATUS_SENDING_READ);
    readStartTime = codefree_comm_ext_getTimeMs();

    if (!codefree_comm_ext_sendReadRequest(&readInterfaceIO)) {
      readPending = true;
      lastSentWasWrite = prevLastSentWasWrite;
      codefree_comm_setStatus(CODEFREE_COMM_STATUS_IDLE);
    }
  } else {
    /* nothing to send */
  }
}

static bool codefree_comm_frameGapElapsed(void) {
#if (CODEFREE_COMM_MIN_FRAME_GAP_MS > 0)
  return !frameEndValid ||
         ((codefree_comm_ext_getTimeMs() - lastFrameEndTime) >=
          CODEFREE_COMM_MIN_FRAME_GAP_MS);
#else
  return true;
#endif
}

static bool codefree_comm_readInProgress(void) {
  CodefreeCommStatus_t status = codefree_comm_getStatus();
  return (status == CODEFREE_COMM_STATUS_SENDING_READ) ||
         (status == CODEFREE_COMM_STATUS_RECEIVING);
}

static inline uint8_t codefree_comm_getMSB(uint16_t value) {
  return ((value & 0xFF00u) >> 8);
}

static inline uint8_t codefree_comm_getLSB(uint16_t value) {
  return (value & 0xFFu);
}

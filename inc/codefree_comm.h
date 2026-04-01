/**
 * @file The codefree_comm module definitions.
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
 * This file contains the definitions of the codefree_comm module.
 * This module can be used to send commands to the codefree 80339 product.
 */

#ifndef CODEFREE_COMM_H_
#define CODEFREE_COMM_H_

#include <stdbool.h>
#include <stdint.h>

/* ---------------------------------------------
 * Public type Defines
 * --------------------------------------------- */

/**
 * @defgroup codefree_comm_types Type Definitions Data types for the 80339
 * driver.
 * @{
 */

typedef enum CodefreeCommSupportedCommands_e {
  CODEFREE_COMM_COMMAND_NOCOMMAND = 0u,
  CODEFREE_COMM_COMMAND_SETSPEEDRELATIVE = 1u,
  CODEFREE_COMM_COMMAND_SETSPEEDABSOLUTE = 3u,
  CODEFREE_COMM_COMMAND_SETPOSITIONRELATIVE = 9u,
  CODEFREE_COMM_COMMAND_SETPOSITIONABSOLUTE = 12u,
  CODEFREE_COMM_COMMAND_SETACTIVECOMMAND = 102u
} CodefreeCommSupportedCommands_t;

typedef enum CodefreeCommStatus_e {
  CODEFREE_COMM_STATUS_INACTIVE = 0u,
  CODEFREE_COMM_STATUS_IDLE,
  CODEFREE_COMM_STATUS_SENDING_WRITE,
  CODEFREE_COMM_STATUS_SENDING_READ,
  CODEFREE_COMM_STATUS_RECEIVING
} CodefreeCommStatus_t;

typedef enum {
  CODEFREE_COMM_EVENT_READ_COMPLETE,
  CODEFREE_COMM_EVENT_TIMEOUT
} CodefreeCommEvent_t;

typedef struct {
  uint8_t slaveAddress;
  uint8_t commandId;
  uint8_t data[4];
  uint8_t crc;
} CodefreeCommWriteMessage_t;

typedef struct {
  uint8_t slaveAddress;

  union {
    uint8_t readResponse[8]; /* payload[6] + commandId[1] + crc[1] */
    struct {
      union {
        uint8_t raw[6];

        struct {
          uint8_t motorSpeed;
          uint8_t reserved0;
          uint8_t motorCurrent;
          uint8_t motorVoltage;
          uint8_t icTemp;
          uint8_t statusByte;
        } relativeSpeed;

        struct {
          uint8_t motorSpeedMSB;
          uint8_t motorSpeedLSB;
          uint8_t motorCurrent;
          uint8_t motorVoltage;
          uint8_t icTemp;
          uint8_t statusByte;
        } absoluteSpeed;

        struct {
          uint8_t motorPosition;
          uint8_t reserved0;
          uint8_t motorCurrent;
          uint8_t motorVoltage;
          uint8_t icTemp;
          uint8_t statusByte;
        } relativePosition;

        struct {
          uint8_t motorPositionMSB;
          uint8_t motorPositionLSB;
          uint8_t motorCurrent;
          uint8_t motorVoltage;
          uint8_t icTemp;
          uint8_t statusByte;
        } absolutePosition;
      } payload;

      uint8_t commandId;
      uint8_t crc;
    };
  };

  bool crcCorrect;
} CodefreeCommReadMessage_t;

/** Is called from library with an event param
 * - RX complete -> callback (checksum is validated and filled into crcCorrect
 * field)
 * - Timeout -> chip did not respond to read request in time.
 */
typedef void (*CodefreeComm_ReadCallback_t)(CodefreeCommReadMessage_t *msg,
                                            CodefreeCommEvent_t event);

/** @} */ /* End of codefree_comm_types group */

/* ---------------------------------------------
 * Public Function prototypes
 * --------------------------------------------- */

/**
 * @defgroup codefree_comm_functions Function Definitions Function prototypes
 * for the 80339 driver.
 * @{
 */

/* Driver control functions */

/** Initializes the codefree_comm module and underlying communication driver.
 *
 * @param appCallback pointer to function that is to be executed when receiving
 * data.
 * @return bool true if initialization was successful, false otherwise
 */
bool codefree_comm_init(CodefreeComm_ReadCallback_t appCallback);

/* Motor Control Commands */

/** Sets a relative speed target for the specified slave.
 *
 * @param slvAddr The 7-bit address of the slave device.
 * @param speedRel The relative speed adjustment value [0.5%].
 */
void codefree_comm_SetSpeedRelative(uint8_t slvAddr, uint8_t speedRel);

/** Sets an absolute speed target for the specified slave.
 *
 * @param slvAddr The 7-bit address of the slave device.
 * @param speedAbs The absolute target speed value [rpm].
 */
void codefree_comm_SetSpeedAbsolute(uint8_t slvAddr, uint16_t speedAbs);

/** Sets a relative position target for the specified slave.
 *
 * @param slvAddr The 7-bit address of the slave device.
 * @param posRel The relative target position.
 * @param speedTarget Speed target to get to position [0.5%]
 */
void codefree_comm_SetPositionRelative(uint8_t slvAddr, uint8_t posRel,
                                       uint8_t speedTarget);

/** Sets an absolute position target for the specified slave.
 *
 * @param slvAddr The 7-bit address of the slave device.
 * @param posAbs The absolute target position.
 * @param speedTarget Speed target to get to position [0.5%]
 */
void codefree_comm_SetPositionAbsolute(uint8_t slvAddr, uint16_t posAbs,
                                       uint8_t speedTarget);

/** Sets the active response type for the specified slave.
 *
 * @param slvAddr The 7-bit address of the slave device.
 * @param newCommandType The new active response type.
 */
void codefree_comm_SetActiveCommand(
    uint8_t slvAddr, CodefreeCommSupportedCommands_t newCommandType);

/* Execution Helpers */

/** Executes a general read operation from the specified slave.
 * @param readMsgStruct The data structure to fill with received data.
 * @note the slave address in the data structure needs to be set to the slave
 * address you want to read.
 * @note by default, the amount of bytes read is 8.
 * @return bool true if read was succesfully sent, false otherwise
 */
bool codefree_comm_executeRead(CodefreeCommReadMessage_t *readMsgStruct);

/** Check the Checksum of an incoming message
 *
 * @param msg pointer to the message on which to check the checksum.
 * @return bool true if checksum is correct, false otherwise.
 * @note the crcCorrect flag for the checked message is set accordingly.
 */
bool codefree_comm_checkReadCrc(CodefreeCommReadMessage_t *msg);

/** Handle a completed read (should be executed on rx buffer full)
 * This function copies the received data in to the application-provided read
 * buffer. It also executes the checkCrc, and executes the application-provided
 * function.
 */
void codefree_comm_handleRxComplete(void);

/** Communication block background handler
 * Check for timeout on slave response.
 */
void codefree_comm_backgroundHandler(void);

/* ---------------------------------------------
 * Inline functions
 * --------------------------------------------- */

static inline CodefreeCommReadMessage_t codefree_comm_getReadInterfaceIO(void) {
  extern CodefreeCommReadMessage_t readInterfaceIO;
  return readInterfaceIO;
}

static inline void codefree_comm_setStatus(CodefreeCommStatus_t status) {
  extern CodefreeCommStatus_t codefree_commStatus;
  codefree_commStatus = status;
}

static inline CodefreeCommStatus_t codefree_comm_getStatus(void) {
  extern CodefreeCommStatus_t codefree_commStatus;
  return codefree_commStatus;
}

/** @} */ /* End of codefree_comm_functions group */

#endif /* CODEFREE_COMM_H_ */

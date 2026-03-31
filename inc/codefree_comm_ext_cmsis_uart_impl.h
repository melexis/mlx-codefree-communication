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
 * This module can be used to send commands to the codefree 80339 product.
 */

#ifndef CODEFREE_COMM_EXT_CMSIS_IMPL_H_
#define CODEFREE_COMM_EXT_CMSIS_IMPL_H_

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include "dig_io_mux_ll.h"
#include "mlx_percfg.h" /* Required for enableUart */

#include "Driver_USART.h"
#include "codefree_comm.h"
#include "codefree_comm_external.h"

/* ----------------------------------------------------------------------------
 * Configuration & Macros
 * ------------------------------------------------------------------------- */

/* Define default driver instance if not provided by build system */
#ifndef CODEFREE_COMM_USART_DRIVER_INSTANCE
#define CODEFREE_COMM_USART_DRIVER_INSTANCE 1
#endif

extern ARM_DRIVER_USART ARM_Driver_USART_(CODEFREE_COMM_USART_DRIVER_INSTANCE);

#define CODEFREE_COMM_USART_BAUDRATE     2000000u
#define CODEFREE_COMM_USART_CONFIG      (ARM_USART_MODE_ASYNCHRONOUS | \
                                         ARM_USART_DATA_BITS_8 | \
                                         ARM_USART_PARITY_NONE | \
                                         ARM_USART_STOP_BITS_1 | \
                                         ARM_USART_FLOW_CONTROL_NONE)

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

    #if (CODEFREE_COMM_USART_DRIVER_INSTANCE == 0)
    enableUart(UART_INSTANCE_0);
    dig_io_mux_set_1_4_set_output_mode(DIG_IO_MUX_IO_1_4_CTRL, DIG_IO_MUX_IO_1_4_CTRL_OEN_PUSH_PULL);
    dig_io_mux_set_1_4_mux(DIG_IO_MUX_IO_1_4_CTRL, DIG_IO_MUX_IO_1_4_CTRL_OUT_MUX_SEL_UART0_TX);
    dig_io_mux_set_1_5_input_enable(DIG_IO_MUX_IO_1_5_CTRL, DIG_IO_MUX_IO_1_5_CTRL_IEN_ENABLED);
    dig_io_mux_enable_1_5_pullup_resistor(DIG_IO_MUX_IO_1_5_CTRL, DIG_IO_MUX_IO_1_5_CTRL_PUEN_ENABLED);
    dig_io_mux_set_uart0_rx_input_mux(DIG_IO_MUX_UART0_RX_INPUT_MUX, DIG_IO_MUX_UART0_RX_INPUT_MUX_IN_MUX_SEL_IO_1_5);

    #elif (CODEFREE_COMM_USART_DRIVER_INSTANCE == 1)
    enableUart(UART_INSTANCE_1);
    dig_io_mux_set_1_4_set_output_mode(DIG_IO_MUX_IO_1_4_CTRL, DIG_IO_MUX_IO_1_4_CTRL_OEN_PUSH_PULL);
    dig_io_mux_set_1_4_mux(DIG_IO_MUX_IO_1_4_CTRL, DIG_IO_MUX_IO_1_4_CTRL_OUT_MUX_SEL_UART1_TX);
    dig_io_mux_set_1_5_input_enable(DIG_IO_MUX_IO_1_5_CTRL, DIG_IO_MUX_IO_1_5_CTRL_IEN_ENABLED);
    dig_io_mux_enable_1_5_pullup_resistor(DIG_IO_MUX_IO_1_5_CTRL, DIG_IO_MUX_IO_1_5_CTRL_PUEN_ENABLED);
    dig_io_mux_set_uart1_rx_input_mux(DIG_IO_MUX_UART1_RX_INPUT_MUX, DIG_IO_MUX_UART1_RX_INPUT_MUX_IN_MUX_SEL_IO_1_5);
    #endif

    ARM_DRIVER_USART* usartDrv = &ARM_Driver_USART_(CODEFREE_COMM_USART_DRIVER_INSTANCE);
    retVal = (usartDrv->Initialize(codefree_comm_ext_callback) == ARM_DRIVER_OK);

    if (retVal) {
        retVal = (usartDrv->PowerControl(ARM_POWER_FULL) == ARM_DRIVER_OK);
    }

    if (retVal) {
        retVal = (usartDrv->Control(CODEFREE_COMM_USART_CONFIG, CODEFREE_COMM_USART_BAUDRATE) == ARM_DRIVER_OK);
    }

    if (retVal) {
        retVal = (usartDrv->Control(ARM_USART_CONTROL_TX, 1) == ARM_DRIVER_OK);
    }

    if (retVal) {
        retVal = (usartDrv->Control(ARM_USART_CONTROL_RX, 1) == ARM_DRIVER_OK);
    }

    return retVal;
}

static inline bool codefree_comm_ext_sendWriteMessage(CodefreeCommWriteMessage_t* msg)
{
    ARM_DRIVER_USART* usartDrv = &ARM_Driver_USART_(CODEFREE_COMM_USART_DRIVER_INSTANCE);
    bool retVal = true;
    if (usartDrv->GetStatus().tx_busy) {
        retVal = false;
    } else {
        /** Frame Format: [Addr+W][0x00][Cmd][Data0..3][CRC]
         * Length: 1 + 1 + 1 + 4 + 1 = 8 bytes
         */
        static uint8_t messageBuffer[8];
        messageBuffer[0] = (uint8_t)((msg->slaveAddress << 1) | 0x00);
        messageBuffer[1] = 0;
        messageBuffer[2] = msg->commandId;
        memcpy(&messageBuffer[3], msg->data, 4);
        messageBuffer[7] = msg->crc;

        if (usartDrv->Send(messageBuffer, 8) != ARM_DRIVER_OK) {
            retVal = false;
        }
    }

    return retVal;
}

static inline bool codefree_comm_ext_sendReadRequest(CodefreeCommReadMessage_t* readMsgStruct)
{
    ARM_DRIVER_USART* usartDrv = &ARM_Driver_USART_(CODEFREE_COMM_USART_DRIVER_INSTANCE);
    bool retVal = true;
    /* Non-blocking check: Do not execute if hardware is still transmitting */
    if (usartDrv->GetStatus().tx_busy) {
        retVal = false;
    } else {
        if (usartDrv->GetStatus().rx_busy) {
            usartDrv->Control(ARM_USART_ABORT_RECEIVE, 0);
            usartDrv->Control(ARM_USART_CONTROL_RX, 0);
            usartDrv->Control(ARM_USART_CONTROL_RX, 1);
        }

        /* Set up the message (static for memory retention in non-blocking send method) */
        static uint8_t req;
        /* Construct Address Byte: (Address << 1) | Read(1) */
        req = (uint8_t)((readMsgStruct->slaveAddress << 1) | 0x01);

        if (usartDrv->Send(&req, 1) != ARM_DRIVER_OK) {
            return false;
        }

        if (usartDrv->Receive(readMsgStruct->readResponse, 8) != ARM_DRIVER_OK) {
            return false;
        }
    }

    return retVal;
}

/* ----------------------------------------------------------------------------
 * Local function implementation
 * ------------------------------------------------------------------------- */

static inline void codefree_comm_ext_callback(uint32_t event)
{
    ARM_DRIVER_USART* usartDrv = &ARM_Driver_USART_(CODEFREE_COMM_USART_DRIVER_INSTANCE);

    /* 1. RX full */
    if (event & ARM_USART_EVENT_RECEIVE_COMPLETE) {
        codefree_comm_handleRxComplete();
    }

    if (((event & ARM_USART_EVENT_SEND_COMPLETE) != 0) ||
        ((event & ARM_USART_EVENT_TX_COMPLETE) != 0)) {
        if (codefree_comm_getStatus() == CODEFREE_COMM_STATUS_SENDING_READ) {
            codefree_comm_setStatus(CODEFREE_COMM_STATUS_RECEIVING);
        } else if (codefree_comm_getStatus() == CODEFREE_COMM_STATUS_SENDING_WRITE) {
            codefree_comm_setStatus(CODEFREE_COMM_STATUS_IDLE);
        }
    }

    uint32_t errorMask = ARM_USART_EVENT_RX_OVERFLOW |
                         ARM_USART_EVENT_RX_BREAK |
                         ARM_USART_EVENT_RX_FRAMING_ERROR |
                         ARM_USART_EVENT_RX_PARITY_ERROR;

    if ((event & errorMask) != 0) {
        /** Reset uart block */
        usartDrv->Control(ARM_USART_ABORT_RECEIVE, 0);
        usartDrv->Control(ARM_USART_CONTROL_RX, 0);
        usartDrv->Control(ARM_USART_CONTROL_RX, 1);
        codefree_comm_setStatus(CODEFREE_COMM_STATUS_IDLE);
    }
}

#endif /* CODEFREE_COMM_EXT_CMSIS_IMPL_H_ */


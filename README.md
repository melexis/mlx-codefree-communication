# MLX Code-Free Communication Library

This repository contains the MLX Code-Free Communication Library, designed to simplify and accelerate the integration of communication protocols (specifically I2C and UART) into embedded systems. The "code-free" approach aims to minimize the need for extensive manual coding by providing a highly configurable and abstracted interface, primarily leveraging CMSIS for hardware interaction.

## Features

-   **I2C Communication:** Support for inter-integrated circuit communication via a configurable interface.
-   **UART Communication:** Support for Universal Asynchronous Receiver-Transmitter communication.
-   **CMSIS Integration:** Built upon CMSIS (Cortex Microcontroller Software Interface Standard) for portability and standardized access to microcontroller peripherals.
-   **Configuration-Driven:** Designed to be easily configurable through dedicated header files, reducing the need for direct code modifications for typical use cases.

## Implementation Notes for Integrators

To successfully integrate and utilize this library in your project, consider the following:

1.  **Configuration (`codefree_comm_config.h`):**
    The `template/codefree_comm_config.h` file is crucial for tailoring the library to your specific hardware and application needs. Copy this file to your project's include path and modify it according to your system's clock speeds, peripheral addresses, and desired communication parameters (e.g., I2C clock speed, UART baud rate, data bits, parity).
    **Do not modify the original file in `template/`.**

2.  **CMSIS Dependency:**
    This library relies on the ARM CMSIS standard for hardware abstraction. Ensure that your project includes the necessary CMSIS core and device-specific header files and libraries compatible with your target microcontroller. The I2C and UART implementations (`port/cmsis/codefree_comm_cmsis_i2c.c`, `port/cmsis/codefree_comm_cmsis_uart.c`) are built on CMSIS drivers. Build exactly one of them, or provide your own port that implements the functions in `codefree_comm_external.h`.

3.  **Peripheral Drivers:**
    Beyond CMSIS core, you will need to ensure that your project provides the actual CMSIS-compliant I2C and UART peripheral drivers for your specific microcontroller. These drivers typically come from the chip vendor's Software Pack.

4.  **External Interface (`codefree_comm_external.h`):**
    This header defines the external interface for the communication module. Integrators should include this file to access the library's functions.

5.  **Initialization:**
    Proper initialization of the communication modules (I2C, UART) is critical. Refer to the specific API calls exposed through `codefree_comm_external.h` and ensure all configuration settings in `codefree_comm_config.h` are correctly applied before attempting any communication.

6.  **Background handler:**
    Call `codefree_comm_backgroundHandler()` periodically (every main loop iteration or at least every 1 ms). It handles the slave response timeout and sends pending frames.

7.  **Frame scheduling and inter-frame gap:**
    Commands are never dropped. If the bus is busy or the minimum inter-frame gap has not elapsed, one write (the latest command wins) and one read are kept pending and sent from the background handler. When both are pending they alternate, so neither can starve the other. `codefree_comm_executeRead()` returns `true` when the read is accepted (sent or pending); the result is always reported via the application callback.
    The MLX80339 needs idle time between frames: a read that directly follows a write gets a response whose CRC is computed over the write address byte, so the CRC check fails. `CODEFREE_COMM_MIN_FRAME_GAP_MS` (default 2, giving 1..2 ms with a 1 ms time base) prevents this.

8.  **Timeouts:**
    `CODEFREE_COMM_RECEIVE_TIMEOUT_MS` must be longer than one read frame plus latency (I2C 100 kHz: ~1 ms, UART 9600 baud: ~10 ms). On timeout the library calls `codefree_comm_ext_abortTransfer()` and then the application callback with `CODEFREE_COMM_EVENT_TIMEOUT`.

## Porting

A custom port implements these functions from `codefree_comm_external.h`:

| Function | Purpose |
|---|---|
| `codefree_comm_ext_hw_init()` | Pin configuration (called from the port's init) |
| `codefree_comm_ext_getTimeMs()` | Millisecond time base for timeout and inter-frame gap |
| `codefree_comm_ext_initDriver()` | Initialize the communication peripheral |
| `codefree_comm_ext_sendWriteMessage()` | Send a write frame; copy the frame, the library may reuse its buffer |
| `codefree_comm_ext_sendReadRequest()` | Start a read of 8 bytes into the given structure |
| `codefree_comm_ext_abortTransfer()` | Abort an ongoing transfer (on response timeout) |

On transfer completion or error the port calls `codefree_comm_setStatus(CODEFREE_COMM_STATUS_IDLE)` (write) or `codefree_comm_handleRxComplete()` (read). Blocking ports may do this before returning from the send function.

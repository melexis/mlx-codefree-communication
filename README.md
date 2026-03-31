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
    This library relies on the ARM CMSIS standard for hardware abstraction. Ensure that your project includes the necessary CMSIS core and device-specific header files and libraries compatible with your target microcontroller. The I2C and UART implementations (`codefree_comm_ext_cmsis_i2c_impl.h`, `codefree_comm_ext_cmsis_uart_impl.h`) are built on CMSIS drivers.

3.  **Peripheral Drivers:**
    Beyond CMSIS core, you will need to ensure that your project provides the actual CMSIS-compliant I2C and UART peripheral drivers for your specific microcontroller. These drivers typically come from the chip vendor's Software Pack.

4.  **External Interface (`codefree_comm_external.h`):**
    This header defines the external interface for the communication module. Integrators should include this file to access the library's functions.

5.  **Initialization:**
    Proper initialization of the communication modules (I2C, UART) is critical. Refer to the specific API calls exposed through `codefree_comm_external.h` and ensure all configuration settings in `codefree_comm_config.h` are correctly applied before attempting any communication.

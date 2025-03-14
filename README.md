# Description

This repository contains the **S2LP** RF transceiver driver.

# Dependencies

The driver relies on:

* An external `types.h` header file defining the **standard C types** of the targeted MCU.
* The **embedded utility functions** defined in the [embedded-utils](https://github.com/Ludovic-Lesur/embedded-utils) repository.

Here is the versions compatibility table:

| **s2lp-driver** | **embedded-utils** |
|:---:|:---:|
| [sw1.2](https://github.com/Ludovic-Lesur/s2lp-driver/releases/tag/sw1.2) | >= [sw5.0](https://github.com/Ludovic-Lesur/embedded-utils/releases/tag/sw5.0) |
| [sw1.1](https://github.com/Ludovic-Lesur/s2lp-driver/releases/tag/sw1.1) | >= [sw1.3](https://github.com/Ludovic-Lesur/embedded-utils/releases/tag/sw1.3) |
| [sw1.0](https://github.com/Ludovic-Lesur/s2lp-driver/releases/tag/sw1.0) | >= [sw1.3](https://github.com/Ludovic-Lesur/embedded-utils/releases/tag/sw1.3) |

# Compilation flags

| **Flag name** | **Value** | **Description** |
|:---:|:---:|:---:|
| `S2LP_DRIVER_DISABLE_FLAGS_FILE` | `defined` / `undefined` | Disable the `s2lp_driver_flags.h` header file inclusion when compilation flags are given in the project settings or by command line. |
| `S2LP_DRIVER_DISABLE` | `defined` / `undefined` | Disable the S2LP driver. |
| `S2LP_DRIVER_SPI_ERROR_BASE_LAST` | `<value>` | Last error base of the low level SPI driver. |
| `S2LP_DRIVER_DELAY_ERROR_BASE_LAST` | `<value>` | Last error base of the low level delay driver. |
| `S2LP_DRIVER_XO_FREQUENCY_HZ` | `<value>` | Oscillator frequency in Hz. |
| `S2LP_DRIVER_TX_ENABLE` | `defined` / `undefined` | Enable radio transmission functions. |
| `S2LP_DRIVER_RX_ENABLE` | `defined` / `undefined` | Enable radio reception functions. |

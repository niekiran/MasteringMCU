# MasteringMCU

This repository contains practice code and driver development work from the **Mastering Microcontroller and Embedded Driver Development** course offered by **FastBit Embedded Brain Academy** on Udemy.

## Project Structure

```text
MasteringMCU/
├── Arduino/
│   ├── I2C/
│   │   ├── 001I2CSlaveRxString/
│   │   ├── 002I2CSlaveTxString/
│   │   ├── 003I2CMasterRxString/
│   │   └── 003I2CMasterRxStringLen/
│   ├── SPI/
│   │   ├── 001SPISlaveRxString/
│   │   ├── 002SPISlaveCmdHandling/
│   │   └── 003SPISlaveUartReadOverSPI/
│   └── USART/
│       ├── 001UARTRxString/
│       └── 002UARTTxString/
├── STM32/
│   ├── Apps/        # Application examples (GPIO, SPI, I2C, UART)
│   ├── Drivers/     # STM32F407 peripheral drivers (Inc/Src)
│   ├── BSP/         # Board support code (LCD, DS1307, etc.)
│   ├── System/      # Startup, syscalls, low-level system files
│   ├── .project     # Eclipse/CubeIDE project metadata
│   └── .cproject    # Eclipse/CubeIDE build metadata
└── README.md
```

## Reference Documents

| Document | Link |
|---|---|
| STM32F407 Datasheet | [https://www.st.com/resource/en/datasheet/stm32f405rg.pdf](https://www.st.com/resource/en/datasheet/stm32f405rg.pdf) |
| STM32F407 Reference Manual | [https://www.st.com/resource/en/reference_manual/dm00031020-stm32f405-415-stm32f407-417-stm32f427-437-and-stm32f429-439-advanced-arm-based-32-bit-mcus-stmicroelectronics.pdf](https://www.st.com/resource/en/reference_manual/dm00031020-stm32f405-415-stm32f407-417-stm32f427-437-and-stm32f429-439-advanced-arm-based-32-bit-mcus-stmicroelectronics.pdf) |
| STM32F407 Discovery User Manual | [https://www.st.com/resource/en/user_manual/um1472-discovery-kit-with-stm32f407vg-mcu-stmicroelectronics.pdf](https://www.st.com/resource/en/user_manual/um1472-discovery-kit-with-stm32f407vg-mcu-stmicroelectronics.pdf) |
| STM32F407 Discovery Schematics | [https://www.st.com/resource/en/schematic_pack/mb997-f407vgt6-e01_schematic.pdf](https://www.st.com/resource/en/schematic_pack/mb997-f407vgt6-e01_schematic.pdf) |
| ARM Cortex-M4 Generic User Guide | [https://developer.arm.com/documentation/dui0553/latest/](https://developer.arm.com/documentation/dui0553/latest/) |
| ARM Cortex-M4 Technical Reference Manual | [https://developer.arm.com/documentation/ddi0439/b/](https://developer.arm.com/documentation/ddi0439/b/) |

## Setup Guide

### 1. Prerequisites

- Laptop with macOS, Linux, or Windows
- [Git](https://git-scm.com/)
- [STM32CubeIDE](https://www.st.com/en/development-tools/stm32cubeide.html)
- STM32F407 Discovery board (recommended)
- USB cable (data cable, not charge-only)
- Arduino IDE (for Arduino examples)
- Optional: Logic analyzer for protocol debugging (SPI/I2C/UART)

### 2. Clone the Repository

```bash
git clone <your-repo-url>
cd MasteringMCU
```

### 3. Open and Run STM32 Project (STM32CubeIDE)

1. Open STM32CubeIDE.
2. Go to `File -> Import -> Existing Projects into Workspace`.
3. Select the `STM32/` folder from this repository.
4. Build the project (`Project -> Build Project`).
5. Connect the STM32F407 Discovery board via ST-LINK USB.
6. Flash and debug using `Run` or `Debug`.

Notes:
- Source examples are in `STM32/Apps/`.
- Peripheral drivers are in `STM32/Drivers/`.

### 4. Open and Run Arduino Examples

1. Open Arduino IDE.
2. Navigate to any `.ino` sketch inside `Arduino/`.
3. Select your board and COM/serial port.
4. Upload and monitor serial output from Serial Monitor.

# STM32 Based 6-Channel High-Precision PWM Driver Board 🚀

![Project Status](https://img.shields.io/badge/Status-Completed-success)
![Platform](https://img.shields.io/badge/Platform-STM32-blue)
![License](https://img.shields.io/badge/License-MIT-green)

## 📖 About the Project
This project was developed within the scope of the **"Engineering Design-1"** course at **Kocaeli University, Department of Electronics and Communication Engineering**.

The board is designed to control **Servo Motors**, **ESCs (Electronic Speed Controllers)**, and **High-Power LEDs** for robotic systems and industrial automation applications[cite: 24]. Unlike standard software-based drivers, this board generates **jitter-free**, stable PWM signals using **Hardware Timers**, independent of the main processor (e.g., Raspberry Pi or PC).

## ✨ Key Features
* **Microcontroller:** STM32F103C8T6 (ARM Cortex-M3 @ 72MHz)[cite: 38, 45].
* **Channel Capacity:** 6 Independent PWM Output Channels with 1µs precision[cite: 29, 37].
* **Signal Stability:** Uses hardware **TIM2** and **TIM3** timers to ensure signal integrity without CPU load[cite: 39].
* **Dual Communication Interface:**
    * **UART:** Text-based simple control (e.g., `1:1500`)[cite: 58].
    * **I2C:** Addressable Slave mode for multi-device systems (Address: `0x10`)[cite: 59].
* **Power Management:**
    * Isolated power architecture for Servo/ESC noise reduction[cite: 31].
    * Onboard **LM7805** (5V) and **LM1117** (3.3V) voltage regulators[cite: 46].
* **Safety:** Software-imposed signal limits (1000µs - 2000µs) to prevent mechanical damage[cite: 80].

## 🛠️ Hardware Architecture
The PCB was designed using **Altium Designer** as a 2-layer board, prioritizing signal integrity and power stability[cite: 53, 624].

### Pin Configuration
| Channel | STM32 Pin | Timer | Function |
| :--- | :--- | :--- | :--- |
| **CH1** | PA0 | TIM2_CH1 | Servo / ESC / LED |
| **CH2** | PA1 | TIM2_CH2 | Servo / ESC / LED |
| **CH3** | PA2 | TIM2_CH3 | Servo / ESC / LED |
| **CH4** | PA6 | TIM3_CH1 | Servo / ESC / LED |
| **CH5** | PA7 | TIM3_CH2 | Servo / ESC / LED |
| **CH6** | PB0 | TIM3_CH3 | Servo / ESC / LED |

[cite_start]*(See [cite: 157, 161, 163, 167, 169, 170] for mapping)*

`![PCB 3D View](images/pcb_3d.png)`

## 💻 Software & Algorithm
The firmware is developed in **STM32CubeIDE** using **C** and **HAL Libraries**[cite: 53, 74]. [cite_start]The system operates on a fully **Interrupt-driven** architecture to prevent data loss during high-speed communication.

### Data Processing Flow
1.  **Input:** Data packet arrives via UART or I2C, triggering an Interrupt[cite: 77].
2.  **Parsing:** The incoming data is parsed into `Channel ID` and `Pulse Duration`[cite: 479].
3.  **Safety Check:** The pulse value is validated against the safe range (1000µs - 2000µs). [cite_start]Out-of-bounds values are clamped[cite: 80].
4.  **Hardware Output:** The validated value is written to the Timer Capture/Compare Register (CCR) to generate the signal[cite: 61].

## 📡 Communication Protocols

### 1. UART Control
Send commands via Serial Port (**Baudrate: 115200**) in the following format[cite: 435]:

**Format:** `Channel:Pulse_Duration` + `\n` (Enter)

**Examples:**
* `1:1500` -> Move Channel 1 to Center (90 degrees).
* `3:2000` -> Move Channel 3 to Max Right (180 degrees).
* `6:1000` -> Move Channel 6 to Max Left (0 degrees).

### 2. I2C Control
Master device sends 3 Bytes to Slave Address `0x10`:
* **Byte 1:** Channel Number (1-6)
* **Byte 2:** Pulse Value (High Byte)
* **Byte 3:** Pulse Value (Low Byte)

## 📂 File Structure
* `/Hardware`: Altium Designer Project Files (Schematic, PCB, Gerber)
* `/Software`: STM32CubeIDE Source Codes (main.c, stm32f1xx_it.c)
* `/Docs`: Project Report and Datasheets

## 👨‍💻 Author
**Salim Hamza AŞÇI**
* **Kocaeli University** - Electronics and Communication Engineering
* **LinkedIn:** www.linkedin.com/in/salimhamzaasci

---
*This project is open-source and developed for educational purposes.*

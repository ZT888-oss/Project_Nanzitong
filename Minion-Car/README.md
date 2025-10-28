# Bluetooth Motor Controller 

A Bluetooth-controlled motor system using the **Adafruit Motor Shield** and **Bluefruit SPI module**.  
This project allows real-time remote control of four DC motors using a Bluetooth-enabled device.

---

## Overview
This project demonstrates how to control multiple DC motors through a Bluetooth connection.  
It uses an **Arduino** with an **Adafruit Motor Shield** and an **Adafruit Bluefruit LE SPI module** to receive commands and operate the motors in different modes.

---

## Features
- Control four DC motors individually or in combination  
- Bluetooth commands for forward, backward, left, and right movement  
- Multiple operating modes:
  - **Manual mode:** Control motors directly with button commands  
  - **Sensor-assisted mode:** Motors adjust speed based on sensor inputs  
  - **Stop mode:** Motors are released and stopped safely  
- Real-time feedback from sensors to adjust movement  

---

## Hardware Components
- Arduino (Uno, Mega, or compatible)  
- Adafruit Motor Shield v2  
- Four DC motors  
- Adafruit Bluefruit LE SPI module  
- Digital sensors (for sensor-assisted mode)  
- Connecting wires and power supply  

---

## How It Works
1. **Bluetooth Setup:**  
   - The Bluefruit LE SPI module receives data from a paired device.  
   - The Arduino reads the incoming commands to determine motor actions.  

2. **Motor Control:**  
   - Motors are connected to the Adafruit Motor Shield.  
   - Commands from the Bluetooth module set motor speed and direction.  

3. **Modes of Operation:**  
   - **Manual Mode (Mode 4):** Use buttons to move forward, backward, left, or right.  
   - **Sensor Mode (Mode 2):** Motor behavior adjusts automatically based on sensor inputs.  
   - **Stop Mode (Mode 0):** Motors stop safely when no command is active.  

4. **Loop Execution:**  
   - Continuously read Bluetooth commands and sensor values  
   - Update motor speeds and directions in real-time  

---

## Controls
| Button Command | Action       |
|----------------|-------------|
| 5              | Move Forward|
| 6              | Move Back   |
| 7              | Turn Left   |
| 8              | Turn Right  |

---

## Wiring
- **Motors:** Connect to Motor Shield terminals A, B, C, D  
- **Bluetooth:** SPI interface pins  
  - CS → Pin 8  
  - IRQ → Pin 7  
  - RST → Pin 4  
- **Sensors:** Connect to digital pins 5, 6, 9  

---

## Code Highlights
- `Adafruit_MotorShield` library for motor control  
- `Adafruit_BluefruitLE_SPI` library for Bluetooth communication  
- Multi-mode operation: manual control, sensor-assisted, stop mode  
- Real-time motor speed and direction adjustment  
- Safe motor shutdown using `RELEASE` command  

---

## Getting Started
1. Install **Arduino IDE** and required libraries:  
   - `Adafruit_MotorShield`  
   - `Adafruit_BluefruitLE_SPI`  

2. Connect motors, sensors, and Bluetooth module to Arduino  

3. Upload the provided Arduino code  

4. Pair your Bluetooth device and start sending commands  

---

## Skills Demonstrated
- Arduino programming with hardware libraries  
- Bluetooth communication using SPI protocol  
- Motor control logic and PWM speed management  
- Real-time sensor feedback integration  
- Modular and maintainable embedded C/C++ code  

---


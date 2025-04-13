# Embedded EV Crash Detection & Safety System

This project is a hardware-based embedded system designed to detect potential electric vehicle (EV) crashes and ensure battery and motor health using various sensors. Built using the PIC16F877A microcontroller, this safety system performs voltage, temperature, and motor health checks to enhance EV safety.

## 🚗 Project Overview

The system monitors:

- **Battery Voltage** using a **ZMPT101B sensor**
- **Battery Temperature** using an **LM35 temperature sensor**
- **Motor Current** using an **ACS712 sensor**
- **Motor control and testing** using an **L293D Motor Driver**

When the **crash button is pressed**, the system checks:

1. Battery voltage and temperature
2. If values exceed safe thresholds, it triggers an alert and stops the vehicle
3. If battery is healthy, it tests the motor sequentially and checks the current consumption to identify faults

## 📦 Components Used

| Component           | Purpose                        |
| ------------------- | ------------------------------ |
| PIC16F877A          | Main microcontroller           |
| 0-25 Voltage Sensor | Voltage sensing                |
| LM35                | Temperature sensing            |
| ACS712              | Current sensing                |
| L293D               | Dual motor driver              |
| 12V DC Motor        | Simulating EV wheels           |
| 16x2 LCD            | Status display                 |
| Pushbutton          | Simulates crash detection      |
| Buzzer              | Audio alert on fault detection |
| 12V Battery         | Power supply for motors        |

## 🔧 Features

- Real-time battery health monitoring
- Motor functionality testing
- Overvoltage and overtemperature protection
- LCD user interface
- Fault indication via buzzer

## 🧠 How it Works

1. **Crash Detected (Button Pressed)**

   - Read battery voltage & temperature
   - If voltage > 15V or < 5V , temperature > 60°C → `Battery Damaged`
   - Else → `Battery OK` → Proceed to motor tests

2. **Motor Testing**

   - Test motor via L293D
   - Measure current using ACS712
   - If current < or > threshold → `Motor Fault` (buzzer + LCD alert)

## 📁 Repository Contents

- `schematic.pdf` – Circuit schematic
- `project_report.docx` – Full documentation
- `code.c` – Embedded C code for PIC16F877A
- `test_videos/` – Demo and testing videos
- `sensor_data.csv` – Recorded sensor data during testing
- `README.md` – Project description (this file)

Built with ❤️ by Vidhyarth


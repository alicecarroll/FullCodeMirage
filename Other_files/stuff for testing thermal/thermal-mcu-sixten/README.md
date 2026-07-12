# Thermal MCU V3: Multi-Channel Temperature Controller

Thermal MCU V3 is a multi-threaded, robust, and highly configurable 8-channel temperature controller firmware built on the **Arduino Nano ESP32 (ESP32-S3)**. It is designed to run as an I2C slave device, managing 8 MOSFET driver channels to regulate temperature via heating or cooling elements. 

The firmware utilizes FreeRTOS tasks to achieve real-time deterministic behavior, segregating high-speed communications, control loop calculations, and software-based PWM generation across multiple tasks and cores.

---

## 📖 Table of Contents
1. [System Overview](#1-system-overview)
2. [Hardware Mapping](#2-hardware-mapping)
3. [Software & Thread Architecture](#3-software--thread-architecture)
4. [I2C Communication Protocol](#4-i2c-communication-protocol)
5. [Control Loop Algorithms](#5-control-loop-algorithms)
6. [Software PWM Pin Driver](#6-software-pwm-pin-driver)
7. [Safety Protections & Watchdogs](#7-safety-protections--watchdogs)
8. [Non-Volatile Storage (NVS)](#8-non-volatile-storage-nvs)
9. [Diagnostic Serial CLI](#9-diagnostic-serial-cli)
10. [Master-Side Integration & Verification](#10-master-side-integration--verification)
11. [Build & Installation Guide](#11-build--installation-guide)

---

## 1. System Overview
The Thermal MCU controller acts as an intelligent peripheral in a larger embedded system. A master processor reads physical sensors and periodically sends the current temperatures to the Thermal MCU over I2C. The Thermal MCU then:
* Calculates the necessary output power using configured algorithms (**PID**, **Hysteresis**, or **Manual Override**).
* Generates low-frequency, high-precision software PWM signals to switch MOSFET gates.
* Returns safety status flags, active modes, and calculated power levels back to the master.
* Provides a diagnostic serial CLI for tuning, configuration, and monitoring.

---

## 2. Hardware Mapping
The 8 MOSFET gate driver channels are assigned to physical GPIO pins on the Arduino Nano ESP32 (ESP32-S3):

| Channel ID | Default Channel Name | Arduino Board Pin | Microcontroller GPIO |
|:----------:|:--------------------:|:-----------------:|:--------------------:|
| **Ch 0**   | `sdcrd`              | `D2`              | GPIO5                |
| **Ch 1**   | `P_ch`               | `D3`              | GPIO6                |
| **Ch 2**   | `outl`               | `D4`              | GPIO7                |
| **Ch 3**   | `INlt 1`             | `D5`              | GPIO8                |
| **Ch 4**   | `INLT2`              | `D6`              | GPIO9                |
| **Ch 5**   | `PLT1`               | `D7`              | GPIO10               |
| **Ch 6**   | `PLT2`               | `D8`              | GPIO17               |
| **Ch 7**   | `BCKUP`              | `D9`              | GPIO18               |

*All control pins are configured as `OUTPUT` on boot and initialized to `LOW` (MOSFETs disabled) for safety.*

---

## 3. Software & Thread Architecture
The ESP32-S3 is a dual-core processor. The firmware leverages **FreeRTOS** to distribute tasks and prioritize time-critical operations:

```
  ┌─────────────────────────────────┐      ┌─────────────────────────────────┐
  │             CORE 0              │      │             CORE 1              │
  ├─────────────────────────────────┤      ├─────────────────────────────────┤
  │                                 │      │ ┌─────────────────────────────┐ │
  │                                 │      │ │     pwmTask (10Hz / #3)     │ │
  │ ┌─────────────────────────────┐ │      │ └──────────────┬──────────────┘ │
  │ │      I2C ISR Callbacks      │ │      │                │ Reads Power    │
  │ └──────────────┬──────────────┘ │      │ ┌──────────────▼──────────────┐ │
  │                │ Read/Write     │      │ │    controlTask (1Hz / #2)   │ │
  │                │                │      │ └──────────────┬──────────────┘ │
  │ ┌──────────────▼──────────────┐ │      │                │ Calculates     │
  │ │    Shared Channel State     │◄┼──────┼────────────────┘ Power          │
  │ │     (Mutex-Protected)       │ │      │ ┌─────────────────────────────┐ │
  │ └─────────────────────────────┘ │      │ │      cliTask (Idle / #1)    │ │
  │                                 │      │ └─────────────────────────────┘ │
  └─────────────────────────────────┘      └─────────────────────────────────┘
```

### FreeRTOS Tasks
1. **I2C ISR Handler (`i2cTask`)** — Runs on **Core 0** (inherent to ESP32 Arduino Core I2C Stack):
   * Triggered by hardware I2C write (`onReceive`) and read (`onRequest`) interrupts.
   * Performs CRC-8 validation on incoming bytes.
   * Locks the individual channel mutex to update state parameters or format response packets.
2. **Control Loop Task (`controlTaskFunc`)** — Runs on **Core 1** at priority level `2` (1 Hz):
   * Calculates control outputs (PID outputs, Hysteresis toggles, or Manual output duty cycles).
   * Runs safety guards (thermal runaway check, communication watchdog, sensor validity bounds).
3. **PWM Generation Task (`pwmTaskFunc`)** — Runs on **Core 1** at priority level `3` (10 Hz / 100 ms):
   * Operates at the highest priority on Core 1 to ensure low-jitter software PWM.
   * Manages cycle times (10s to 60s periods) and turns physical GPIOs `HIGH`/`LOW` based on target duty cycle.
4. **CLI Task (`cliTaskFunc`)** — Runs on **Core 1** at priority level `1` (Event-driven):
   * Polls USB Serial for user inputs and executes configuration/monitoring commands.

### Thread Safety (Mutexes)
To prevent torn reads/writes of 16-bit variables (like temperature values) between the fast asynchronous I2C interrupt routine and the 1 Hz control/10 Hz PWM tasks, **8 independent Mutex Semaphores** (`channelMutexes[8]`) protect each channel's state structure. 
* To prevent blocking, tasks try to acquire the mutex with a small timeout (e.g., 2ms for PWM, 5ms for I2C ISR) and fall back safely to cached/default states if blocked, avoiding high-priority thread starvation.

---

## 4. I2C Communication Protocol
The controller acts as an I2C slave responding to address **`0x10`** (default). 

All interactions consist of a **Write-then-Read** transaction: the Master writes a command packet (8 bytes) to a channel and immediately reads a status packet (8 bytes) back.

### A. Master Write Packet (8 Bytes)
Sent from the Master to configure or update a channel's status:

| Byte | Field | Type | Description |
|:---:|:---|:---:|:---|
| **0** | Command / Register | `uint8_t` | Command index: `0x01` = Update/Request Channel State |
| **1** | Channel ID | `uint8_t` | Destination channel: `0x00` to `0x07` |
| **2** | Mode / Power | `uint8_t` | Operating Mode setting: <br>`0` = Hysteresis mode<br>`1` = PID mode<br>`155` to `255` = Manual Mode (`0%` to `100%` power) |
| **3** | Current Temp MSB | `int8_t` | High byte of 16-bit signed integer representing current temperature (Scaled by 100) |
| **4** | Current Temp LSB | `uint8_t` | Low byte of 16-bit signed integer representing current temperature (Scaled by 100) |
| **5** | Target Temp MSB | `int8_t` | High byte of 16-bit signed integer representing target temperature setpoint (Scaled by 100) |
| **6** | Target Temp LSB | `uint8_t` | Low byte of 16-bit signed integer representing target temperature setpoint (Scaled by 100) |
| **7** | Checksum | `uint8_t` | CRC-8 checksum of Bytes 0 through 6 |

*Note: In Manual Mode, the manual power duty cycle percentage is extracted as: `Power = Mode - 155`.*

### B. Master Read Packet (8 Bytes)
Returned by the Thermal MCU to report current telemetry:

| Byte | Field | Type | Description |
|:---:|:---|:---:|:---|
| **0** | Channel ID | `uint8_t` | Active channel: `0x00` to `0x07` |
| **1** | Active Mode | `uint8_t` | `0` = Hysteresis mode, `1` = PID mode, `2` = Manual mode |
| **2** | Output Power | `uint8_t` | Calculated power duty cycle: `0` to `100` (%) |
| **3** | Target Temp MSB | `int8_t` | High byte of active 16-bit signed target setpoint (Scaled by 100) |
| **4** | Target Temp LSB | `uint8_t` | Low byte of active 16-bit signed target setpoint (Scaled by 100) |
| **5** | Status Byte | `uint8_t` | Bitmask of safety, warning, and error flags (see [Safety Protections](#7-safety-protections--watchdogs)) |
| **6** | Padding | `uint8_t` | Reserved (always `0x00`) |
| **7** | Checksum | `uint8_t` | CRC-8 checksum of Bytes 0 through 6 |

### C. The "Bad CRC" Transaction Guard
To protect the system against electrical noise on the SDA/SCL lines, a hardware transaction guard is implemented:
1. If the calculated CRC-8 in the write handler does not match Byte 7, the data is discarded, and the global pointer `lastAddressedChannel` is set to `0xFF`.
2. When the Master immediately calls the read request, the slave detects `lastAddressedChannel == 0xFF` and sends a **CRC Error Packet**:
   * `Byte 0 = 0xFF` (Invalid Channel ID)
   * `Byte 1 = 0xFF` (Invalid Mode)
   * `Byte 2-4 = 0`
   * `Byte 5 = 0x08` (`FLAG_CRC_ERROR` bit active)
   * `Byte 6 = 0`
   * `Byte 7 = CRC-8 checksum`
3. Upon receiving this CRC Error packet, the Master can log the error and immediately retry the write-then-read sequence.

### D. CRC-8 Algorithm
The CRC-8 checksum is calculated using the **SMBus polynomial** $x^8 + x^2 + x + 1$ (`0x07`), initialized to `0x00`:
```cpp
uint8_t calculateCRC8(const uint8_t *data, size_t length) {
    uint8_t crc = 0x00;
    for (size_t i = 0; i < length; i++) {
        crc ^= data[i];
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x80) {
                crc = (crc << 1) ^ 0x07;
            } else {
                crc <<= 1;
            }
        }
    }
    return crc;
}
```

---

## 5. Control Loop Algorithms
The `controlTaskFunc` calculates the output power ($0\%$ to $100\%$) for each channel once every second.

### Hysteresis Mode
Hysteresis provides a simple on/off window (deadband) centered around the target setpoint to prevent rapid switching. The deadband width is configured via `hystDelta` (scaled by 100).

* **Heater Configuration (`isCooler == false`)**:
  * Turns **ON** ($100\%$): if $\text{currentTemp} < \text{targetTemp} - \text{hystDelta}$
  * Turns **OFF** ($0\%$): if $\text{currentTemp} > \text{targetTemp} + \text{hystDelta}$
* **Cooler Configuration (`isCooler == true`)**:
  * Turns **ON** ($100\%$): if $\text{currentTemp} > \text{targetTemp} + \text{hystDelta}$
  * Turns **OFF** ($0\%$): if $\text{currentTemp} < \text{targetTemp} - \text{hystDelta}$

### PID Mode
The PID loop calculates output power using Proportional, Integral, and Derivative coefficients. It operates on temperature values scaled back to float values ($T_{\text{Celsius}} = \frac{T_{\text{Scaled}}}{100.0}$).

1. **Error Calculation**:
   * Heater: $\text{error} = T_{\text{target}} - T_{\text{current}}$
   * Cooler: $\text{error} = T_{\text{current}} - T_{\text{target}}$
2. **Derivative-on-Measurement (No Derivative Kick)**:
   To prevent huge output spikes (derivative kick) when target setpoints are suddenly changed, the derivative term calculates the rate of change of the *measured temperature* rather than the error:
   * Heater: $D = -K_d \times \frac{T_{\text{current}} - T_{\text{previous}}}{\Delta t}$
   * Cooler: $D = K_d \times \frac{T_{\text{current}} - T_{\text{previous}}}{\Delta t}$
3. **Integral Term Accumulation & Clamping (Anti-Windup)**:
   To prevent the integrator from saturating (winding up) when the output is pegged at $0\%$ or $100\%$, clamping is used:
   * Calculate Proportional term: $P = K_p \times \text{error}$
   * Calculate candidate Integral term: $I_{\text{candidate}} = \text{integralError} + K_i \times \text{error}$
   * If the final PID sum ($P + I_{\text{candidate}} + D$) exceeds $100\%$ or falls below $0\%$, the output power is clamped, and the candidate integral term is **discarded** (unless the error is moving back toward the active range).

---

## 6. Software PWM Pin Driver
Because thermal systems are slow, they do not require high-frequency PWM (which would cause excessive electromagnetic interference and stress on MOSFETs). Instead, the controller uses low-frequency software PWM driven at a **10 Hz** update rate (every $100\text{ ms}$).

### Mechanism
* **PWM Period**: Configurable from **10 to 60 seconds** (`pwmPeriodMs`) per channel.
* **Resolution**: $100\text{ ms}$ increments (e.g., $1\%$ duty cycle on a 10s period corresponds to $100\text{ ms}$ ON time).
* **Execution**:
  1. Calculate active ON time: $\text{onDurationMs} = \frac{\text{PeriodMs} \times \text{outputPower}}{100}$
  2. Track cycle time: $\text{elapsed} = \text{now} - \text{cycleStart}$
  3. If $\text{elapsed} \ge \text{PeriodMs}$, reset $\text{cycleStart} = \text{now}$.
  4. Write physical pin `HIGH` if $\text{elapsed} < \text{onDurationMs}$, otherwise write `LOW`.

---

## 7. Safety Protections & Watchdogs
Each channel monitors its operation independently and reports faults through bits in the status byte:

```
┌───────┬───────┬───────┬───────┬───────────────┬───────────────┬───────────────┬───────────────┐
│ Bit 7 │ Bit 6 │ Bit 5 │ Bit 4 │     Bit 3     │     Bit 2     │     Bit 1     │     Bit 0     │
├───────┼───────┼───────┼───────┼───────────────┼───────────────┼───────────────┼───────────────┤
│ Rsrvd │ Rsrvd │ Rsrvd │ Rsrvd │  FLAG_CRC_ERR │ FLAG_SENS_ERR │ FLAG_RUNAWAY  │ FLAG_TIMEOUT  │
│ (0x00)│ (0x00)│ (0x00)│ (0x00)│    (0x08)     │    (0x04)     │    (0x02)     │    (0x01)     │
└───────┴───────┴───────┴───────┴───────────────┴───────────────┴───────────────┴───────────────┘
```

### A. I2C Communication Watchdog (`FLAG_COMM_TIMEOUT` - `0x01`)
* **Trigger**: Triggers if no valid I2C Write command has been received for a channel within its configured communication timeout (default: **30 seconds**).
* **Action**: Sets the timeout bit. If the channel is in a closed-loop mode (`PID` or `Hysteresis`), the output power is **forced to 0%** immediately. In `Manual` mode, the flag is raised but the output power is *not* cut (bypassed for testing purposes).
* **Clear**: Clears immediately upon receiving a valid, checksum-passed I2C write.

### B. Thermal Runaway Protection (`FLAG_THERMAL_RUNAWAY` - `0x02`)
* **Trigger**: Triggers if output power is saturated ($\ge 95\%$) for **60 seconds**, but the temperature does not respond.
  * Heater: Temp does not rise by at least **$0.50^\circ\text{C}$** over the 60s period.
  * Cooler: Temp does not fall by at least **$0.50^\circ\text{C}$** over the 60s period.
* **Action**: Sets the runaway flag. Note: The master should poll this flag and take system-level shutdown actions.
* **Clear**: Clears when output power drops below $50\%$.

### C. Sensor Failure Watchdog (`FLAG_SENSOR_ERROR` - `0x04`)
* **Trigger**: Triggers if the master reports a temperature reading out of safe range ($< -50.00^\circ\text{C}$ or $> 250.00^\circ\text{C}$), or passes the sensor fault sentinel value (`-9999`).
* **Action**: Sets the sensor error flag. If the channel is in closed-loop control (`PID` or `Hysteresis`), output power is **forced to 0%** and PID accumulators are reset. Manual mode bypasses the shutdown to allow diagnostic testing.

---

## 8. Non-Volatile Storage (NVS)
Configurations are persisted across reboots using the ESP32 **Preferences** library. This writes parameters directly to the flash memory inside the `"thermal"` namespace.

* **Loaded Parameters**: On startup, each channel's type (heater/cooler), PID constants ($K_p, K_i, K_d$), hysteresis window delta, and communication timeout are loaded.
* **Fallback Defaults**: If configuration keys are missing or size-corrupted on boot, defaults are loaded:
  * Heating mode (`isCooler = false`)
  * 20-second PWM period
  * $K_p = 2.0, K_i = 0.1, K_d = 1.0$
  * Hysteresis window = $\pm 0.50^\circ\text{C}$
  * Comm timeout = 30 seconds
* **NVS Commands**: You can read configurations or manually save live tunings via the [CLI commands](#9-diagnostic-serial-cli).

---

## 9. Diagnostic Serial CLI
The Thermal MCU includes a USB Serial-based command-line interface running at **115200 baud**. 

### Commands List
| Command | Arguments | Description |
|:---|:---|:---|
| **`help`** | None | Prints the CLI help menu showing syntax. |
| **`status`** | None | Prints a formatted status table of all 8 channels. |
| **`set`** | `<ch> <m\|b\|p> <val>` | Overrides mode and targets:<br>`m` = Manual (val = 0-100 power)<br>`b` = Hysteresis (val = target temp)<br>`p` = PID (val = target temp) |
| **`tune`** | `<ch> <kp> <ki> <kd>` | Updates the $K_p, K_i,$ and $K_d$ parameters for a channel in RAM. |
| **`config`** | `<ch> <h\|c> <period_s>` | Sets channel type (`h` = heater, `c` = cooler) and PWM period in seconds (10 to 60). |
| **`save`** | None | Writes all current RAM configurations to NVS flash. |
| **`load`** | None | Reloads configurations from NVS flash. |
| **`monitor`** | None | Prints the status table every 500 ms. Press any key to exit. |

### CLI Status Table Output Example
```
-----------------------------------------------------------------------------------------------------------
| Ch | Name    | Pin | Config | Period | Mode | Power | Cur Temp | Tgt Temp | PinState | Status Flags      |
-----------------------------------------------------------------------------------------------------------
| 0  | sdcrd   | D2  | HEATER |  20.0s | PID  |   35% |   34.50C |   35.00C | HIGH     | OK                |
| 1  | P_ch    | D3  | COOLER |  10.0s | MAN  |   50% |   21.00C |    0.00C | LOW      | OK                |
| 2  | outl    | D4  | HEATER |  20.0s | HYST |    0% |   20.50C |   30.00C | LOW      | TIMEOUT           |
| 3  | INlt 1  | D5  | HEATER |  30.0s | PID  |    0% |  -99.99C |   35.00C | LOW      | SENSOR_ERR        |
...
-----------------------------------------------------------------------------------------------------------
```

---

## 10. Master-Side Integration & Verification
To verify the slave, the master device (e.g. another ESP32-S3 or Arduino) must perform I2C Write-then-Read transfers. 

An example test sketch (`main_mcu_test.ino`) is included in the project directory. The communication routine is structured as follows:

```cpp
#include <Wire.h>

#define I2C_SLAVE_ADDR  0x10
#define SDA_PIN         8
#define SCL_PIN         9

void setup() {
    Serial.begin(115200);
    Wire.setPins(SDA_PIN, SCL_PIN);
    Wire.begin();
    Wire.setClock(400000); // 400 kHz Fast Mode
}

void loop() {
    for (uint8_t ch = 0; ch < 8; ch++) {
        int16_t currentTemp = (20 + ch) * 100; // 20.00C + ch
        int16_t targetTemp = 3500;             // 35.00C

        // 1. Structure the 8-byte write packet
        uint8_t writePacket[8];
        writePacket[0] = 0x01; // CMD
        writePacket[1] = ch;   // Channel ID
        writePacket[2] = 205;  // Mode (205 = Manual Mode at 50% power)
        writePacket[3] = (currentTemp >> 8) & 0xFF;
        writePacket[4] = currentTemp & 0xFF;
        writePacket[5] = (targetTemp >> 8) & 0xFF;
        writePacket[6] = targetTemp & 0xFF;
        writePacket[7] = calculateCRC8(writePacket, 7); // CRC checksum

        // 2. Transmit to Slave
        Wire.beginTransmission(I2C_SLAVE_ADDR);
        Wire.write(writePacket, 8);
        uint8_t writeStatus = Wire.endTransmission();
        
        if (writeStatus != 0) {
            Serial.printf("Write error: %d\n", writeStatus);
            continue;
        }

        // Give the slave ISR brief parsing time
        delayMicroseconds(200);

        // 3. Request 8 bytes status back
        uint8_t bytesReceived = Wire.requestFrom(I2C_SLAVE_ADDR, 8);
        if (bytesReceived == 8) {
            uint8_t readPacket[8];
            for (int i = 0; i < 8; i++) {
                readPacket[i] = Wire.read();
            }

            // 4. Validate CRC checksum
            if (calculateCRC8(readPacket, 7) != readPacket[7]) {
                Serial.println("Read CRC Mismatch!");
                continue;
            }

            // Decode packet values
            uint8_t rxChannel = readPacket[0];
            uint8_t rxMode = readPacket[1];
            uint8_t rxPower = readPacket[2];
            int16_t rxTarget = (int16_t)((readPacket[3] << 8) | readPacket[4]);
            uint8_t rxStatus = readPacket[5];
            
            Serial.printf("Ch %d: Power: %d%% | Status Flags: 0x%02X\n", rxChannel, rxPower, rxStatus);
        }
        delay(50);
    }
    delay(2000);
}
```

---

## 11. Build & Installation Guide

### Prerequisites
* **PlatformIO IDE** (VS Code plugin) or PlatformIO Core CLI.
* USB-C cable to connect to the Arduino Nano ESP32.

### Directory Structure
```
├── include/
│   ├── cli.h
│   ├── config.h
│   ├── control.h
│   ├── i2c_comm.h
│   ├── pwm.h
│   └── storage.h
├── src/
│   ├── cli.cpp
│   ├── control.cpp
│   ├── i2c_comm.cpp
│   ├── main.cpp
│   ├── pwm.cpp
│   └── storage.cpp
├── platformio.ini
└── README.md
```

### Building and Uploading via CLI
To build the firmware and upload it to the connected microcontroller:

1. Open your terminal in the project directory.
2. Build the project:
   ```bash
   pio run
   ```
3. Upload the project to the board:
   ```bash
   pio run --target upload
   ```
4. Start the serial monitor to view CLI messages:
   ```bash
   pio device monitor --baud 115200
   ```

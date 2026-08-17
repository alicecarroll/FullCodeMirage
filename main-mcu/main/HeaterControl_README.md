# Heater Control System Documentation

## Overview
The Heater Control System provides a unified interface for managing 8 different heaters on the main MCU, allowing each heater to be independently configured with different control types (Bang-Bang, PID, or Manual), target temperatures, and manual duty cycles.

## Architecture

### Components
1. **HeaterControl.h/.cpp** - Core heater control management library
2. **main.cpp integration** - Command parsing and thermal MCU communication
3. **Thermal MCU communication** - I2C-based control of heater parameters

### Key Data Structures

#### HeaterControlMode (enum)
- `HEATER_MODE_BANGBANG (0)` - Hysteresis/Bang-Bang on/off control
- `HEATER_MODE_PID (1)` - PID closed-loop control
- `HEATER_MODE_MANUAL (200)` - Manual control with fixed duty cycle

#### HeaterConfig (struct)
Stores configuration for a single heater:
- `heater_id` - Heater index (0-7)
- `mode` - Current control mode
- `target_temp` - Target temperature in 0.01°C units (e.g., 5000 = 50.00°C)
- `manual_duty_cycle` - Duty cycle percentage (0-100) for manual mode
- `enabled` - Whether the heater is active

#### HeaterSystem (struct)
Array of 8 HeaterConfig structures for all heaters.

## API Functions

### Initialization
```cpp
void heater_system_init(HeaterSystem* system);
```
Initializes all heaters with default values:
- Mode: Bang-Bang
- Target: 20.00°C
- Enabled: false

### Configuration Management
```cpp
void heater_set_mode(HeaterSystem* system, uint8_t heater_id, HeaterControlMode mode);
void heater_set_target(HeaterSystem* system, uint8_t heater_id, int16_t target_temp);
void heater_set_manual_duty(HeaterSystem* system, uint8_t heater_id, uint8_t duty_cycle);
void heater_set_enabled(HeaterSystem* system, uint8_t heater_id, bool enabled);
HeaterConfig* heater_get_config(HeaterSystem* system, uint8_t heater_id);
```

### Communication
```cpp
bool heater_send_config_to_thermal(uint8_t heater_id, int16_t current_temp);
```
Sends heater configuration to the thermal MCU via I2C.

### Access Global Instance
```cpp
HeaterSystem* heater_system_get_global();
```
Returns the global heater system instance used by main.cpp.

## Ethernet Command Interface

### Configure Heater (Combined Mode + Settings) - RECOMMENDED

**For Bang-Bang and PID modes:**
**Command Format:** `HEATER <id> MODE <mode> TARGET <temperature_C>`

**Parameters:**
- `<id>` - Heater number (1-8)
- `<mode>` - Control mode: `BANGBANG`, `BANG-BANG`, or `PID`
- `<temperature_C>` - Temperature in degrees Celsius (can include decimals)

**Examples:**
```
HEATER 1 MODE PID TARGET 50.5
HEATER 3 MODE BANGBANG TARGET 25
HEATER 7 MODE PID TARGET 30.75
```

**For Manual mode:**
**Command Format:** `HEATER <id> MODE MANUAL DUTY <0-100>`

**Parameters:**
- `<id>` - Heater number (1-8)
- `<0-100>` - Duty cycle percentage

**Examples:**
```
HEATER 2 MODE MANUAL DUTY 75
HEATER 6 MODE MANUAL DUTY 100
HEATER 4 MODE MANUAL DUTY 0
```

### Set Heater Control Mode (Separate)
**Command Format:** `HEATER <id> MODE <mode>`

**Parameters:**
- `<id>` - Heater number (1-8)
- `<mode>` - Control mode: `BANGBANG`, `BANG-BANG`, `PID`, or `MANUAL`

**Examples:**
```
HEATER 1 MODE BANGBANG
HEATER 5 MODE PID
HEATER 8 MODE MANUAL
```

### Set Target Temperature (Separate)
**Command Format:** `HEATER <id> TARGET <temperature_C>`

**Parameters:**
- `<id>` - Heater number (1-8)
- `<temperature_C>` - Temperature in degrees Celsius (can include decimals)

**Examples:**
```
HEATER 1 TARGET 50.5
HEATER 3 TARGET 25
HEATER 7 TARGET 30.75
```

### Set Manual Duty Cycle (Separate)
**Command Format:** `HEATER <id> DUTY <0-100>`

**Parameters:**
- `<id>` - Heater number (1-8)
- `<0-100>` - Duty cycle percentage (only effective when mode is MANUAL)

**Examples:**
```
HEATER 2 DUTY 75
HEATER 6 DUTY 100
HEATER 4 DUTY 0
```

### Enable/Disable Heater (Existing Commands)
**Existing Commands** - These still work but now integrate with the new system:
```
HEATER ON 1        # Enable heater 1
HEATER OFF 1       # Disable heater 1
HEATER ALL ON      # Enable all heaters
HEATER ALL OFF     # Disable all heaters
```

## Usage Example Sequence

### Combined Commands (RECOMMENDED)
To set up Heater 1 for PID control at 50°C:
```
HEATER 1 MODE PID TARGET 50.0
HEATER 1 ON
```

To set up Heater 2 for manual control at 75% duty cycle:
```
HEATER 2 MODE MANUAL DUTY 75
HEATER 2 ON
```

To set up Heater 3 for Bang-Bang control at 30°C:
```
HEATER 3 MODE BANGBANG TARGET 30.0
HEATER 3 ON
```

### Separate Commands (Alternative)
To set up Heater 1 for PID control at 50°C:
```
HEATER 1 MODE PID
HEATER 1 TARGET 50.0
HEATER 1 ON
```

To set up Heater 2 for manual control at 75% duty cycle:
```
HEATER 2 MODE MANUAL
HEATER 2 DUTY 75
HEATER 2 ON
```

To set up Heater 3 for Bang-Bang control at 30°C:
```
HEATER 3 MODE BANGBANG
HEATER 3 TARGET 30.0
HEATER 3 ON
```

## Temperature Scale
All temperature values are sent to the thermal MCU in units of 0.01°C:
- Input: Decimal degrees Celsius (e.g., 50.5)
- Internal representation: Integer (5050)
- Thermal MCU receives: 5050 (meaning 50.50°C)

## Synchronization with Thermal MCU
The `comms_thermal_sensor()` function in main.cpp:
1. Reads current temperatures from all sensors
2. For each enabled heater, sends its configuration to the thermal MCU
3. Includes current temperature and target temperature
4. Transmits the appropriate mode/duty cycle value

The thermal MCU updates each heater's control output based on the received configuration.

## Integration with Existing Code

### Thermal Communication Loop
The modified `comms_thermal_sensor()` function now:
- Iterates through all 8 heaters
- Checks if each heater is enabled
- Retrieves the heater's configuration from the control system
- Sends mode, temperature, and target to thermal MCU
- Automatically handles manual vs. automatic mode conversion

### Heater Bit Operations
The `set_heater_bit()` function now:
- Updates the active heater mask (for telemetry)
- Updates the heater control system enabled state
- Maintains backward compatibility with existing code

## Error Handling
- Invalid heater IDs (outside 0-7 range) are rejected with warnings
- Invalid modes are rejected
- Duty cycle values outside 0-100 are rejected
- All errors are logged via ESP_LOG

## Future Enhancements
Possible future additions:
- Persistent heater configuration storage (NVRAM/SD Card)
- Heater grouping/presets
- Failsafe modes
- Temperature ramping
- Automated mode switching based on system state

# Heater Control Quick Reference Guide

## Command Syntax

### 1. Set Heater Control Mode
```
HEATER <heater_number> MODE <mode_type>
```
- **heater_number**: 1-8
- **mode_type**: BANGBANG | BANG-BANG | PID | MANUAL

**Examples:**
```
HEATER 1 MODE BANGBANG   # ON/OFF control for heater 1
HEATER 2 MODE PID        # PID control for heater 2
HEATER 3 MODE MANUAL     # Manual control for heater 3
```

### 2. Set Target Temperature
```
HEATER <heater_number> TARGET <temperature_celsius>
```
- **heater_number**: 1-8
- **temperature_celsius**: Any decimal value (e.g., 25, 50.5, 100.25)

**Examples:**
```
HEATER 1 TARGET 50       # Set heater 1 to 50°C
HEATER 2 TARGET 30.5     # Set heater 2 to 30.5°C
HEATER 5 TARGET 100      # Set heater 5 to 100°C
```

### 3. Set Manual Duty Cycle
```
HEATER <heater_number> DUTY <percentage>
```
- **heater_number**: 1-8
- **percentage**: 0-100 (only used when mode is MANUAL)

**Examples:**
```
HEATER 3 DUTY 50         # 50% power for heater 3
HEATER 4 DUTY 100        # Full power for heater 4
HEATER 1 DUTY 0          # Off for heater 1
```

### 4. Enable/Disable Heater (Existing Commands)
```
HEATER ON <heater_number>
HEATER OFF <heater_number>
HEATER ALL ON
HEATER ALL OFF
```

**Examples:**
```
HEATER ON 1              # Enable heater 1
HEATER OFF 2             # Disable heater 2
HEATER ALL ON            # Enable all heaters
HEATER ALL OFF           # Disable all heaters
```

## Typical Usage Sequences

### Scenario 1: PID-Controlled Temperature Chamber
```
HEATER 1 MODE PID
HEATER 1 TARGET 50
HEATER 1 ON
HEATER 2 MODE PID
HEATER 2 TARGET 50
HEATER 2 ON
```

### Scenario 2: Manual Power Control
```
HEATER 3 MODE MANUAL
HEATER 3 DUTY 75
HEATER 3 ON
```

### Scenario 3: Bang-Bang Control
```
HEATER 4 MODE BANGBANG
HEATER 4 TARGET 25
HEATER 4 ON
```

### Scenario 4: Disable All Heaters
```
HEATER ALL OFF
```

### Scenario 5: Mixed Configuration
```
# Heater 1-2: PID at 50°C
HEATER 1 MODE PID
HEATER 1 TARGET 50
HEATER 2 MODE PID
HEATER 2 TARGET 50

# Heater 3-4: Bang-Bang at 30°C
HEATER 3 MODE BANGBANG
HEATER 3 TARGET 30
HEATER 4 MODE BANGBANG
HEATER 4 TARGET 30

# Heater 5: Manual at 50%
HEATER 5 MODE MANUAL
HEATER 5 DUTY 50

# Enable all
HEATER ALL ON
```

## Control Mode Selection Guide

### Bang-Bang (Hysteresis) Mode
- **Best for**: Simple ON/OFF control with a temperature deadzone
- **Characteristics**: 
  - Simple, reliable, low CPU usage
  - Creates temperature oscillation around target
  - Good for coarse temperature control
- **Example**:
  ```
  HEATER 1 MODE BANGBANG
  HEATER 1 TARGET 40
  HEATER 1 ON
  ```

### PID Mode
- **Best for**: Precise temperature control with minimal oscillation
- **Characteristics**:
  - More responsive than Bang-Bang
  - Smooth approach to target temperature
  - Requires more CPU/tuning
  - Better for sensitive equipment
- **Example**:
  ```
  HEATER 2 MODE PID
  HEATER 2 TARGET 50.5
  HEATER 2 ON
  ```

### Manual Mode
- **Best for**: Debugging, testing, or manual interventions
- **Characteristics**:
  - Fixed power output
  - User controls exact power level
  - No automatic feedback control
  - Useful for troubleshooting
- **Example**:
  ```
  HEATER 3 MODE MANUAL
  HEATER 3 DUTY 75      # 75% power
  HEATER 3 ON
  ```

## Temperature Guidelines
- **Minimum**: 0°C (can use negative with decimal format)
- **Precision**: 0.01°C (e.g., 25.37°C)
- **Common setpoints**:
  - Ambient: 20-25°C
  - Moderate heating: 30-50°C
  - High heating: 50-100°C
  - Extreme: >100°C (check equipment limits)

## Troubleshooting

### Heater not responding
1. Check enabled status: `HEATER n ON`
2. Verify mode is set correctly
3. Check temperature target is reasonable

### Temperature not reaching target
1. Check control mode (PID or Bang-Bang preferred)
2. Verify duty cycle if in MANUAL mode
3. Check if heater sensor is reading properly

### Oscillating temperature
1. Normal for Bang-Bang mode
2. Try PID mode for smoother control
3. Adjust deadzone in Bang-Bang if available

### Unexpected behavior
1. Verify command syntax matches examples
2. Check heater number is 1-8
3. Confirm mode is spelled correctly (BANGBANG not BANG_BANG)

## Status Monitoring
- Check telemetry for current heater status
- Monitor temperature readings from all sensors
- Track active_heater_mask to see which heaters are enabled
- Review error logs for I2C communication issues

## Safety Notes
⚠️ Always verify:
- Target temperatures are appropriate for your application
- Heaters are turned off before equipment shutdown
- Manual duty cycles don't cause thermal runaway
- I2C connection to thermal MCU is functioning

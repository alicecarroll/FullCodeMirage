#include "main.h"
#include "driver/i2c.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>
#include <stdio.h>

// ------------------------------------------------------------------
// Pin assignments — from the Pressure MCU pinout table.
// Board: ESP32-S3 (IO0-IO48 range, dedicated VUSB/VIN pins).
// ------------------------------------------------------------------

#define VALVE_PIN        GPIO_NUM_9   // IO9  — VALVE1
#define COMPRESSOR_PIN   GPIO_NUM_10  // IO10 — PWM1 (digital on/off for now, real PWM later)
#define PUMP1_PIN        GPIO_NUM_47  // IO47 — PWM2, vacuum pump 1 
#define PUMP2_PIN        GPIO_NUM_38  // IO38 — PWM3, vacuum pump 2

// PDB relay signals — connector pins 1, 4, 5, 28 -> these GPIOs.
// Plain on/off to MOSFETs on the power distribution board, no
// sequencing/timing logic.
#define PDB_RELAY1_PIN   GPIO_NUM_21  // pin 28
#define PDB_RELAY2_PIN   GPIO_NUM_48   // pin 1
#define PDB_RELAY3_PIN   GPIO_NUM_1   // pin 4
#define PDB_RELAY4_PIN   GPIO_NUM_2  // pin 5

// I2C_SDA / I2C_SCL (IO13 / IO14)
#define I2C_SCL_PIN      GPIO_NUM_14
#define I2C_SDA_PIN      GPIO_NUM_13

#define I2C_SLAVE_NUM    I2C_NUM_1
#define I2C_SLAVE_ADDR   0x10

static constexpr uint8_t OPCODE_SENSORS = 0x01;
static constexpr uint8_t OPCODE_COMMANDS = 0x02;
static constexpr uint8_t OPCODE_SETTINGS = 0x03;

// Define the scaling factor used to convert float values to int16_t for transmission
#define SENSOR_SCALE      100.0f

// Packet types
typedef enum
{
    Slave_packet_data     = 0x01,
    Slave_packet_command  = 0x02,
    Slave_packet_setting  = 0x03

} PacketType;

static const char *TAG = "main";

// ------------------------------------------------------------------
// Internal state
// ------------------------------------------------------------------

static PressureStatus status;
static float external_sensors[7] = {};
static bool external_sensors_valid = false;
static uint8_t last_channel_id = 0x00;

static float target_pressure = 3.0f;       // chamber target (air exchange phase)

// Bug 2 fix: the old hardcoded 1.0f and the missing inlet lower limit
// are now named, settable values instead of magic numbers.
static float compressor_inlet_upper_limit = 1.0f;  // ends prepressurisation
static float compressor_inlet_lower_limit = 0.2f;   // ends air exchange — PLACEHOLDER, confirm real value

// Bug 4 fix: simple error codes so pressure_update() can report *why*
// it went into PRESSURE_ERROR.
#define PRESSURE_ERR_NONE          0
#define PRESSURE_ERR_CHAMBER_SENSOR 1
#define PRESSURE_ERR_INLET_SENSOR   2

static void pump1_on();
static void pump1_off();
static void pump2_on();
static void pump2_off();
static void pumps_on();
static void pumps_off();
static void compressor_on();
static void compressor_off();
static void valve_open();
static void valve_close();

static void pressure_update_external_sensors(const float sensors[7])
{
    memcpy(external_sensors, sensors, sizeof(external_sensors));
    external_sensors_valid = true;

    // Reuse received main-MCU sensor data when it is available.
    // Pp2 is the chamber measurement and Pp1 is the inlet-side pipe
    // pressure that best matches the compressor inlet use in this state
    // machine.
    status.chamber_pressure = sensors[2];
    status.compressor_inlet_pressure = sensors[1];
}

static void pressure_set_valve_override(bool open)
{
    if (open)
    {
        valve_open();
    }
    else
    {
        valve_close();
    }
}

static void pressure_set_vpump1_pwm(uint8_t pwm)
{
    // The hardware is still driven as plain digital outputs, but the
    // packet values are now consumed instead of being dropped on the floor.
    pump1_on();
    
}

static void pressure_set_vpump2_pwm(uint8_t pwm)
{
    // The hardware is still driven as plain digital outputs, but the
    // packet values are now consumed instead of being dropped on the floor.
    pump2_on();
    
}

static void pressure_set_compressor_pwm(uint8_t pwm)
{
    // The hardware is still driven as plain digital outputs, but the
    // packet values are now consumed instead of being dropped on the floor.
    compressor_on();
    
}
// ------------------------------------------------------------------
// Internal hardware functions
//
// pumps_on()/pumps_off() drive both vacuum pumps together, since the
// state machine's logic still treats "the pumps" as one concept (both
// on during prepressurisation, both off otherwise). pump1/pump2 are
// also exposed individually in case one needs to be driven separately
// later (e.g. for fault isolation or staggered start).
// ------------------------------------------------------------------

static void pump1_on()
{
    gpio_set_level(PUMP1_PIN, 1);
}

static void pump1_off()
{
    gpio_set_level(PUMP1_PIN, 0);
}

static void pump2_on()
{
    gpio_set_level(PUMP2_PIN, 1);
}

static void pump2_off()
{
    gpio_set_level(PUMP2_PIN, 0);
}

static void pumps_on()
{
    pump1_on();
    pump2_on();
}

static void pumps_off()
{
    pump1_off();
    pump2_off();
}

static void compressor_on()
{
    // PWM1 driven as plain digital HIGH for now — swap to real PWM
    // (ledc driver) once a duty cycle is decided.
    gpio_set_level(COMPRESSOR_PIN, 1);
}

static void compressor_off()
{
    gpio_set_level(COMPRESSOR_PIN, 0);
}

static void valve_open()
{
    gpio_set_level(VALVE_PIN, 1);
}

static void valve_close()
{
    gpio_set_level(VALVE_PIN, 0);
}

// PDB relays — plain on/off, no sequencing.
static void pdb_relay1_set(int level) { gpio_set_level(PDB_RELAY1_PIN, level); }
static void pdb_relay2_set(int level) { gpio_set_level(PDB_RELAY2_PIN, level); }
static void pdb_relay3_set(int level) { gpio_set_level(PDB_RELAY3_PIN, level); }
static void pdb_relay4_set(int level) { gpio_set_level(PDB_RELAY4_PIN, level); }

// ------------------------------------------------------------------
// Internal sensor functions
// Replace these later with real sensor drivers.
//
//
// Until real sensors are wired in, these return a fake value that
// changes over time, so you can watch state transitions actually
// happen in simulation. Each call returns ok=false if the "sensor"
// is considered to have failed (always false here — wire this up
// to a real fault check once a real driver exists).
// ------------------------------------------------------------------

static bool read_chamber_pressure(float *out_pressure)
{
    if (external_sensors_valid)
    {
        *out_pressure = external_sensors[2];
        return true;
    }

    // TODO: replace with real sensor read (e.g. I2C/ADC driver call).
    // Placeholder: ramps up slowly each call so air-exchange has
    // something to react to during testing.
    static float fake_chamber_pressure = 0.0f;
    fake_chamber_pressure += 0.05f;

    *out_pressure = fake_chamber_pressure;
    return true; // change to false to simulate a sensor fault
}

static bool read_compressor_inlet_pressure(float *out_pressure)
{
    if (external_sensors_valid)
    {
        *out_pressure = external_sensors[1];
        return true;
    }

    // TODO: replace with real sensor read.
    // Placeholder: ramps up during prepressurisation-style testing.
    static float fake_compressor_inlet_pressure = 0.0f;
    fake_compressor_inlet_pressure += 0.05f;

    *out_pressure = fake_compressor_inlet_pressure;
    return true; // change to false to simulate a sensor fault
}

// ------------------------------------------------------------------
// State implementations
// ------------------------------------------------------------------

static void run_prepressurisation()
{
    valve_close();

    compressor_off();

    pumps_on();

    // Bug 1 fix: this phase ends when the COMPRESSOR INLET pressure
    // has been pumped up to its upper limit (named constant, not
    // magic 1.0f).
    if (status.compressor_inlet_pressure >= compressor_inlet_upper_limit)
    {
        pumps_off();

        status.state =
            PRESSURE_AIR_EXCHANGE;
    }
}

static void run_air_exchange()
{
    pumps_off();

    valve_open();

    compressor_on();

    // Bug 1 fix: per the spec, this phase ends when COMPRESSOR INLET
    // pressure has dropped to its lower limit — not when chamber
    // pressure rises to target. The chamber pressure is still
    // tracked in status (and reported to the Main MCU), but it's no
    // longer what decides the phase transition here.
    if (status.compressor_inlet_pressure <= compressor_inlet_lower_limit)
    {
        compressor_off();

        valve_close();

        status.state =
            PRESSURE_PREPRESSURISATION;
    }
}

// ------------------------------------------------------------------
// I2C communication with the Main MCU
// ------------------------------------------------------------------

//Crc 8 table. Is a table to make algorithm more compute efficient
uint8_t crc8_table[256] = {0, 7, 14, 9, 28, 27, 18, 21, 56, 63, 54, 49, 36, 35, 42, 45, 112, 119, 126, 121, 108, 107, 98, 101, 72, 79, 70, 65, 84, 83,
     90, 93, 224, 231, 238, 233, 252, 251, 242, 245, 216, 223, 214, 209, 196, 195, 202, 205, 144, 151, 158, 153, 140, 139, 130, 133, 168, 175,
     166, 161, 180, 179, 186, 189, 199, 192, 201, 206, 219, 220, 213, 210, 255, 248, 241, 246, 227, 228, 237, 234, 183, 176, 185, 190, 171, 172, 
     165, 162, 143, 136, 129, 134, 147, 148, 157, 154, 39, 32, 41, 46, 59, 60, 53, 50, 31, 24, 17, 22, 3, 4, 13, 10, 87, 80, 89, 94, 75, 76, 69, 66, 
     111, 104, 97, 102, 115, 116, 125, 122, 137, 142, 135, 128, 149, 146, 155, 156, 177, 182, 191, 184, 173, 170, 163, 164, 249, 254, 247, 240,
     229, 226, 235, 236, 193, 198, 207, 200, 221, 218, 211, 212, 105, 110, 103, 96, 117, 114, 123, 124, 81, 86, 95, 88, 77, 74, 67, 68, 25, 30, 23, 
     16, 5, 2, 11, 12, 33, 38, 47, 40, 61, 58, 51, 52, 78, 73, 64, 71, 82, 85, 92, 91, 118, 113, 120, 127, 106, 109, 100, 99, 62, 57, 48, 55, 34, 37, 44, 
     43, 6, 1, 8, 15, 26, 29, 20, 19, 174, 169, 160, 167, 178, 181, 188, 187, 150, 145, 152, 159, 138, 141, 132, 131, 222, 217, 208, 215, 194, 197 ,
     204, 203, 230, 225, 232, 239, 250, 253, 244, 243 };

//Crc8 algorithm using table above. Uses table to make algorithm more efficient
uint8_t computeCRC8(
    const uint8_t *data, 
    size_t length)
{
    uint8_t crc=0x00; 

    for(int i=0; i<length; i++){
        crc=crc8_table[data[i]^crc];
    }
    return crc;
}

uint8_t cmd = 0;
uint8_t info_bit = 0;

typedef enum
{
    CMD_VPUMP1_OFF       = 0x01,
    CMD_VPUMP1_ON        = 0x02,

    CMD_VPUMP2_OFF       = 0x03,
    CMD_VPUMP2_ON        = 0x04,
    CMD_COMPRESSOR_OFF    = 0x05,
    CMD_COMPRESSOR_ON     = 0x06,

    CMD_OPEN_SHUTTERS   = 0x07,
    CMD_CLOSE_SHUTTERS  = 0x08,

    CMD_STANDBY         = 0x09,
    CMD_MEASUREMENTS    = 0x0A,

    CMD_HEATER_ON       = 0x0B,
    CMD_HEATER_OFF      = 0x0C,

    CMD_OPEN_RELAY1      = 0x0D,
    CMD_CLOSE_RELAY1     = 0x0E,
    CMD_OPEN_RELAY2      = 0x0F,
    CMD_CLOSE_RELAY2     = 0x10,
    CMD_OPEN_RELAY3      = 0x11,
    CMD_CLOSE_RELAY3     = 0x12,
    CMD_OPEN_RELAY4      = 0x13,
    CMD_CLOSE_RELAY4     = 0x14

} SlaveCommands;

struct CommandMapping
{
     void (*func)();
    SlaveCommands cmd;
};

static const CommandMapping relay_commands[] =
{
    {pdb_relay1_on,  CMD_OPEN_RELAY1},
    {pdb_relay1_off, CMD_CLOSE_RELAY1},
    {pdb_relay2_on,  CMD_OPEN_RELAY2},
    {pdb_relay2_off, CMD_CLOSE_RELAY2},
    {pdb_relay3_on,  CMD_OPEN_RELAY3},
    {pdb_relay3_off, CMD_CLOSE_RELAY3},
    {pdb_relay4_on,  CMD_OPEN_RELAY4},
    {pdb_relay4_off, CMD_CLOSE_RELAY4},
    {valve_open,      CMD_OPEN_SHUTTERS},
    {valve_close,     CMD_CLOSE_SHUTTERS},
};

void i2c_slave_task(void *pvParameters) {
    uint8_t rx_data[24]; // Large enough to fit either packet type
    uint8_t tx_data[8];
 
    while (1) {
        // Non-blocking read check
        int rx_len = i2c_slave_read_buffer(I2C_SLAVE_NUM, rx_data, sizeof(rx_data), 10 / portTICK_PERIOD_MS);
        
        if (rx_len > 0) {
            uint8_t opcode = rx_data[0];

            if (opcode == OPCODE_SENSORS && rx_len == 16) {
                // Verify CRC for sensor packet (bytes 0 to 14)
                if (computeCRC8(rx_data, 16) == rx_data[15]) {
                    float incoming_sensors[7];
                    for(int i = 0; i < 7; i++) {
                        int16_t raw_val = (static_cast<int16_t>(rx_data[1 + (i*2)]) << 8) |
                                          static_cast<int16_t>(rx_data[2 + (i*2)]);
                        incoming_sensors[i] = (float)raw_val / SENSOR_SCALE;
                    }
                    pressure_update_external_sensors(incoming_sensors);
                }
            } 
            else if (opcode == OPCODE_COMMANDS && rx_len == 3) {
                // Verify CRC for command packet 
                if (computeCRC8(rx_data, 3) == rx_data[3]) {
                    cmd = rx_data[1]; // The command byte
                    info_bit = rx_data[2]; // The info bit
                    for (const auto &mapping : relay_commands) {
                        if (cmd == mapping.cmd) {
                            // For pump and compressor ON commands, use info_bit to set PWM
                            if (cmd == CMD_VPUMP1_ON) {
                                mapping.func();
                                pressure_set_vpump1_pwm(info_bit);
                            } else if (cmd == CMD_VPUMP2_ON) {
                                mapping.func();
                                pressure_set_vpump2_pwm(info_bit);
                            } else if (cmd == CMD_COMPRESSOR_ON) {
                                mapping.func();
                                pressure_set_compressor_pwm(info_bit);
                            }
                            else {
                                mapping.func();
                            }
                            break;
                        }
                        else {
                            ESP_LOGW(TAG, "Received unknown command: 0x%02X", cmd);
                        }
                    }
                }
            }
        }

        // Prepare Status Packet for Master to read anytime
        PressureStatus current_status = pressure_get_status();
        
        tx_data[0] = last_channel_id; 
        tx_data[1] = (uint8_t)current_status.state;
        tx_data[2] = current_status.error;
        
        tx_data[3] = 0x00; 
        tx_data[4] = 0x00; 
        tx_data[5] = 0x00; 
        tx_data[6] = 0x00; 
        tx_data[7] = computeCRC8(tx_data, 7);

        i2c_reset_tx_fifo(I2C_SLAVE_NUM);
        i2c_slave_write_buffer(I2C_SLAVE_NUM, tx_data, sizeof(tx_data), 0);

        // Run local state machine logic
        pressure_update();

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// ------------------------------------------------------------------
// Public API
// ------------------------------------------------------------------

void pressure_init()
{
    // Configure actuator pins as outputs — mirrors the init_gpio_pins()
    // pattern used on the Main MCU side.
    gpio_config_t io_conf = {};
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask =
        (1ULL << VALVE_PIN) |
        (1ULL << COMPRESSOR_PIN) |
        (1ULL << PUMP1_PIN) |
        (1ULL << PUMP2_PIN) |
        (1ULL << PDB_RELAY1_PIN) |
        (1ULL << PDB_RELAY2_PIN) |
        (1ULL << PDB_RELAY3_PIN) |
        (1ULL << PDB_RELAY4_PIN);
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    gpio_config(&io_conf);

    status.state =
        PRESSURE_STANDBY;

    status.chamber_pressure = 0.0f;

    status.compressor_inlet_pressure = 0.0f;

    status.error = PRESSURE_ERR_NONE;

    pumps_off();

    compressor_off();

    valve_close();

    pdb_relay1_off();
    pdb_relay2_off();
    pdb_relay3_off();
    pdb_relay4_off();
}

void pressure_update()
{
    // Bug 3 fix: actually read into status, and check for failure
    // (Bug 4 fix) instead of silently trusting whatever came back.
    float chamber_reading;
    float compressor_inlet_reading;

    bool chamber_ok = read_chamber_pressure(&chamber_reading);
    bool compressor_inlet_ok = read_compressor_inlet_pressure(&compressor_inlet_reading);

    if (!chamber_ok)
    {
        status.error = PRESSURE_ERR_CHAMBER_SENSOR;
        status.state = PRESSURE_ERROR;
    }
    else if (!compressor_inlet_ok)
    {
        status.error = PRESSURE_ERR_INLET_SENSOR;
        status.state = PRESSURE_ERROR;
    }
    else
    {
        status.chamber_pressure = chamber_reading;
        status.compressor_inlet_pressure = compressor_inlet_reading;
        status.error = PRESSURE_ERR_NONE;
    }

    switch(status.state)
    {
        case PRESSURE_STANDBY:

            break;

        case PRESSURE_PREPRESSURISATION:

            run_prepressurisation();

            break;

        case PRESSURE_AIR_EXCHANGE:

            run_air_exchange();

            break;

        case PRESSURE_ERROR:

            pumps_off();

            compressor_off();

            valve_close();

            break;
    }
}

void pressure_cmd_standby()
{
    status.state =
        PRESSURE_STANDBY;
    status.error = PRESSURE_ERR_NONE;

    pumps_off();

    compressor_off();

    valve_close();
}

void pressure_cmd_measurements()
{
    status.state =
        PRESSURE_PREPRESSURISATION;
    status.error = PRESSURE_ERR_NONE;
}

void pressure_set_target_pressure(
    float pressure)
{
    target_pressure = pressure;
}

void pressure_set_compressor_inlet_upper_limit(
    float pressure)
{
    compressor_inlet_upper_limit = pressure;
}

void pressure_set_compressor_inlet_lower_limit(
    float pressure)
{
    compressor_inlet_lower_limit = pressure;
}

PressureStatus pressure_get_status()
{
    return status;
}

bool pressure_system_is_on()
{
    return status.state != PRESSURE_STANDBY;
}

// ------------------------------------------------------------------
// PDB relays — plain on/off, no sequencing.
// ------------------------------------------------------------------

void pdb_relay1_on()  { pdb_relay1_set(1); }
void pdb_relay1_off() { pdb_relay1_set(0); }

void pdb_relay2_on()  { pdb_relay2_set(1); }
void pdb_relay2_off() { pdb_relay2_set(0); }

void pdb_relay3_on()  { pdb_relay3_set(1); }
void pdb_relay3_off() { pdb_relay3_set(0); }

void pdb_relay4_on()  { pdb_relay4_set(1); }
void pdb_relay4_off() { pdb_relay4_set(0); }


extern "C" void app_main() {
    // 1. Init I2C Slave
    i2c_config_t conf_slave = {
        .mode = I2C_MODE_SLAVE,
        .sda_io_num = I2C_SDA_PIN, // GPIO 13
        .scl_io_num = I2C_SCL_PIN, // GPIO 14
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .slave = {
            .addr_10bit_en = 0,
            .slave_addr = I2C_SLAVE_ADDR,
            .maximum_speed = 100000
        },
        .clk_flags = 0,
    };
    i2c_param_config(I2C_SLAVE_NUM, &conf_slave);
    i2c_driver_install(I2C_SLAVE_NUM, conf_slave.mode, 128, 128, 0);

    // 2. Init Pressure Hardware
    pressure_init();

    // 3. Start I2C & State Machine Task
    xTaskCreate(i2c_slave_task, "i2c_slave_task", 4096, NULL, 5, NULL);
}

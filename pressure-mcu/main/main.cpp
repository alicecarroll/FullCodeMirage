#include "main.h"
#include "driver/gpio.h"

// ------------------------------------------------------------------
// Pin assignments — from the Pressure MCU pinout table.
// Board: ESP32-S3 (IO0-IO48 range, dedicated VUSB/VIN pins).
// ------------------------------------------------------------------

#define VALVE_PIN        GPIO_NUM_9   // IO9  — VALVE1
#define COMPRESSOR_PIN   GPIO_NUM_10  // IO10 — PWM1 (digital on/off for now, real PWM later)
#define PUMP1_PIN        GPIO_NUM_38  // IO38 — PWM3, vacuum pump 1 (label mismatch vs PWM2/3 numbering — wired per pin column)
#define PUMP2_PIN        GPIO_NUM_47  // IO47 — PWM2, vacuum pump 2

// PDB relay signals — connector pins 1, 4, 5, 28 -> these GPIOs.
// Plain on/off to MOSFETs on the power distribution board, no
// sequencing/timing logic.
#define PDB_RELAY1_PIN   GPIO_NUM_48  // pin 1, 
#define PDB_RELAY2_PIN   GPIO_NUM_1   // pin 4, 
#define PDB_RELAY3_PIN   GPIO_NUM_2   // pin 5
#define PDB_RELAY4_PIN   GPIO_NUM_21  // pin 28

// I2C_SDA / I2C_SCL (IO13 / IO14) intentionally not defined here yet —
// pressure sensor wiring is still undecided. read_chamber_pressure()/
// read_compressor_inlet_pressure() remain fake/placeholder until
// that's settled.

// ------------------------------------------------------------------
// Internal state
// ------------------------------------------------------------------

static PressureStatus status;

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

    pumps_off();

    compressor_off();

    valve_close();
}

void pressure_cmd_measurements()
{
    status.state =
        PRESSURE_PREPRESSURISATION;
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

// ------------------------------------------------------------------
// Example ESP-IDF entry point
// ------------------------------------------------------------------

extern "C" void app_main(void)
{
    printf("Hello\n");
    pdb_relay1_off();
    pdb_relay2_off();
    pdb_relay3_off();
    pdb_relay4_off();
}

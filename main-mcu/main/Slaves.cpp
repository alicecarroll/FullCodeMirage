#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "driver/gpio.h"
#include "Settings.h"
#include "Multiplexer.h"
#include "Initialize.h"
//#include "communication.h"
#include "Slaves.h"
#include "ErrorStatus.h"
#include "read_sensors.h"

// Shared slave address
#define Slave_MCU_addr  0x10

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

// Internal helper
static bool select_slave(
    SlaveDevice slave,
    uint8_t* mux_channel,
    gpio_num_t* reset_pin)
{
    switch(slave)
    {
        case thermal_mcu:
            *mux_channel = multiplex_Tt3_devP;
            *reset_pin = Thermal_reset_PIN;
            return true;

        case pressure_mcu:
            *mux_channel = multiplex_Tp1_devT;
            *reset_pin = Pressure_reset_PIN;
            return true;
    }
    return false;
}


void recover_i2c_driver() {
    ESP_LOGW("I2C_DEBUG", "Hard resetting I2C peripheral...");
    
    // 1. Delete the current driver instance
    i2c_driver_delete(I2C_master);
    
    // 2. Wait for pending operations to clear
    vTaskDelay(pdMS_TO_TICKS(50));
    
    // 3. Re-install the driver using your existing initialization
    init_i2c(); 
}

// Reset slave MCU
void slave_reset(SlaveDevice slave)
{
    uint8_t mux_channel;
    gpio_num_t reset_pin;

    if (!select_slave(slave, &mux_channel, &reset_pin))
    {
        return;
    }

    // REMOVED: gpio_config() calls that were causing the conflict.
    // Instead, just perform the pulse on the existing, pre-configured pin.
    
    gpio_set_level(reset_pin, 0); // Pulse low
    vTaskDelay(pdMS_TO_TICKS(100));
    gpio_set_level(reset_pin, 1); // Release
    vTaskDelay(pdMS_TO_TICKS(20));
}


bool thermal_test_send_package(
    SlaveDevice slave, 
    uint8_t channel_id, //0x00- 0x07
    uint8_t mode, //0 bang bang 1 PID 155-255 D_cycle
    int16_t currentTemp, // 5000 = 50,0C 
    int16_t target  
)
{

    //From earlier
    uint8_t mux_channel; //multiplexer channel, defined in select_slave
    gpio_num_t reset_pin;//reset pin, defined in select_slave

    //Purpose is to run selectskave if is just to handle error
    if (!select_slave(
            slave,
            &mux_channel,
            &reset_pin))
        {
            return false;
        }
    

    //Purpose us to run selectmuxchannel if is just to handle errors
    if (sel_mux_channel(mux_channel) != ESP_OK)
    {
        return false;
    }
    
    //Creates package
    const int package_Length=8;
    uint8_t package[package_Length];

    package[0]=0x01; 
    package[1]= channel_id;
    package[2]=mode;
    package[3]= static_cast<uint8_t>((currentTemp >> 8) & 0xFF); //MSB data
    package[4]= static_cast<uint8_t>(currentTemp & 0xFF);   //LSB data
    package[5]= static_cast<uint8_t>((target >> 8) & 0xFF); //MSB target
    package[6]= static_cast<uint8_t>(target & 0xFF);   //LSB target

    package[7]=computeCRC8(package, package_Length-1);

    esp_err_t err =
            i2c_master_write_to_device(
                I2C_master,
                Slave_MCU_addr,
                package,
                sizeof(package),
                100 / portTICK_PERIOD_MS
            );

    return (err == ESP_OK); 

}

bool thermal_test_receive_package(  //when passing variable to this one remember to pass as &channel_id for all pointer
    SlaveDevice slave,
    uint8_t* channel_id, 
    uint8_t* mode,
    uint8_t* power,
    uint16_t* target,
    uint8_t* status,
    uint8_t* error)
{
    int dataLength=8;
    *error=0; 

    uint8_t mux_channel; //multiplexer channel, defined in select_slave
    gpio_num_t reset_pin;//reset pin, defined in select_slave

    //Purpose is to run selectskave "if" is just to handle error
    if (!select_slave(
            slave,
            &mux_channel,
            &reset_pin))
        {
            return false;
        }

    //Purpose is to run selectmuxchannel "if" is just to handle errors
    if (sel_mux_channel(mux_channel) != ESP_OK)
    {
        return false;
    }
    
    
    

    //Size of incoming data
    uint8_t data[dataLength];

    esp_err_t err =
        i2c_master_read_from_device(
            I2C_master,
            Slave_MCU_addr,
            data,
            sizeof(data),
            100 / portTICK_PERIOD_MS
        );

    if (err != ESP_OK)
    {
        *error=1; //esp error
        return false;
    }

    if(data[7]!=computeCRC8(data, dataLength-1)){  //Verifies packet integrity start value for crc is 0 so crc should return 0
        *error=2; //crc error packet has been corrupted
        return false; 
    }

    *channel_id=data[0];
    *mode=data[1];
    *power=data[2];
    *target=(static_cast<uint16_t>(data[3]) <<8 | static_cast<uint16_t>(data[4])); //takes two int8 and combines into int16
    *status=data[5];

    return true;
}







bool pressure_send_sensors(
    SlaveDevice slave, 
    const SensorData &sensor_data
) {
    uint8_t mux_channel; 
    gpio_num_t reset_pin;

    if (!select_slave(slave, &mux_channel, &reset_pin)) return false;
    if (sel_mux_channel(mux_channel) != ESP_OK) return false;
    
    // Packet layout:
    // [0] opcode
    // [1..14] seven int16 sensor values
    // [15] CRC8
    const int package_length = 16;
    uint8_t package[package_length];

    package[0] = PRESSURE_PACKET_SENSORS;

    int16_t sensor_ints[7] = {
        (int16_t)(sensor_data.Pp3 * SENSOR_SCALE),
        (int16_t)(sensor_data.Pp1 * SENSOR_SCALE),
        (int16_t)(sensor_data.Pp2 * SENSOR_SCALE),
        (int16_t)(sensor_data.Pa1 * SENSOR_SCALE),
        (int16_t)(sensor_data.Tp1 * SENSOR_SCALE),
        (int16_t)(sensor_data.Tp2 * SENSOR_SCALE),
        (int16_t)(sensor_data.Tp3 * SENSOR_SCALE)
    };

    for(int i = 0; i < 7; i++) {
        package[1 + (i*2)] = (sensor_ints[i] >> 8) & 0xFF; // MSB
        package[2 + (i*2)] = sensor_ints[i] & 0xFF;        // LSB
    }

    package[package_length - 1] = computeCRC8(package, package_length - 1);

    esp_err_t err = i2c_master_write_to_device(
        I2C_master, Slave_MCU_addr, package, sizeof(package), 50 / portTICK_PERIOD_MS
    );

    return (err == ESP_OK);
}




bool pressure_send_command(
    SlaveDevice slave, 
    uint8_t cmd,
    uint8_t info_bit
) {
    uint8_t mux_channel; 
    gpio_num_t reset_pin;

    if (!select_slave(slave, &mux_channel, &reset_pin)) return false;
    if (sel_mux_channel(mux_channel) != ESP_OK) return false;
    
    // Every command has the same frame shape. The info byte is reserved for
    // PWM values today and mode values in the future; it is never omitted.
    uint8_t package[PRESSURE_COMMAND_FRAME_SIZE];

    package[0] = PRESSURE_PACKET_COMMANDS;
    package[1] = cmd;
    package[2] = info_bit;
    package[PRESSURE_COMMAND_FRAME_SIZE - 1] =
        computeCRC8(package, PRESSURE_COMMAND_FRAME_SIZE - 1);

    esp_err_t err = i2c_master_write_to_device(
        I2C_master, Slave_MCU_addr, package, sizeof(package), 100 / portTICK_PERIOD_MS
    );

    return (err == ESP_OK);
}


bool pressure_receive_package(
    SlaveDevice slave,
    PressureStatusData* status_out
) {
    uint8_t mux_channel; 
    gpio_num_t reset_pin;

    if (!select_slave(slave, &mux_channel, &reset_pin)) return false;
    if (sel_mux_channel(mux_channel) != ESP_OK) return false;

    const int data_length = 8;
    uint8_t data[data_length] = {};

    esp_err_t err = i2c_master_read_from_device(
        I2C_master, Slave_MCU_addr, data, sizeof(data), 100 / portTICK_PERIOD_MS
    );

    if (err != ESP_OK)
    {
        return false;
    }

    if (data[data_length - 1] != computeCRC8(data, data_length - 1)) {
        return false; // CRC mismatch
    }

    status_out->channel_id = data[0];
    status_out->state      = data[1];
    status_out->error_code = data[2];
    status_out->relay_mask = data[3];
    status_out->pump1_pwm = data[4];
    status_out->pump2_pwm = data[5];
    status_out->compressor_pwm = data[6];
    status_out->valve_open = (data[3] & 0x80) != 0;
    status_out->manual_override = (data[3] & 0x40) != 0;

    return true;
}

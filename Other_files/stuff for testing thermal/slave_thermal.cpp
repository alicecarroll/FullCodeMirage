#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "driver/gpio.h"
#include "Settings.h"
#include "Multiplexer.h"
#include "communication.h"
#include "slave_thermal.h" 
#include "read_sensors.h"

// Shared slave address
#define Slave_MCU_addr  0x10

uint8_t crc8_table[256] = {0, 7, 14, 9, 28, 27, 18, 21, 56, 63, 54, 49, 36, 35, 42, 45, 112, 119, 126, 121, 108, 107, 98, 101, 72, 79, 70, 65, 84, 83,
     90, 93, 224, 231, 238, 233, 252, 251, 242, 245, 216, 223, 214, 209, 196, 195, 202, 205, 144, 151, 158, 153, 140, 139, 130, 133, 168, 175,
     166, 161, 180, 179, 186, 189, 199, 192, 201, 206, 219, 220, 213, 210, 255, 248, 241, 246, 227, 228, 237, 234, 183, 176, 185, 190, 171, 172, 
     165, 162, 143, 136, 129, 134, 147, 148, 157, 154, 39, 32, 41, 46, 59, 60, 53, 50, 31, 24, 17, 22, 3, 4, 13, 10, 87, 80, 89, 94, 75, 76, 69, 66, 
     111, 104, 97, 102, 115, 116, 125, 122, 137, 142, 135, 128, 149, 146, 155, 156, 177, 182, 191, 184, 173, 170, 163, 164, 249, 254, 247, 240,
     229, 226, 235, 236, 193, 198, 207, 200, 221, 218, 211, 212, 105, 110, 103, 96, 117, 114, 123, 124, 81, 86, 95, 88, 77, 74, 67, 68, 25, 30, 23, 
     16, 5, 2, 11, 12, 33, 38, 47, 40, 61, 58, 51, 52, 78, 73, 64, 71, 82, 85, 92, 91, 118, 113, 120, 127, 106, 109, 100, 99, 62, 57, 48, 55, 34, 37, 44, 
     43, 6, 1, 8, 15, 26, 29, 20, 19, 174, 169, 160, 167, 178, 181, 188, 187, 150, 145, 152, 159, 138, 141, 132, 131, 222, 217, 208, 215, 194, 197 ,
     204, 203, 230, 225, 232, 239, 250, 253, 244, 243 };


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

//selects slave for multiplexer. Mux channel value will be saved in uc channel and reset pin value will be saved in reset pin

static bool select_slave(  
    SlaveDevice slave,
    uint8_t* mux_channel, 
    gpio_num_t* reset_pin)// I dont understand reset pin
{
    switch(slave)
    {
        case thermal_mcu:

            *mux_channel =
                multiplex_Tp1_devT;  // actual multiplexer channel

            *reset_pin =
                Thermal_reset_PIN;

            return true;


        case pressure_mcu:

            *mux_channel =
                multiplex_Tt3_devP;

            *reset_pin =
                Preassure_reset_PIN;

            return true;
    }

    return false;
}



//To send data. Package |pointer|Mode s1|temp s1|mode s2|temps2| //Slave will have to also have the same defined length of lists as decided beforehand
bool slave_send_data( //Send data thermal might need to switch name
    SlaveDevice slave, //Send data thermal might need to switch name
    SlaveData data_id, // 0x00
    int16_t data[], //Data and mode MUST have the same number of elements use 5000= 50.00 C
    uint8_t mode[],
    int number_switches)  // number switches is same as number of elements in mode and data array
    {
        
        int package_size=number_switches*3+2;//number of switches *3 + crc+ pointer

        uint8_t package[package_size];
        
        //Selectslave works properly though the actual select slave part might not be used depending on implementation
        uint8_t mux_channel;
        gpio_num_t reset_pin;

        if (!select_slave(
            slave,
            &mux_channel,
            &reset_pin))
        {
            return false;
        }

        //actual package creation

        package[0] = Data_package;

        //splits the uint16 into 2 uint 8 and adds that together with mode into array
        //Structure should be |pointer|mode1|val1 MSB|val1 LSB|mode2|val2 MSB|val2 LSB| etc
        for(int i=0; i<number_switches; i+=3)
        {
            package[3*i+1]= mode[i];
            package[3*i+2]= static_cast<uint8_t>((data[i] >> 8) & 0xFF); //MSB data
            package[3*i+3]= static_cast<uint8_t>(data[i] & 0xFF);   //LSB data
        }

        //Add crc8 checksum
        package[package_size-1]=computeCRC8(package, package_size-1);  //Sets the checksum last. To pass cheksum put entire package through crc8 it should return 0
        //add sending part
        
        if (select_mux_channel(mux_channel) != ESP_OK)
        {
            return false;
        }

        esp_err_t err =             // I dunno what this returns
            i2c_master_write_to_device(
                I2C_master,
                Slave_MCU_addr,
                package,
                sizeof(package),
                100 / portTICK_PERIOD_MS
            );

        return (err == ESP_OK); //I dunno what this does
    }

//|Pointer|mode/power|Temperature MSB|Temperature LSB| Target MSB| Target LSB| crc8|
bool slave_send_command(
    SlaveDevice slave,
    uint8_t data_id,  // pointer 0x00-0x07
    uint8_t mode, //Mode 0 PID , 1 bangbang 155-255 manual
    int16_t data,       //Data and target can be negative
    int16_t target)
    {
        const int package_Length=8;
        uint8_t package[package_Length]; 

        //Selectslave works properly though the actual select slave part might not be used depending on implementation
        uint8_t mux_channel;
        gpio_num_t reset_pin;
        if (!select_slave(
            slave,
            &mux_channel,
            &reset_pin))
        {
            return false;
        }


        //Creates package
        package[0]=0x01; 
        package[1]= data_id;
        package[2]= mode; 
        package[3]= static_cast<uint8_t>((data >> 8) & 0xFF); //MSB data
        package[4]= static_cast<uint8_t>(data & 0xFF);   //LSB data
        package[5]= static_cast<uint8_t>((target >> 8) & 0xFF); //MSB target
        package[6]= static_cast<uint8_t>(target & 0xFF);   //LSB target
        package[7]= computeCRC8(package, package_Length-1);

        //Sends stuff 
        if (select_mux_channel(mux_channel) != ESP_OK)
        {
            return false;
        }

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


//Read slave status
bool slave_read_status(
    SlaveDevice slave,
    SlaveStatus* status)
{
    uint8_t mux_channel;
    gpio_num_t reset_pin;

    //checks select slave
    if (!select_slave(
            slave,
            &mux_channel,
            &reset_pin))
    {
        return false;
    }

    if (select_mux_channel(mux_channel) != ESP_OK)
    {
        return false;
    }

    uint8_t data[2];
    
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
        status->online = false;

        return false;
    }

    status->online = true;

    status->state =
        data[0];

    status->error =
        data[1];

    return true;
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
    if (select_mux_channel(mux_channel) != ESP_OK)
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

bool thermal_test_recieve_package(
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
    if (select_mux_channel(mux_channel) != ESP_OK)
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

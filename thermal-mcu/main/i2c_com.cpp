//libaries
#include "string.h"
#include "driver/i2c.h"
#include "esp_log.h"

//other parts of project
#include "config.h"
#include "i2c_com.h"

//numbers for functions
constexpr size_t INDIVIDUAL_SWITCH_RX_LEN = 8;
constexpr size_t INDIVIDUAL_SWITCH_TX_LEN = 8;

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

//This one should be in its own freertos task which will run paralell to the controller task
void i2c_loop_task(void *pvParameters)
{

    individual_switch_data_rx receive_indvidual_switch_packet;
    uint8_t dataBuffer[rx_buffer_len]; //data buffer for incoming i2c communication
    uint8_t controllerData[number_switches*6]={}; // Data array for controller

    for(int i=0; i<number_switches*6;i++) //initilizes controllerData with start values (0% dutycycle for all switches)
    {
        switch(i%6){
            case 0:
                controllerData[i]= static_cast<uint8_t>(i/6);
                break;
            case 1: 
                controllerData[i]=155; //0% dutycycle
                break;
        }
    }

    while(1){
        //non-blocking read //This should be changed such that it wont get partial messages
        int bytes_read = i2c_slave_read_buffer(I2C_PORT,dataBuffer,rx_buffer_len, 0);

        if(bytes_read>0) //is true if package has been recieved
        {
            uint8_t regi=dataBuffer[0]; //regiester/command byte which determines packet type

            switch(regi){  //inside this switch case multiple package types will be implemented
                case packet_type_indvidual_switch:
                    
                    if (bytes_read==8) //ensures correct package size
                    {
                        if(!data_unpack_indvidual_switch(dataBuffer,&receive_indvidual_switch_packet)){
                            ESP_LOGE("ERROR", "CRC8 failed for indvidual switch packet %d",regi);
                        }
                        //implement clear of databuffer after read message
                    }
                    break;
                default:
                        ESP_LOGW("I2C", "Unknown packet register: 0x%02X", regi);
                    break;
            }
            //here it should send data to another freertos task

        }
    }   

}

/*
Under this is packet structures
*/

//receive packets

//thermal indvidual switch packet
bool data_unpack_indvidual_switch(const uint8_t *dataBuffer, individual_switch_data_rx *packet){ 
    if(dataBuffer[INDIVIDUAL_SWITCH_RX_LEN-1]!=computeCRC8(dataBuffer, INDIVIDUAL_SWITCH_RX_LEN-1)){
        return false; //Crc8 didn't pass so bad datastream
    }

    //unpacks buffer into the packet
    packet -> regist    = dataBuffer[0]; //regiester/command determines packet type
    packet -> switchID  = dataBuffer[1]; //switch id 0-7
    packet -> mode      = dataBuffer[2]; //mode 
    packet ->temperature= static_cast<int16_t>(static_cast<uint16_t>(dataBuffer[3]) <<8 | static_cast<uint16_t>(dataBuffer[4]));
    packet -> target    = static_cast<int16_t>(static_cast<uint16_t>(dataBuffer[5]) <<8 | static_cast<uint16_t>(dataBuffer[6]));
    packet -> crc8      = dataBuffer[7];
    
    return true; 
}


//send packets
//Thermal indvidual switch send packet
bool data_pack_indvidual_switch(
    const individual_switch_data_tx *packet,
    uint8_t *data // data should be 1 byte more than packet
    )
{
    uint16_t unsigned_target=static_cast<uint16_t>(packet->target);
    
    data[0]=packet -> switchID;
    data[1]=packet -> mode; 
    data[2]=packet -> D_cycle;
    data[3]=static_cast<uint8_t>((unsigned_target>>8) & 0xFF); //msb
    data[4]=static_cast<uint8_t>((unsigned_target) & 0xFF); //lsb
    data[5]=packet -> status;
    data[6]=0x00;

    data[7]=computeCRC8(data, INDIVIDUAL_SWITCH_TX_LEN-1);
    return true;
}


//xTaskCreate stuff to handle parallelization
//Send data to paralell task
void send_data_parallel_tasks(uint8_t *data, size_t len){

}

//recieve data from paralell task










//libaries
#include <cstdint>
#include <cstring>

#include "string.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "driver/i2c_slave.h"

//other parts of project
#include "config.h"
#include "i2c_com.h"


//numbers for functions
constexpr size_t INDIVIDUAL_SWITCH_RX_LEN = 8;
constexpr size_t INDIVIDUAL_SWITCH_TX_LEN = 8;

//Crc 8 table. Is a table to make algorithm more compute efficient DO NOT TOUCH OR BECOME SAD
uint8_t crc8_table[256] = {0, 7, 14, 9, 28, 27, 18, 21, 56, 63, 54, 49, 36, 35, 42, 45, 112, 119, 126, 121, 108, 107, 98, 101, 72, 79, 70, 65, 84, 83,
     90, 93, 224, 231, 238, 233, 252, 251, 242, 245, 216, 223, 214, 209, 196, 195, 202, 205, 144, 151, 158, 153, 140, 139, 130, 133, 168, 175,
     166, 161, 180, 179, 186, 189, 199, 192, 201, 206, 219, 220, 213, 210, 255, 248, 241, 246, 227, 228, 237, 234, 183, 176, 185, 190, 171, 172, 
     165, 162, 143, 136, 129, 134, 147, 148, 157, 154, 39, 32, 41, 46, 59, 60, 53, 50, 31, 24, 17, 22, 3, 4, 13, 10, 87, 80, 89, 94, 75, 76, 69, 66, 
     111, 104, 97, 102, 115, 116, 125, 122, 137, 142, 135, 128, 149, 146, 155, 156, 177, 182, 191, 184, 173, 170, 163, 164, 249, 254, 247, 240,
     229, 226, 235, 236, 193, 198, 207, 200, 221, 218, 211, 212, 105, 110, 103, 96, 117, 114, 123, 124, 81, 86, 95, 88, 77, 74, 67, 68, 25, 30, 23, 
     16, 5, 2, 11, 12, 33, 38, 47, 40, 61, 58, 51, 52, 78, 73, 64, 71, 82, 85, 92, 91, 118, 113, 120, 127, 106, 109, 100, 99, 62, 57, 48, 55, 34, 37, 44, 
     43, 6, 1, 8, 15, 26, 29, 20, 19, 174, 169, 160, 167, 178, 181, 188, 187, 150, 145, 152, 159, 138, 141, 132, 131, 222, 217, 208, 215, 194, 197 ,
     204, 203, 230, 225, 232, 239, 250, 253, 244, 243};

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
//Fix sending stuff to main
//I2C Receive task
void i2c_loop_task(void *pvParameters)
{
    bool packet_error=false;
    const individual_switch_data_rx default_off_package = {
        .regist=0x01,
        .switchID=0x00,
        .mode=155,
        .temperature=20,
        .target=0,
        .crc8=0}; //crc8 should get actual calue but doesnt really matter
    individual_switch_data_rx receive_indvidual_switch_packet;
    individual_switch_data_rx controllerData;
    i2c_data_evt evtData; //Data from i2c

    while(1){
        
        if(xQueueReceive(dataQueue_slave_rx,&evtData,portMAX_DELAY)) //is true if package has been recieved
        {
            ESP_LOGI("I2C loop:", "First check");
            packet_error=false;
            uint8_t regi=evtData.data[0]; //regiester/command byte which determines packet type

            switch(regi){  //inside this switch case multiple package types will be implemented
                case packet_type_indvidual_switch:
                    
                    if (evtData.length==8) //ensures correct package size
                    {
                        if(!data_unpack_indvidual_switch(evtData.data,&receive_indvidual_switch_packet)){
                            ESP_LOGE("I2C loop:", "CRC8 failed for indvidual switch packet %d",regi);
                            packet_error=true;
                        }
                        else if(receive_indvidual_switch_packet.switchID>=number_switches){
                            packet_error=true;
                            ESP_LOGE("I2C loop:", "Incorrect switch ID %d",regi);
                        }
                        else
                        {
                            controllerData = receive_indvidual_switch_packet;
                            ESP_LOGE("I2C loop:", "controller data? %d",regi);
                        }
                        //implement clear of databuffer after read message
                        //Add timeout for specific packages ie replace crc8 with time since this package has last been received
                       
                    }
                    packet_error=true;
                    ESP_LOGE("I2C loop:", "Incomplete packet");
                    break;
                case packet_stop_all:  //Emergency stop
                    controllerData=default_off_package;
                    controllerData.regist = packet_stop_all;
                    break;
                case packet_resume_all: //Resume normal operations
                    controllerData=default_off_package;
                    controllerData.regist = packet_resume_all;
                    break;
                default:
                        ESP_LOGW("I2C", "Unknown packet register: 0x%02X", regi);
                    break;
            }
            //here it should send data to another freertos task
            
            if(!packet_error){
                xQueueOverwrite(dataQueue,&controllerData); //Data to control
            }
            //Add a way to send packeterrors to master

            
            
        }
        
    }   
    
}

void i2c_loop_send_task(void *pvParameters)
{
    i2c_data_evt send; //packet with stuff to send
    while(1)
    {
        if(xQueueReceive(dataQueue_slave_tx,&send, portMAX_DELAY)==pdTRUE) //Reads queue coming from controller
        { 
            i2c_slave_reset_tx_fifo(slave_handle);//Wipes send queue in preperation for it getting filled with send data at later stage logic:As this task triggers after/during recieve it should have time to wipe and write before the request as the timeout between write and read is 50ms
            uint32_t bytes_written=0; //Variable needed but not used currently
            esp_err_t err =i2c_slave_write(slave_handle,send.data,static_cast<uint32_t>(send.length),&bytes_written,1000); //Writes to the buffer
            if (err != ESP_OK) {
                ESP_LOGE("I2C_SLAVE", "Write failed: %s", esp_err_to_name(err));
            }
        }        
    }
}

/*
I2C callback funtions
*/

//I2C recievedatatask interrupt task (event driven callback task onReceive)
bool IRAM_ATTR i2c_slave_on_receive_cb(i2c_slave_dev_handle_t slave_handle, const i2c_slave_rx_done_event_data_t *edata, void *arg)
{ 
    BaseType_t taskAwoken=pdFALSE; //for if this task interrups other task
    i2c_data_evt evt; //Struct to put data into
    evt.length=(edata->length>sizeof(evt.data)?sizeof(evt.data):edata->length);
    memcpy(evt.data,edata->buffer,evt.length); //Buffer into data
    xQueueOverwriteFromISR(dataQueue_slave_rx,&evt, &taskAwoken); //Sends data to i2c handler task
    return (taskAwoken==pdTRUE);
}

//I2c requesttask interrupt task (event driven callback task onRequest) 
//Function currently not in use/does nothing needs to exist because .onrequest needs to be defined in config
bool IRAM_ATTR i2c_slave_on_request_cb(i2c_slave_dev_handle_t slave_handle, const i2c_slave_request_event_data_t *evt_data, void *arg)
{
    BaseType_t taskAwoken=pdFALSE;
    //vTaskNotifyGiveFromISR(i2c_send_task_handle,&taskAwoken); //Sends ping to the i2c write task that its supposed to load the queue
    return(taskAwoken==pdTRUE);
}

/*
Under this is packet structures
*/

//receive packets

//thermal indvidual switch packet
bool data_unpack_indvidual_switch(const uint8_t *dataBuffer, individual_switch_data_rx *packet){  //Input uint8 arary and get a struct
    if(dataBuffer[INDIVIDUAL_SWITCH_RX_LEN-1]!=computeCRC8(dataBuffer, INDIVIDUAL_SWITCH_RX_LEN-1)){
        return false; //Crc8 didn't pass so bad datastream
    }

    //unpacks buffer into the packet
    packet -> regist    = dataBuffer[0]; //regiester/command determines packet type
    packet -> switchID  = dataBuffer[1]; //switch id 0-7
    packet -> mode      = dataBuffer[2]; //mode 0 bang bang 1 PID 155-255 D_cycle
    packet -> temperature= static_cast<int16_t>(static_cast<uint16_t>(dataBuffer[3]) <<8 | static_cast<uint16_t>(dataBuffer[4]));
    packet -> target    = static_cast<int16_t>(static_cast<uint16_t>(dataBuffer[5]) <<8 | static_cast<uint16_t>(dataBuffer[6]));
    packet -> crc8      = dataBuffer[7];
    
    return true; 
}


//send packets
//Thermal indvidual switch send packet
bool data_pack_indvidual_switch(    //input a struct and get ouy uint8array
    const individual_switch_data_tx *packet,
    uint8_t *data // data should be 1 byte more than packet
    )
{
    int16_t target_int=static_cast<int16_t>(packet->target*100.0f); //Keep in mind that code will not work for numbers above 320C or below -320C due to limits of int16
    uint16_t unsigned_target=static_cast<uint16_t>(target_int);
    
    data[0]=packet -> switchID;
    data[1]=packet -> mode; 
    data[2]=packet -> D_cycle;
    data[3]=static_cast<uint8_t>((unsigned_target>>8) & 0xFF); //msb
    data[4]=static_cast<uint8_t>((unsigned_target) & 0xFF); //lsb
    data[5]=packet -> status; //Errors
    data[6]=packet->global_mode; //Which global mode its in
    data[7]=computeCRC8(data, INDIVIDUAL_SWITCH_TX_LEN-1);
    return true;
}












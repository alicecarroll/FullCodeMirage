//libaries
#include "string.h"
#include "driver/i2c.h"
#include "esp_log.h"

//other parts of project
#include "config.h"
#include "i2c_com.h"
#include "esp_timer.h"
#include "control.h"



// Things that should be sent to main MCU: Modes of all pin ie Pin1 manual, pin2 PID et. 


//Things main MCU should be able to send. Sensor data for all sensors. 


//Need to add error handling to the whole program
class Controller{
  public:
    virtual float update(float desired_value, float meas, float dutyCycle) = 0; //Dutycycle only used for manual mode. Its an extra variable to prevent sending bad data in case of wrong mode being on
    virtual void reset()=0;
};

//Fix bang bang such that it has proper deadzon (current deadzone is not correct)
class Bang : public Controller // ON/OFF controller
{   
  private: 
    float dutyOn;
    float deadZone; //The distance from desired temp before turning things on/off
    bool cooler;

  public: 
    Bang(float D_cycle, float dead_zone, bool cool){
      dutyOn=D_cycle; deadZone=dead_zone; cooler=cool;
    }

    float update(float desired_value, float meas, float duty_cycle_on) override{  
      dutyOn=duty_cycle_on;
        if(cooler && (meas>=(desired_value+deadZone))){ //with cooler
          return dutyOn;
        }

        if(!cooler && (meas<=desired_value-deadZone)){ // with heater
          return dutyOn;
        }
        return 0;
      }
    

    void reset() override{
      //Think this should be empty
    }

    void changeParams(float D_cycle, float dead_zone, bool cool){ //changes the parameters for Bang controller
      dutyOn=D_cycle; deadZone=dead_zone; cooler=cool;
    }
};

class PID_control : public Controller{
  private: 
    float kp, ki, kd;
    float integral=0;
    float prev_error = 0;  // 
    float dt; // in seconds
    bool cooler;

  public: 
    PID_control(float p, float i, float d, float timestep, bool cool){
      kp=p; ki=i; kd=d; dt=timestep; cooler=cool;
      reset();
    }
     

    void reset() override{
      integral=0;
      prev_error=0;
    }

    float update(float desired_value, float meas, float max_duty_cycle) override{  //Runs the PID control. 
      const int windup_limit=100;  //The windup limit needs to be dimensioned properly !! 
      float derivative,error;
      if (dt==0) //makes sure dt isnt 0
      {  
        return 0;
      }

      error=desired_value-meas;
      if (cooler){
        error=-error;
      }
      
      integral+= error * dt;

      //Anti windup. // This can be implemented better if needed
      if (integral>windup_limit){  
        integral=windup_limit;
        }
      else if(integral<-windup_limit){
        integral=-windup_limit;
        }

      derivative=(error-prev_error)/dt;
      prev_error=error;

      float PID_output=kp*error+ki*integral+kd*derivative; 

      //Converts output to duty cyvle
      if (PID_output>max_duty_cycle){PID_output=max_duty_cycle;} // This part might be changed such that it handles negatives to be able to use cooler on same PID
      else if( PID_output<0){PID_output=0;}  // IE returns 100- (-100) where negative corresponds to cooler

      return PID_output;
    }

    void changeParams(float p, float i, float d, float timestep, bool cool){ //Changeng the values for PID control
      kp=p; ki=i; kd=d; dt=timestep; cooler=cool;
      //reset();  //The reset may be removed here depending on what we want from chagneParams i think we'll want it here
    }

    void set_dt(float timestep){
      dt=timestep;
    }
};

class manual_control : public Controller {
  private:
    float D_cycle;

  public:
    manual_control(float Duty_Cycle){
      D_cycle=Duty_Cycle; 
    }
    // the 
    float update(float desired_value, float meas, float Duty_Cycle2) override{ //duty cycle2 is only it's own variable for safety it might be changed to use space of desired value of meas. In that case dutycycle 2 can be removed from controller class
      D_cycle=Duty_Cycle2;
      return Duty_Cycle2;
    }
    void reset() override{
      //should be empty
    }
    void changeParams(float Duty_Cycle2){
      D_cycle=Duty_Cycle2;
    }
};

// int d_cycle_heat_control(int mode, double value, double desired_value, int manual_d_cycle){  //will remove
//   switch (mode){
//     case 1:  //PID CONTROL
//       //Place PID here
//       break; 
//     case 2: //Simple control
//       //AA
//       break; 
//     case 3:
//       return manual_d_cycle; 
//       break; 
//   }

const int number_controllers=number_switches;
//useful things
Bang bangPool[number_controllers]{
  Bang(100,1,false),
  Bang(100,1,false), 
  Bang(100,1,false),
  Bang(100,1,false),
  Bang(100,1,false),
  Bang(100,1,false),
  Bang(100,1,false),
  Bang(100,1,false)
};


//Some things for controllers
PID_control PIDPool[number_controllers]{
  PID_control(0.1,0.1,1.5,1,false), //0
  PID_control(0.1,0.1,1.5,1,false),
  PID_control(0.1,0.1,1.5,1,false),
  PID_control(0.1,0.1,1.5,1,false),
  PID_control(0.1,0.1,1.5,1,false),
  PID_control(0.1,0.1,1.5,1,false),
  PID_control(0.1,0.1,1.5,1,false),
  PID_control(0.1,0.1,1.5,1,false) //7
};

manual_control manualPool[number_controllers]{
  manual_control(0), //0
  manual_control(0),
  manual_control(0),
  manual_control(0),
  manual_control(0),
  manual_control(0),
  manual_control(0),
  manual_control(0) //7
};

Controller* controllers[number_controllers]{ //Default values for controolers may change to &manualpool or &PIDpool
  &manualPool[0],
  &manualPool[1],
  &manualPool[2],
  &manualPool[3],
  &manualPool[4],
  &manualPool[5], //Coolers should be in bangbang during normal operation while heaters should be in PID or bangbang depending on how often they're in manual. If they're often in manual use bang bang
  &manualPool[6],
  &manualPool[7]
};
float temperature_lower_bound=-80; //Celcius
float temperature_higher_bound=120; //celcius

//Actual variables
// unsigned long previous_time=0;
// float timeStep=1; 
// unsigned long current_time;
// //placeholders for actual implementation
// float desired_values[number_controllers];
// float meas_values[number_controllers];


// bool change_controller=false; //Command for when you want to change controltype or initilize controller with new values
// bool initilize_val=false; //Command for initilizing new values for a specific controltype as decided by controltype
// int num_controller; //Chooses which controller to change ie nr 1 or nr 2 etc
// char control_type;  //chooses which control type manual PID or bangbang

// float D_cycle; float dead_zone; bool cool=false; // For changeParams in bang
// float p; float i; float d; float timestep; bool cool1=false; //For changeParams in PID
// float duty_cycle1; //for changeParams in manual

//Converts a dutycycle 0-100% to on off logic together with the period
bool duty_cycle_to_on_off(float dutyCycle, const int64_t period_us, int64_t currentTime_us)
{
  if(dutyCycle>100){ //Duty cycle should not be above 100
    ESP_LOGW("Control", "Duty cycle above 100 value: %.2f",dutyCycle);
    dutyCycle=100.0f;
    
  }
  if(dutyCycle<0){ //Duty cycle shoyld not be below 0
    ESP_LOGW("Control", "Duty cycle below 0 value: %.2f",dutyCycle);
    dutyCycle=0.0f;
  }
  if(period_us<2){ //period should not be below 2us otherwise function doesnt do shit
    ESP_LOGW("Control", "Period less than 1us should not be used current period %" PRId64, period_us);
    return false;
  }
    
  int64_t onTime=static_cast<int64_t>(period_us*dutyCycle/100);
  
  //Checks if it should be on otherwise off
  if(currentTime_us%period_us<onTime){
    return true;
  }
  else{
    return false;
  }
  
}
void packet_conversion(individual_switch_data_rx input_packat){

}

// void mode_to_off(controller_data_struct *controllerData, size_t switchID){
//   controllerData[switchID].duty_cycle=0;
//   controllerData[switchID].max_duty_cycle=0;
//   controllerData[switchID].mode=155;

//   controllers[switchID]=&manualPool[switchID];
//   controllers[switchID] ->update(controllerData[switchID].target,
//     controllerData[switchID].temperature,0);
// }
void control_loop(){

  bool all_switches_off =false; 
  const individual_switch_data_rx default_off_package = {
        .regist=0x01,
        .switchID=0x00,
        .mode=155,
        .temperature=20,
        .target=20,
        .crc8=0}; //crc8 should get actual value but doesnt really matter as its not checked again
  individual_switch_data_rx recievedData; //Data from i2c task will be recieved in this variable
  
  controller_data_struct controllerData[number_switches];
  controllerData[0].pin=SD_Card_PIN; // 5
  controllerData[1].pin=pressure_ch_PIN; //6
  controllerData[2].pin=outlet_PIN; //7
  controllerData[3].pin=inlet1_PIN; //8
  controllerData[4].pin=inlet2_PIN; //9
  controllerData[5].pin=plt1_PIN; // 10
  controllerData[6].pin=plt2_PIN; //17
  controllerData[7].pin=backup_PIN;// 18

  //Timeout stuff 
  int64_t timestep_us= 1000000; //1 second  Need to make sure we recieve temperature data such that timestep>=measuring period otherwise PID doesnt work properly
  int64_t current_time = esp_timer_get_time();
  int64_t last_recieved_time = esp_timer_get_time();


  for(int i=0; i<number_switches;i++){

  }
  while(1){

    current_time=esp_timer_get_time();
    //gets data from i2c receive task
    if(xQueueReceive(dataQueue,&recievedData,0)==pdTRUE){
      last_recieved_time=esp_timer_get_time();
      //Handles different packets
      switch(recievedData.regist){ 
        case packet_type_indvidual_switch:{ //Default case ie normal data packet
          controllerData[recievedData.switchID]={
            .temperature = recievedData.temperature/100.0f,
            .target      = recievedData.target/100.0f,
          };

          //sets up various controllers
          if(controllerData[recievedData.switchID].mode==0) //bangbang
          {
            controllerData[recievedData.switchID].mode=recievedData.mode;
            controllers[recievedData.switchID]=&bangPool[recievedData.switchID];
            controllerData[recievedData.switchID].max_duty_cycle=100; //This should be changed to a variable or array
          }
          else if(controllerData[recievedData.switchID].mode==1)//PID
          {
            controllers[recievedData.switchID]=&PIDPool[recievedData.switchID];
            //Resets PID if you've changed to it will otherwise mess things up
            if (controllerData[recievedData.switchID].mode!=recievedData.mode) 
            {
              controllerData[recievedData.switchID].mode=recievedData.mode;
              controllers[recievedData.switchID]->reset();
            }
            
            controllerData[recievedData.switchID].max_duty_cycle=100; //This should be changed to a variable or array        
          }
          else if(controllerData[recievedData.switchID].mode>=155) //manual //recievedData[i].mode<=255 not needed as its always true for uint8
          {
            controllerData[recievedData.switchID].mode=recievedData.mode; //Sets mode for the controllerDta
            controllers[recievedData.switchID]=&manualPool[recievedData.switchID];
            controllerData[recievedData.switchID].max_duty_cycle=recievedData.mode-155; //As 155 =0% dutycycle and 255=100%
            controllers[recievedData.switchID]->update(0,0,controllerData[recievedData.switchID].max_duty_cycle); //Sets dutycycle to the proper one
          }
          else{
            //error case
            break;
          }
          controllerData[recievedData.switchID].last_updated=esp_timer_get_time(); //For timeout later
          break;
        }
          
        case packet_stop_all: //This will need a special command to stop. Or just reset from main to reinitilize defaults //I can also change it so that it needs to be sent every loop to be viable
          all_switches_off=true;
          break;

        case packet_resume_all:
          //If a switch was in PID and has been paused for a while it needs to reset when reentering PID
          for(int i=0;i<number_switches;i++){ 
            controllers[i]->reset();
          }
          all_switches_off=false;
          break;
      }
    }


    //Stuff below this needs fixing
    //Need to add if so that PID only triggers once every x ms (is dependant on how often you get data but needs to be constant)
    //Main control stuff
    
    for(int i=0;i<number_switches;i++)
    {
      //If timeout or off set dutycycle to 0 else call controller class and set dutycycle
      if(
        abs(current_time-last_recieved_time)>control_timeout_us || //Global i2c timeout no packat has been recieved in a while
        abs(current_time-controllerData[i].last_updated)>control_timeout_us || //Local timeout no data for this switch has been recieved
        all_switches_off //All switches off
      )
      { //Error case/Off case
        controllerData[i].duty_cycle=0; 
      }
      //This is a case in case that temperature is out of bounds ie assuming sensor is broken
      else if(controllerData[i].temperature<temperature_lower_bound || //<-50C
        controllerData[i].temperature>temperature_higher_bound || //>120C
        recievedData.temperature==-9999)  //Idunno i believe its error code
      {
        controllerData[i].duty_cycle=0;
      }
      else
      {
        controllerData[i].duty_cycle=controllers[i]->update(controllerData[i].target,controllerData[i].temperature,controllerData[i].max_duty_cycle);
      }

      bool is_on = duty_cycle_to_on_off(controllerData[i].duty_cycle, timestep_us, current_time); //should add proper timestep that syncs with looptime
      gpio_set_level(controllerData[i].pin, is_on);
    }
    
    
    vTaskDelay(1); //Otherwise will cause error //Doesnt need to be delay but needs to be something that can pass task to other task
  }
}


//libaries
#include "string.h"
#include "driver/i2c.h"
#include "esp_log.h"

//other parts of project
#include "config.h"
#include "i2c_com.h"



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
      reset();  //The reset may be removed here depending on what we want from chagneParams i think we'll want it here
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

const int number_controllers=8;
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
  manual_control(50), //0
  manual_control(50),
  manual_control(50),
  manual_control(50),
  manual_control(50),
  manual_control(50),
  manual_control(50),
  manual_control(50) //7
};

Controller* controllers[number_controllers]{ //Default values for controolers may change to &manualpool or &PIDpool
  &bangPool[0],
  &bangPool[1],
  &bangPool[2],
  &bangPool[3],
  &bangPool[4],
  &bangPool[5],
  &bangPool[6],
  &bangPool[7]
};
float dutyCycles[8];

//Actual variables
unsigned long previous_time=0;
float timeStep=1; 
unsigned long current_time;
//placeholders for actual implementation
float desired_values[number_controllers];
float meas_values[number_controllers];


bool change_controller=false; //Command for when you want to change controltype or initilize controller with new values
bool initilize_val=false; //Command for initilizing new values for a specific controltype as decided by controltype
int num_controller; //Chooses which controller to change ie nr 1 or nr 2 etc
char control_type;  //chooses which control type manual PID or bangbang

float D_cycle; float dead_zone; bool cool=false; // For changeParams in bang
float p; float i; float d; float timestep; bool cool1=false; //For changeParams in PID
float duty_cycle1; //for changeParams in manual




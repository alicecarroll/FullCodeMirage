// libaries
#include "esp_log.h"
#include "string.h"


// other parts of project
#include "config.h"
#include "control.h"
#include "esp_timer.h"
#include "i2c_com.h"

// Things that should be sent to main MCU: Modes of all pin ie Pin1 manual, pin2
// PID et.

// Things main MCU should be able to send. Sensor data for all sensors.

// Need to add error handling to the whole program
class Controller {
public:
  virtual float
  update(float desired_value, float meas,
         float dutyCycle) = 0; // Dutycycle only used for manual mode. Its an
                               // extra variable to prevent sending bad data in
                               // case of wrong mode being on
  virtual void reset() = 0;
};

// Fix bang bang such that it has proper deadzon (current deadzone is not
// correct)
class Bang : public Controller // ON/OFF controller
{
private:
  float dutyOn;   // Max dutyCycle
  float deadZone; // The distance from desired temp before turning things on/off
  bool cooler;
  float duty_Cycle = 0; // Current dutyCycle

public:
  Bang(float D_cycle, float dead_zone, bool cool) {
    dutyOn = D_cycle;
    deadZone = dead_zone;
    cooler = cool;
  }

  float update(float desired_value, float meas, float duty_cycle_on) override {
    dutyOn = duty_cycle_on;
    // Cooler
    if (cooler && (meas >= (desired_value + deadZone))) { // with cooler
      duty_Cycle = dutyOn;
    } else if (cooler && (meas <= (desired_value - deadZone))) {
      duty_Cycle = 0;
    }

    // heater
    if (!cooler && (meas <= (desired_value - deadZone))) { // with heater
      duty_Cycle = dutyOn;
    } else if (!cooler && (meas >= (desired_value + deadZone))) {
      duty_Cycle = 0;
    }
    return duty_Cycle;
  }

  void reset() override { duty_Cycle = 0; }

  void changeParams(float D_cycle, float dead_zone,
                    bool cool) { // changes the parameters for Bang controller
    dutyOn = D_cycle;
    deadZone = dead_zone;
    cooler = cool;
  }
};

class PID_control : public Controller {
private:
  float kp, ki, kd;
  float integral = 0;
  float dt; // in seconds
  bool cooler;
  // Derivative part variables
  float prev_error = 0;
  float filteredDerivative = 0; // Used to reduce noise impace on derivative
  bool firstLoop = true;        // Used to make proper values for derivative

public:
  PID_control(float p, float i, float d, float timestep, bool cool) {
    kp = p;
    ki = i;
    kd = d;
    dt = timestep;
    cooler = cool;
    reset();
  }

  void reset() override {
    integral = 0;
    prev_error = 0;
    filteredDerivative = 0;
    firstLoop = true;
  }

  float update(float desired_value, float meas,
               float max_duty_cycle) override { // Runs the PID control.

    float windup_limit; // The windup limit needs to be dimensioned properly !!
    const float tau = 2;
    float derivative, error;

    // makes sure dt isnt 0 or less than 0 because negative time is made up by
    // mathematicians
    if (dt <= 0) {
      return 0;
    }

    // Makes sure windup limit is properly dimensioned. This implementation
    // ensures that ki*I<=max_duty_cycle
    if (ki > 0) {
      windup_limit = max_duty_cycle /
                     ki; // The windup limit needs to be dimensioned properly !!
    } else {
      windup_limit = 0;
    }

    error = desired_value - meas; // Error
    // Switches error so that positive output always is equivilant to ON for
    // actuator
    if (cooler) {
      error = -error;
    }

    // Used for derivative part //Makes sure proper derivative values if in
    // first loop
    if (firstLoop) {
      filteredDerivative = 0;
      prev_error = error;
      firstLoop = false;
    }

    derivative = (error - prev_error) / dt;
    prev_error = error;
    filteredDerivative +=
        dt / (tau + dt) *
        (derivative -
         filteredDerivative); // Filtering of derivative to reduce noise

    float newIntegral = integral + error * dt;

    // Anti windup. // This can be implemented better if needed
    if (newIntegral > windup_limit) {
      newIntegral = windup_limit;
    } else if (newIntegral < -windup_limit) {
      newIntegral = -windup_limit;
    }

    float PID_output = kp * error + ki * newIntegral +
                       kd * filteredDerivative; // Actual PID calculation

    // Converts output to duty cyvle and conditional integral (Conditional
    // integral prevents integral from increasing if we've reached saturation)
    if (PID_output > max_duty_cycle) {
      PID_output = max_duty_cycle;
      if (error < 0) {
        integral = newIntegral;
      }
    } // This part might be changed such that it handles negatives to be able to
      // use cooler on same PID
    else if (PID_output < 0) {

      PID_output = 0;
      if (error > 0) {
        integral = newIntegral;
      }
    } // IE returns 100- (-100) where negative corresponds to cooler
    else {
      integral = newIntegral;
    }

    return PID_output;
  }

  void changeParams(float p, float i, float d, float timestep,
                    bool cool) { // Changeng the values for PID control
    kp = p;
    ki = i;
    kd = d;
    dt = timestep;
    cooler = cool;
    reset(); // The reset may be removed here depending on what we want from
             // chagneParams i think we'll want it here
  }

  void set_dt(float timestep) { dt = timestep; }
};

class manual_control : public Controller {
private:
  float D_cycle;

public:
  manual_control(float Duty_Cycle) { D_cycle = Duty_Cycle; }
  // the
  float update(float desired_value, float meas, float Duty_Cycle2)
      override { // duty cycle2 is only it's own variable for safety it might be
                 // changed to use space of desired value of meas. In that case
                 // dutycycle 2 can be removed from controller class
    D_cycle = Duty_Cycle2;
    return Duty_Cycle2;
  }
  void reset() override {
    // should be empty
  }
  void changeParams(float Duty_Cycle2) { D_cycle = Duty_Cycle2; }
};

// int d_cycle_heat_control(int mode, double value, double desired_value, int
// manual_d_cycle){  //will remove
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

const int number_controllers = number_switches;
int64_t timestep_us = 1000000; // 1 second  Need to make sure we recieve
                               // temperature data such that timestep>=measuring
                               // period otherwise PID doesnt work properly
float timestep_s = static_cast<float>(timestep_us) / 1000000.0f;
// useful things
// These controllers could probably be initilized better
Bang bangPool[number_controllers]{
    Bang(100, 3, false), // Maxdutycycle, deadzone+-, iscooler t/f
    Bang(100, 3, false), 
    Bang(100, 3, false),
    Bang(100, 3, false),
    Bang(100, 3, false), 
    Bang(100, 3, true),  
    Bang(100, 3, true),
    Bang(100, 3, false)};

// Some things for controllers
PID_control PIDPool[number_controllers]{
    PID_control(2, 0.1, 3, timestep_s, false), // 0 //P, I , D, timestep (s)
    PID_control(2, 0.1, 3, timestep_s, false),
    PID_control(2, 0.1, 3, timestep_s, false),
    PID_control(2, 0.1, 3, timestep_s, false),
    PID_control(2, 0.1, 3, timestep_s, false),
    PID_control(2, 0.1, 3, timestep_s, true),
    PID_control(2, 0.1, 3, timestep_s, true),
    PID_control(2, 0.1, 3, timestep_s, false), // 7
};

manual_control manualPool[number_controllers]{
    manual_control(0), // 0
    manual_control(0), 
    manual_control(0), 
    manual_control(0),
    manual_control(0), 
    manual_control(0), 
    manual_control(0),
    manual_control(0) // 7
};

Controller *controllers[number_controllers]{
    // Default values for controolers may change to &manualpool or &PIDpool
    &manualPool[0], 
    &manualPool[1], 
    &manualPool[2],
    &manualPool[3], 
    &manualPool[4],
    &manualPool[5], // Coolers should be in bangbang during normal operation
                    // while heaters should be in PID or bangbang depending on
                    // how often they're in manual. If they're often in manual
                    // use bang bang
    &manualPool[6], 
    &manualPool[7]};
float temperature_lower_bound = -80;  // Celcius
float temperature_higher_bound = 120; // celcius

// Actual variables
//  unsigned long previous_time=0;
//  float timeStep=1;
//  unsigned long current_time;
//  //placeholders for actual implementation
//  float desired_values[number_controllers];
//  float meas_values[number_controllers];

// bool change_controller=false; //Command for when you want to change
// controltype or initilize controller with new values bool initilize_val=false;
// //Command for initilizing new values for a specific controltype as decided by
// controltype int num_controller; //Chooses which controller to change ie nr 1
// or nr 2 etc char control_type;  //chooses which control type manual PID or
// bangbang

// float D_cycle; float dead_zone; bool cool=false; // For changeParams in bang
// float p; float i; float d; float timestep; bool cool1=false; //For
// changeParams in PID float duty_cycle1; //for changeParams in manual

// Converts a dutycycle 0-100% to on off logic together with the period
bool duty_cycle_to_on_off(float dutyCycle, const int64_t period_us,
                          int64_t currentTime_us) {
  if (dutyCycle > 100) { // Duty cycle should not be above 100
    ESP_LOGW("Control", "Duty cycle above 100 value: %.2f", dutyCycle);
    dutyCycle = 100.0f;
  }
  if (dutyCycle < 0) { // Duty cycle shoyld not be below 0
    ESP_LOGW("Control", "Duty cycle below 0 value: %.2f", dutyCycle);
    dutyCycle = 0.0f;
  }
  if (period_us <
      2) { // period should not be below 2us otherwise function doesnt do shit
    ESP_LOGW("Control",
             "Period less than 1us should not be used current period %" PRId64,
             period_us);
    return false;
  }

  int64_t onTime = static_cast<int64_t>(period_us * dutyCycle / 100);

  // Checks if it should be on otherwise off
  if (currentTime_us % period_us < onTime) {
    return true;
  } else {
    return false;
  }
}
void packet_conversion(individual_switch_data_rx input_packat) {}

// void mode_to_off(controller_data_struct *controllerData, size_t switchID){
//   controllerData[switchID].duty_cycle=0;
//   controllerData[switchID].max_duty_cycle=0;
//   controllerData[switchID].mode=155;

//   controllers[switchID]=&manualPool[switchID];
//   controllers[switchID] ->update(controllerData[switchID].target,
//     controllerData[switchID].temperature,0);
// }
void control_loop(void *pvParameters) {

  bool first_loop = true;
  bool all_switches_off = false;
  const individual_switch_data_rx default_off_package = {
      .regist = 0x01,
      .switchID = 0x00,
      .mode = 155,
      .temperature = 20,
      .target = 20,
      .crc8 = 0}; // crc8 should get actual value but doesnt really matter as
                  // its not checked again
  individual_switch_data_rx
      recievedData; // Data from i2c task will be recieved in this variable

  controller_data_struct controllerData[number_switches];

  controllerData[0].pin = SD_Card_PIN;     // 5
  controllerData[1].pin = pressure_ch_PIN; // 6
  controllerData[2].pin = outlet_PIN;      // 7
  controllerData[3].pin = inlet1_PIN;      // 8
  controllerData[4].pin = inlet2_PIN;      // 9
  controllerData[5].pin = plt1_PIN;        // 10
  controllerData[6].pin = plt2_PIN;        // 17
  controllerData[7].pin = backup_PIN;      // 18

  // Timeout stuff
  int64_t current_time = esp_timer_get_time();
  int64_t last_recieved_time = esp_timer_get_time();
  int64_t last_comtroller_activation = esp_timer_get_time();
  // Used for sending data back to master
  uint8_t error[number_switches] = {};
  uint8_t lastSwitch = 7;
  bool newData = false;

  static uint8_t dataBuffer[8];
  individual_switch_data_tx sendPacket = {.switchID = 8,
                                          .mode = 155,
                                          .D_cycle = 100,
                                          .target = -99.0f,
                                          .status = 0,
                                          .global_mode = 0x01};

  for (int i = 0; i < number_switches; i++) {
    
  }
  while (1) {

    current_time = esp_timer_get_time();

    // receiving part of the program
    // gets data from i2c receive task
    if (xQueueReceive(dataQueue, &recievedData, pdMS_TO_TICKS(10)) ==
        pdTRUE) { // This needs some delay to be able to yield to other tasks
                  // otherwise error
      last_recieved_time = esp_timer_get_time();
      newData = true;
      // Handles different packets
      switch (recievedData.regist) {
      case packet_type_indvidual_switch: { // Default case ie normal data packet
        controllerData[recievedData.switchID].temperature =
            recievedData.temperature / 100.0f;
        controllerData[recievedData.switchID].target =
            recievedData.target / 100.0f;

        lastSwitch = recievedData.switchID;
        // ESP_LOGI("Control", " Switch: %d, Mode: %d", recievedData.switchID,
        // recievedData.mode); sets up various controllers
        if (recievedData.mode == 0) // bangbang
        {
          controllerData[recievedData.switchID].mode = recievedData.mode;
          controllers[recievedData.switchID] = &bangPool[recievedData.switchID];
          controllerData[recievedData.switchID].max_duty_cycle =
              100; // This should be changed to a variable or array
        } else if (recievedData.mode == 1) // PID
        {
          controllers[recievedData.switchID] = &PIDPool[recievedData.switchID];
          // Resets PID if you've changed to it will otherwise mess things up
          if (controllerData[recievedData.switchID].mode != recievedData.mode) {
            controllerData[recievedData.switchID].mode = recievedData.mode;
            controllers[recievedData.switchID]->reset();
          }

          controllerData[recievedData.switchID].max_duty_cycle =
              100; // This should be changed to a variable or array
        } else if (recievedData.mode >=
                   155) // manual //recievedData[i].mode<=255 not needed as its
                        // always true for uint8
        {
          controllerData[recievedData.switchID].mode = recievedData.mode; // Sets mode for the controllerDta
          controllers[recievedData.switchID] = &manualPool[recievedData.switchID];
          controllerData[recievedData.switchID].max_duty_cycle = recievedData.mode - 155; // As 155 =0% dutycycle and 255=100%
          controllerData[recievedData.switchID].duty_cycle =  recievedData.mode - 155;
          controllers[recievedData.switchID]->update(0, 0,controllerData[recievedData.switchID].max_duty_cycle); // Sets dutycycle to the proper one
          
        } else {
          // error case mode is in range from 2-154
          break;
        }
        controllerData[recievedData.switchID].last_updated =
            esp_timer_get_time(); // For timeout later
        break;
      }

      case packet_stop_all: // This will need a special command to stop. Or just
                            // reset from main to reinitilize defaults //I can
                            // also change it so that it needs to be sent every
                            // loop to be viable
        all_switches_off = true;
        break;

      case packet_resume_all:
        // If a switch was in PID and has been paused for a while it needs to
        // reset when reentering PID
        for (int i = 0; i < number_switches; i++) {
          controllers[i]->reset();
        }
        all_switches_off = false;
        break;
      }
    }

    // Need to add if so that PID only triggers once every x ms (is dependant on
    // how often you get data but needs to be constant) Main control stuff
    // //TODO implement staggered dutyCycles

    // Loops through switches
    if (current_time - last_comtroller_activation >
        timestep_us) // Makes sure controllers arent running faster than once
                     // every timestep us
    {
      last_comtroller_activation = esp_timer_get_time();
      for (int i = 0; i < number_switches; i++) {

        // If timeout or off set dutycycle to 0 else call controller class and
        // set dutycycle
        if ((current_time - last_recieved_time) >
                control_timeout_us || // Global i2c timeout no packat has been
                                      // recieved in a while
            (current_time - controllerData[i].last_updated) >
                control_timeout_us || // Local timeout no data for this switch
                                      // has been recieved
            // controllerData[i].last_updated==0 ||
            all_switches_off // All switches off
        ) {                  // Error case/Off case
          controllerData[i].duty_cycle = 0;
        }
        // This is a case in case that temperature is out of bounds ie assuming
        // sensor is broken
        else if (controllerData[i].temperature <
                     temperature_lower_bound || //<-80C
                 controllerData[i].temperature >
                     temperature_higher_bound || //>120C
                 controllerData[i].temperature ==
                     -99.0f) // Idunno i believe its error code
        {
          controllerData[i].duty_cycle = 0;
        } else {
          //ESP_LOGI("Control:", "Normal case");
          controllerData[i].duty_cycle = controllers[i]->update(
              controllerData[i].target, controllerData[i].temperature,
              controllerData[i].max_duty_cycle);
        }
        ESP_LOGI("Control:", "Switch: %d,  Mode: %d, DutyCycle %d, Temperature: %g", i, controllerData[i].mode, controllerData[i].duty_cycle, controllerData[i].temperature);
        
      }
      //ESP_LOGI("Control:", "Switch 2. Mode: %d, DutyCycle: %d Target %g, Temperature %g", controllerData[2].mode, 
      //  controllerData[2].duty_cycle,
      //  controllerData[2].target,
      //  controllerData[2].temperature);
    }

    // sets level of output pins
    for (int i = 0; i < number_switches; i++) {
      // Sets on/off for pin
      bool is_on = duty_cycle_to_on_off(controllerData[i].duty_cycle, timestep_us, current_time); // should add proper timestep that syncs with looptime
      gpio_set_level(controllerData[i].pin, static_cast<uint32_t>(is_on));
      // ESP_LOGI("Control: ", "On/off %d", static_cast<uint32_t>(is_on));
    }

    // Packet with data to send
    sendPacket = {
        .switchID = lastSwitch,
        .mode = controllerData[lastSwitch].mode,
        .D_cycle = controllerData[lastSwitch].duty_cycle,
        .target = controllerData[lastSwitch].target,
        .status = error[lastSwitch], // Fix so this error thing actually does its job
                               // should be done with giving error corresponding
                               // with numbers 2^x and then using or on error ie
                               // error1 |error2 (basically does the same as
                               // adding them but with safety)
        .global_mode = 0x00};

    if (all_switches_off) {
      sendPacket.global_mode = packet_stop_all;
    } else if (recievedData.regist == packet_resume_all) {
      sendPacket.global_mode = packet_resume_all;
    } else if (recievedData.regist == packet_type_indvidual_switch) {
      sendPacket.global_mode = packet_type_indvidual_switch;
    } else {
      sendPacket.global_mode = 0;
    }

    data_pack_indvidual_switch(&sendPacket, dataBuffer);
    i2c_data_evt send;
    send.length = 8;

    memcpy(send.data, dataBuffer, send.length);

    // should be if newdata changed for finding errors
    //  if(
    //    newData || // Makes sure that the buffer will have semi_recent data
    //    first_loop) //Makes sure that the buffer always has data
    if (true) {
      xQueueOverwrite(dataQueue_slave_tx, &send); // Writes to send task
      newData = false;
    }

    first_loop = false;
  }
}

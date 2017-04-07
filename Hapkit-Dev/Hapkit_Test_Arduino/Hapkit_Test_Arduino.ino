#include "Hapkit_Test_Arduino.h"
#include "Hapkit_Kinematics_Simulation.h"

// communication variable declarations
byte cmd_code;
byte reply_code = 3;

byte device_function;
float incoming_data[IN_PARAMETERS];
float outgoing_data[OUT_DATA];

// actuator struct definition
actuator Motor;

// timing and debug functions
int timer1_flag = 0;
int ledPin = 13;

/**
 * @brief    Main setup function, defines parameters and hardware setup
 */
void setup() {
  noInterrupts();
  
  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, LOW);
  standard_hapkit_setup(&Motor);
  kinematics_simulation_setup();
  Serial.begin(57600);

  interrupts();
}


/**
 * @brief    Main loop function
 */
void loop() {

  // timer1 interrupt flag operation (frequency at 1.008kHz)
  if(timer1_flag){
    timer1_flag = 0;

    if(Serial.available() > 0){

      cmd_code = Serial.read();

      switch(cmd_code){
        case 0: // setup parameters code
          device_function = setup_parameters(incoming_data);
          freq = incoming_data[0];
          amplitude = incoming_data[1];
          reply_code  = 0;
          break;
        case 1: // send simulation state code
          reply_code = 1;
          break;
        default:
          break;
      }
      
    }
    else{

      // call sim and kin stuff here

      // update sensor position
      updateSensorPosition();
      
      // compute position
      computePosition();
      
      // rendering algorithm
      renderingAlgorithm(device_function, &Motor);
      

      switch(reply_code){
        case 0: // reply with initialization information
          outgoing_data[0] = freq;
          outgoing_data[1] = amplitude;
          send_data(device_function, outgoing_data);
          reply_code = 3;
          break;
        case 1: // reply with simulation state information
          outgoing_data[0] = force;
          outgoing_data[1] = Tp;
          send_data(device_function, outgoing_data);
          reply_code = 3;
          break;
        default:
          break;
      }
    }
  }
}


ISR(TIMER1_OVF_vect){
  TCNT1 = TIMER1_COUNTER;
  timer1_flag = 1;
}



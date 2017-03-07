/**
 *******************************************************************************
 * @file       Pantagraph_Draw_Float_Arduino.ino
 * @author     
 * @version    V0.0.1
 * @date       14-February-2017
 * @brief      Prototype tests for encoder input for Pantagraph application
 *******************************************************************************
 * @attention
 *
 *
 *******************************************************************************
 */

/* includes ********************************************************************/
#include <Encoder.h>
#include "Pantagraph_Draw_Device_Arduino.h"


/* Encoder declarations ************************************************************************/
Encoder Enc1(ENCPIN1_1, ENCPIN1_2);
Encoder Enc2(ENCPIN2_1, ENCPIN2_2);
Encoder Enc3(ENCPIN3_1, ENCPIN3_2);
Encoder Enc4(ENCPIN4_1, ENCPIN4_2);


/* Actuator parameter declarations *************************************************************/
actuator Motor_1;
actuator Motor_2;
actuator Motor_3;
actuator Motor_4;


/* Actuator Status and Command *****************************************************************/

/* Determines which motors are to be setup for use */
byte motors_active[TOTAL_ACTUATORS]; 

/* Command code definition for board control */
byte cmd_code;       

/* Number of motors actively setup and used */
int  number_motors;  

/* Read request flag, sends encoder information when activated */
bool read_request = false;


/* Iterator and debug definitions **************************************************************/
long lastPublish = 0;
int  ledPin = 13;


/* main setup and loop block  *****************************************************************/

/**
 * @brief    Main setup function, defines parameters and hardware setup
 * @note     None
 * @param    None
 * @return   None 
 */
void setup() {
  /* Sets up onboard Arduino LED for testing purposes */
  pinMode(ledPin, OUTPUT);
  
  /* Initialize the LED to be off */
  digitalWrite(ledPin, LOW);

  /* Start serial communication on the Arduino Native port. 
   * Note: data parameters is meaningless due to USB emulation of Native port, data rate appears 
   *       to follow USB 2.0 rates.
   */
  SerialUSB.begin(0);
}


/**
 * @brief    Main loop function
 * @note     None
 * @param    None
 * @return   None 
 */
void loop() {

  /* Setup loop to operate at 20 kHz */
  if(micros() - lastPublish >= 50){

    lastPublish = micros();

    /* Read incoming serial data */    
    if(SerialUSB.available() > 0){   
      
      /* read command code - first byte of  sent serial data*/ 
      cmd_code = SerialUSB.read();
      
      /* act accordingly based on command code*/ 
      switch(cmd_code){

        /* Setup case, determine number of actuators, and active actuators */
        case 0:
          number_motors = communication_setup(motors_active);
          break; 

        /* Setup actuatur case, setup the actuators that are to be used */
        case 1:
          setup_actuators(number_motors);
          break;
        
        /* Read request, set read encoder request flag to true */
        case 2:
          read_request = true;
          break;
        
        default:
          break;
      }
    }
    /* When Serial line availalbe, write requested data */
    else{

      /* Read request for sending encoder information */
      if(read_request){

        /* Send encoder information of all motors */
        read_encoders(number_motors);

        /* Reset read_request flag to not send stale data */
        read_request = false;
        
      }
    }
  }
}


/* Extended Setup function *****************************************************************/

/**
 * @brief    Sets up actuators that are to be used based on initial activation command
 * @note     Function calls subsequent functions which individually sets up each actuator
 * @param    active_motors: number of actuators that are to be activated
 * @return   None 
 */
void setup_actuators(int active_motors){

  /* declare parameter array for each actuator */
  byte actuator1[4*ACTUATOR_PARAMETERS];
  byte actuator2[4*ACTUATOR_PARAMETERS];
  byte actuator3[4*ACTUATOR_PARAMETERS];
  byte actuator4[4*ACTUATOR_PARAMETERS];

  /* recieve and parse actuator parameters for each actuator */
  read_actuator_parameters(actuator1, actuator2, actuator3, actuator4, active_motors, motors_active);
  
  int j = 0;
  /* Cycle through all possible actuators and activate relevant actuators */
  for(int i = 0; i < TOTAL_ACTUATORS; i++){
      
    if(motors_active[i] > 0){
      
      switch (i){
        case 0:
          initialize_actuator(&Motor_1, Enc1, actuator1);
          break;
  
        case 1:
          initialize_actuator(&Motor_2, Enc2, actuator2);
          break;
  
        case 2:
          initialize_actuator(&Motor_3, Enc3, actuator3); 
          break;
  
        case 3:
          initialize_actuator(&Motor_4, Enc4, actuator4);
          break;
      }
    }
    
  } 
}


/* Extended Control function ------------------------------------------------------------*/

/**
 * @brief    Determine current angle seen by encoders and send data 
 * @note     Will only read active actuator encoder values
 * @param    len: number of motors active
 * @return   None 
 */
void read_encoders(int number_of_actuators){

   float encoder1, encoder2, encoder3, encoder4;

   int j = 0;
   for(int i = 0; i < TOTAL_ACTUATORS; i++){
     
     if(motors_active[i] > 0){

        switch(i){
          case 0:
            encoder1 = read_encoder(&Motor_1, Enc1);
            break;
            
          case 1:
            encoder2 = read_encoder(&Motor_2, Enc2);
            break;
            
          case 2:
            encoder3 = read_encoder(&Motor_3, Enc3);
            break;
            
          case 3:
            encoder4 = read_encoder(&Motor_4, Enc4);
            break;
        }   
     }
   }

   send_encoders_data(encoder1, encoder2, encoder3, encoder4, number_of_actuators, motors_active);
}

/* Physics/dynamics functions ---------------------------------------------------*/



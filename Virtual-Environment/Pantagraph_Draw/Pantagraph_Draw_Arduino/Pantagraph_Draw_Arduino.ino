/**
 *******************************************************************************
 * @file       Pantagraph_Draw_Arduino
 * @author     Yi Ding, Colin Gallacher
 * @version    V0.0.1
 * @date       02-February-2017
 * @brief      Prototype tests for encoder input for Pantagraph application
 *******************************************************************************
 * @attention
 *
 *
 *******************************************************************************
 */

/* includes ********************************************************************/
#include "hAPI_Data.h"
//#include "hAPI_data_packets.h"
#include <Encoder.h>

/* Data length defintions ******************************************************/
#define INIT_LEN    25
#define OUT_LEN     8

/* Encoder Pin definitions *****************************************************/
//#define ENCPIN2_1   28 // J2
//#define ENCPIN2_2   29
//#define ENCPIN1_1   24 // J3
//#define ENCPIN1_2   25 

/* Encoder declarations ********************************************************/
Encoder Enc1(ENCPIN1_1, ENCPIN1_2);
Encoder Enc2(ENCPIN2_1, ENCPIN2_2);

/* Encoder parameter declarations **********************************************/
long Enc1_offset, Enc1_positions, Enc1_resolution;
long Enc2_offset, Enc2_positions, Enc2_resolution;

/* Encoder angle values declarations *******************************************/
long th1_degree;
long th2_degree;

/* Serial data parameters ******************************************************/
//byte inData[INIT_LEN];
//byte outData[OUT_LEN];
byte segments[4];  //corresponds to a four byte data type 

/* misllaneous *****************************************************************/
long lastPublish = 0;
int  ledPin = 13;
bool ledState = LOW; 
long ledTimer = 0; 
long ledTimerLast = 0; 
int blinkTime = 100; 
int loopPeriod = 50; //microseconds 
/**
 * @brief    Main setup function, defines parameters and hardware setup
 * @note     None
 * @param    None
 * @return   None 
 */
void setup() {
  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, ledState);
  SerialUSB.begin(0);

  while (!SerialUSB){
	  ledTimer = millis(); 
	  if ((ledTimer - ledTimerLast) > blinkTime){
		  ledState = !ledState; 
		  digitalWrite(ledPin, ledState);
		  ledTimerLast = ledTimer; 
	  }
	 
  }

  digitalWrite(ledPin, LOW); 

}

/**
 * @brief    Main loop function, updates frame at perscribed frame rate
 * @note     None
 * @param    None
 * @return   None 
 */
void loop() {

  if(micros() - lastPublish >= loopPeriod){

    lastPublish = micros();

    if(SerialUSB.available() > 0){   
      SerialUSB.readBytes(inData, INIT_LEN);

      if(inData[0] == 0){        
          initialize_encoders();
      }
    }
    else{
      read_encoders();
      SerialUSB.write(outData, OUT_LEN);
    } 
  }
  
  switch (inData[0]) {
	  case 	SET_HARDWARE_PORTS:
	  // initlaize the actuator hardware when inData[0] equals 0
	  initialize_encoders();
	  break;
  case SET_HARDWARE_IC:
	  //initialize the actuator states when inData[0] equals 1
	  break;
  case RETURN_ANGLES:
	  //transmit the device angles when inData[0] equals 2

  case COMMAND_TORQUES:
	  //do something when inData[0] equals 3

  default:
	  // if nothing else matches, do the default
	  // default is optional
	  break;
  }

}


/* Initialization functions -----------------------------------------------------*/

/**
 * @brief    Initialize encoders to starting positions
 * @note     Currently under prototype
 * @param    None
 * @return   None 
 */
void initialize_encoders(void){
  for(int i = 0; i < 6; i++){
     
     ArrayCopy(inData, i*4+1, segments, 0, 4);
             
       switch(i){
         case 0:
           Enc1_offset = BytesToInteger(segments);
           break;
         case 1:
           Enc1_resolution = BytesToInteger(segments);
           break;
         case 2:
           Enc2_offset = BytesToInteger(segments);
           break;
         case 3:
           Enc2_resolution = BytesToInteger(segments);
           break;
		 case 4:
			 Enc3_offset = BytesToInteger(segments);
			 break;
		 case 5:
			 Enc3_resolution = BytesToInteger(segments);
			 break;
		 case 6:
			 Enc4_offset = BytesToInteger(segments);
			 break;
		 case 7:
			 Enc4_resolution = BytesToInteger(segments);
			 break;
        }    
   }

   Enc1.write(Enc1_offset * Enc1_resolution / 360);
   Enc2.write(Enc2_offset * Enc1_resolution  / 360);
     
   /* Test indication */     
   if(Enc1_offset == 180){
      digitalWrite(ledPin, HIGH);
   }
}


/* Physics/dynamics functions ---------------------------------------------------*/

/**
 * @brief    Determine current angle seen by encoders 
 * @note     Currently under prototype
 * @param    None
 * @return   None 
 */
void read_encoders(void){
   th1_degree = 360*Enc1.read()/Enc1_resolution;
   th2_degree = 360*Enc2.read()/Enc2_resolution;
      
   IntegerToBytes(th1_degree, segments);
   ArrayCopy(segments, 0, outData, 0, 4);

   IntegerToBytes(th2_degree, segments);
   ArrayCopy(segments, 0, outData, 4, 4);
}


/* Helper functions -------------------------------------------------------------*/

/**
 * @brief    Translates a 32-bit long type into an array of four bytes
 * @note     None
 * @param    val: 32-bit signed long value
 * @param    segments: array of four bytes
 * @return   None 
 */
void IntegerToBytes(long val, byte segments[]){
  segments[3] = (byte)((val >> 24) & 0xff);
  segments[2] = (byte)((val >> 16) & 0xff);
  segments[1] = (byte)((val >> 8) & 0xff);
  segments[0] = (byte)((val) & 0xff); 
}


/**
 * @brief    Translates an array of four bytes into a signed integer format
 * @note     None
 * @param    segment: the input array of four bytes
 * @return   Translated 32-bit signed long value 
 */
long BytesToInteger(byte segment[]){
  long val;
  
  val = (val | (segment[3] & 0xff)) << 8;
  val = (val | (segment[2] & 0xff)) << 8;
  val = (val | (segment[1] & 0xff)) << 8;
  val = (val | (segment[0] & 0xff)); 
  
  return val;
}


/**
 * @brief    Copies elements from one array to another
 * @note     None
 * @param    src: The source array to be copied from
 * @param    src_index: The starting index of the source array
 * @param    dest: The destination array to be copied to
 * @param    dest_index: The starting index of the destination array
 * @param    len: Number of elements to be copied
 * @return   None 
 */
void ArrayCopy(byte src[], int src_index, byte dest[], int dest_index, int len ){
  for(int i = 0; i < len; i++){
    dest[dest_index + i] = src[src_index + i];
  }
}



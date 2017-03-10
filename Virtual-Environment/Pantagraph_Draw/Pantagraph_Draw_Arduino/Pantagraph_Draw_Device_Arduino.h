/**
 *******************************************************************************
 * @file       Pantagraph_Draw_Float_Arduino.h
 * @author     
 * @version    V0.0.1
 * @date       14-February-2017
 * @brief      constants and helper functions for Pantagraph application
 *******************************************************************************
 * @attention
 *
 *
 *******************************************************************************
 */


/* Data length defintions ******************************************************/

/* maximim number of actuators available on board for control */
#define TOTAL_ACTUATORS         4 

/* number of control parameters per actuator */
#define ACTUATOR_PARAMETERS     2 


/* Encoder Pin definitions *****************************************************/
#define ENCPIN2_1   28 // J2
#define ENCPIN2_2   29

#define ENCPIN1_1   24 // J3
#define ENCPIN1_2   25

#define ENCPIN3_1   32 // J4
#define ENCPIN3_2   33 

#define ENCPIN4_1   36 // J5
#define ENCPIN4_2   37


/* Actuator Struct definitions **************************************************/
typedef struct motor{
	float Enc_offset;
	float Enc_resolution;
	
}actuator;


/* function definitions *********************************************************/
int communication_setup(byte motors[]);
void initialize_actuator(actuator *mtr, Encoder &Enc, byte parameters[]);

float read_encoder(actuator *mtr, Encoder &Enc);

void read_actuator_parameters(byte a1[], byte a2[], byte a3[], byte a4[], int number, byte actuators[]);
void send_encoders_data(float a1, float a2, float a3, float a4, int number, byte actuators[]);

void FloatToBytes(float val, byte segments[]);
float BytesToFloat(byte segments[]);
void ArrayCopy(byte src[], int src_index, byte dest[], int dest_index, int len );


/* Initialization functions -----------------------------------------------------*/

/**
 * @brief    Initialize serial communication based on motors in use
 * @note     Currently under prototype
 * @param    motors[]: motor status array, indicates which motor ports are active
 * @return   data_length: number of motors to be setup
 */
int communication_setup(byte motors[]){

  int data = sizeof(motors)/sizeof(motors[0]);

  int number_of_motors = 0;
  
  SerialUSB.readBytes(motors, data);

  for(int i = 0; i < data; i++){
    if(motors[i] > 0){
      number_of_motors++; // see how many motors are active   
    }
  }

  return number_of_motors;
}


/**
 * @brief    Initialize an actuator and a coorsponding Encoder for use 
 * @note     Currently under prototype
 * @param    *mtr: pointer to actuator struct for parameters access
 * @param    &Enc: Encoder object reference 
 * @param    parameters[]: actuator input parameters for one actuator
 * @return   none
 */
void initialize_actuator(actuator *mtr, Encoder &Enc, byte parameters[]){

  int i = 0;
  byte actuator_value[4];
  
  ArrayCopy(parameters, i, actuator_value, 0, 4);
  mtr->Enc_offset = BytesToFloat(actuator_value);
  i = i + 4; 

  ArrayCopy(parameters, i, actuator_value, 0, 4);
  mtr->Enc_resolution = BytesToFloat(actuator_value);

  Enc.write(mtr->Enc_offset * mtr->Enc_resolution / 360);
}


/* Control functions -------------------------------------------------------------*/

/**
 * @brief    read an individual encoder and pasre data for byte transmission
 * @note     Currently under prototype
 * @param    *mtr: pointer to actuator struct for parameters access
 * @param    &Enc: Encoder object reference 
 * @return   th_degrees: angle detected by encoder
 */
float read_encoder(actuator *mtr, Encoder &Enc){

  float th_degrees;
  th_degrees = 360.0 * Enc.read()/mtr->Enc_resolution;

  return th_degrees;
}


/* Communication functions ------------------------------------------------------*/

/**
 * @brief    Parses and recieves initial encoder values 
 * @note     Will log active actuators for use
 * @param    a1: encoder 1 setup value
 * @param    a2: encoder 2 setup value
 * @param    a3: encoder 3 setup value
 * @param    a4: encoder 4 setup value
 * @param    number: number of motors active
 * @param    actuators: active actuator positions
 * @return   None 
 */
void read_actuator_parameters(byte a1[], byte a2[], byte a3[], byte a4[], int number, byte actuators[]){

    /* Determine incoming setup parameters datalength */
  int data_length = number * 4 * ACTUATOR_PARAMETERS;

  /* Incoming parameters array */ 
  byte actuator_parameters[data_length];
  
  SerialUSB.readBytes(actuator_parameters, data_length);
  
  int j = 0;
  /* Cycle through all possible actuators and activate relevant actuators */
  for(int i = 0; i < TOTAL_ACTUATORS; i++){
      
    if(actuators[i] > 0){
      
      switch (i){
        case 0:
          ArrayCopy(actuator_parameters, j, a1, 0, 4*ACTUATOR_PARAMETERS);
          j = j + 4 * ACTUATOR_PARAMETERS;
          break;
  
        case 1:
          ArrayCopy(actuator_parameters, j, a2, 0, 4*ACTUATOR_PARAMETERS); 
          j = j + 4 * ACTUATOR_PARAMETERS;
          break;
  
        case 2:
          ArrayCopy(actuator_parameters, j, a3, 0, 4*ACTUATOR_PARAMETERS);
          j = j + 4 * ACTUATOR_PARAMETERS;
          break;
  
        case 3:
          ArrayCopy(actuator_parameters, j, a4, 0, 4*ACTUATOR_PARAMETERS);
          break;
      }
    }  
  } 
}


/**
 * @brief    Formats and sends encoder values over Serial
 * @note     Will only send encoder values of active actuators
 * @param    a1: encoder 1 value
 * @param    a2: encoder 2 value
 * @param    a3: encoder 3 value
 * @param    a4: encoder 4 value
 * @param    number: number of motors active
 * @param    actuators: active actuator positions
 * @return   None 
 */
void send_encoders_data(float a1, float a2, float a3, float a4, int number, byte actuators[]){

  byte segments[4];
  byte outData[number*4];
  int j = 0;
  
  for(int i = 0; i < TOTAL_ACTUATORS; i++){
     
     if(actuators[i] > 0){

        switch(i){
          case 0:
            FloatToBytes(a1, segments);
            ArrayCopy(segments, 0, outData, j, 4);
            j = j + 4;
            break;
            
          case 1:
            FloatToBytes(a2, segments);
            ArrayCopy(segments, 0, outData, j, 4);
            j = j + 4;
            break;
            
          case 2:
            FloatToBytes(a3, segments);
            ArrayCopy(segments, 0, outData, j, 4);
            j = j + 4;
            break;
            
          case 3:
            FloatToBytes(a4, segments);
            ArrayCopy(segments, 0, outData, j, 4);
            break;
        }   
     }
   }

   SerialUSB.write(outData, number*4);
}

/* Helper functions -------------------------------------------------------------*/

/**
 * Union definition for floating point and integer representation conversion
 */
typedef union{
  long val_l;
  float val_f;
} ufloat;

/**
 * @brief    Translates a 32-bit floating point into an array of four bytes
 * @note     None
 * @param    val: 32-bit floating point
 * @param    segments: array of four bytes
 * @return   None 
 */
void FloatToBytes(float val, byte segments[]){
  ufloat temp;

  temp.val_f = val;

  segments[3] = (byte)((temp.val_l >> 24) & 0xff);
  segments[2] = (byte)((temp.val_l >> 16) & 0xff);
  segments[1] = (byte)((temp.val_l >> 8) & 0xff);
  segments[0] = (byte)((temp.val_l) & 0xff);
}


/**
 * @brief    Translates an array of four bytes into a floating point
 * @note     None
 * @param    segment: the input array of four bytes
 * @return   Translated 32-bit floating point 
 */
float BytesToFloat(byte segments[]){
  ufloat temp;

  temp.val_l = (temp.val_l | (segments[3] & 0xff)) << 8;
  temp.val_l = (temp.val_l | (segments[2] & 0xff)) << 8;
  temp.val_l = (temp.val_l | (segments[1] & 0xff)) << 8;
  temp.val_l = (temp.val_l | (segments[0] & 0xff)); 

  return temp.val_f;
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

// Includes
#include <math.h>


// Parameter that define what environment to render
//#define ENABLE_VIRTUAL_WALL
//#define ENABLE_LINEAR_DAMPING
#define ENABLE_SPRING


/******************************************************************
 *  System variables: 
 */
// interrupt timer:
int timer1_counter = 0; 
// Pin declares:
int pwmPin = 5; // PWM output pin for motor 1
int dirPin = 8; // direction output pin for motor 1
int sensorPosPin = A2; // input pin for MR sensor
// Kinematic Parameters:
double rs = 0.073152;   //[m] length of Sector in m
double rp = 0.004191;   //[m] radius of motor drive in m
double rh = 0.075;      //[m]  length of handle in m
// Position tracking variables
int updatedPos = 0;     // keeps track of the latest updated value of the MR sensor reading
int rawPos = 0;         // current raw reading from MR sensor
int lastRawPos = 0;     // last raw reading from MR sensor
int lastLastRawPos = 0; // last last raw reading from MR sensor
int flipNumber = 0;     // keeps track of the number of flips over the 180deg mark
int tempOffset = 0;
int rawDiff = 0;
int lastRawDiff = 0;
int rawOffset = 0;
int lastRawOffset = 0;
const int flipThresh = 100;  // threshold to determine whether or not a flip over the 180 degree mark occurred
boolean maybeFlipped = false;
double OFFSET = 972-50;
double OFFSET_NEG = 50;
// Kinematics variables:
double xh = 0;           // Position of the handle [m]
double theta_s = 0;      // Angle of the sector pulley in deg
double xh_prev;          // Distance of the handle at previous time step
double xh_prev2;
double dxh;              // Velocity of the handle
double dxh_prev;
double dxh_prev2;
double dxh_filt;         // Filtered velocity of the handle
double dxh_filt_prev;
double dxh_filt_prev2;
// Force output variables
double force = 0;           			// Force at the handle
double Tp = 0;              			// Torque of the motor pulley
double duty = 0;            			// Duty cylce (between 0 and 255)
unsigned int output = 0;    			// Output command to the motor
// Calibration parameters:
double m = 0.0002292;
double b = 0;
// Communication variables:
// Processing variables
String xAPrint = "0";
String fAPrint = "0";
int constant = 10000;
int count = 0;
// Variables that can be changed by processing code:
int functionNumber = 0;        //the function that should be playing right now
double amplitude = 0;         // a number between 0 and 1 that represents the proportion of amplitude
double freq = 0;             // the number of cycles in half of the screen.

char serialInputBuffer[2]; 
int lengthInputBuffer = 2; //need to flush out the ENTIRE transmission! Otherwise it gets super confused


 /*****************************
  *  Haptic rendering variables: 
  */
// Parameter for virtual wall
double x_wall = 0.005;                 	// Position of the virtual wall
double k_wall = 400;                   	// Maximum stiffness of the virtual wall

// Parameter for linear damping
double b_linear = 15;                  	// Linear damping in N/m

// Parameter for spring:
double k_spring = 100;




/*************************************
 *  Position tracking Interrupt functions: DO NOT CHANGE
 */
void setTimerInterrupt( void )
{
  TCCR1A = 0;
  TCCR1B = 0;
  timer1_counter = 65474; // preload timer 65536-16MHz/256/1000Hz
  TCNT1 = timer1_counter;   // preload timer
  TCCR1B |= (1 << CS12);    // 256 prescaler 
  TIMSK1 |= (1 << TOIE1);   // enable timer overflow interrupt
}
ISR(TIMER1_OVF_vect)        // interrupt service routine 
{
  TCNT1 = timer1_counter;   // preload timer
  rawPos= analogRead(sensorPosPin);  //current raw position from MR sensor 1
  
  // Calculate differences between subsequent MR sensor readings
  rawDiff = rawPos - lastRawPos;          //difference btwn current raw position and last raw position
  rawOffset = abs(rawDiff);
  if (( rawOffset > flipThresh ) && (maybeFlipped == 0)) // we don't update the position if we are in the middle of debouncing
  {
    maybeFlipped = 1; 
  }
  else if (maybeFlipped == 1)
  {
    maybeFlipped = 0;
    lastRawDiff = rawPos - lastLastRawPos; //difference btwn current raw position and last last raw position
    lastRawOffset = abs(lastRawDiff);
    if (lastRawOffset > flipThresh) // so here the debouncing is that if the difference is consistently above the threshold, we add "Offset"
    {
      if(lastRawDiff > 0)  // check to see which direction the drive wheel was turning
      {        
        flipNumber--;              // cw rotation 
      } 
      else 
      {                     // if(rawDiff < 0)
        flipNumber++;              // ccw rotation
      }
    }
    updatedPos = (rawPos-OFFSET_NEG) + flipNumber*OFFSET; // need to update pos based on what most recent offset is   
  }
  else
  {
    updatedPos = (rawPos-OFFSET_NEG) + flipNumber*OFFSET; // need to update pos based on what most recent offset is 
  }
  // Update position record-keeping vairables
  lastLastRawPos = lastRawPos;
  lastRawPos = rawPos;

}


/*************************************
 *  Setup function: 
 */
void setup() 
{
  // Set up serial communication
   Serial.begin(9600);
   noInterrupts();
  
  // Set PWM frequency 
  setPwmFrequency(pwmPin,1); 
  
  // Input pins
  pinMode(sensorPosPin, INPUT); // set MR sensor pin to be an input

  // Output pins
  pinMode(pwmPin, OUTPUT);  // PWM pin for motor A
  pinMode(dirPin, OUTPUT);  // dir pin for motor A
  
  // Initialize motor 
  analogWrite(pwmPin, 0);     // set to not be spinning (0/255)
  digitalWrite(dirPin, LOW);  // set direction
  
  // Initialize position valiables
  lastLastRawPos = analogRead(sensorPosPin);
  lastRawPos = analogRead(sensorPosPin);
  flipNumber = 0;
  b = -(lastRawPos-OFFSET_NEG)*m;
  setTimerInterrupt( );
  interrupts();
}


// --------------------------------------------------------------
// Main Loop
// --------------------------------------------------------------
void loop()
{
  
  //*************************************************************
  //*** Section 1. Compute position in meters *******************
  //************************************************************ 
  // Compute the angle of the sector pulley (ts) in degrees based on updatedPos
  double theta_s = m*updatedPos + b; // m = 0.0131 [deg/pos], b = -8.504 [deg]  
  // Compute the position of the handle in m
  xh = rh*theta_s;
  // Calculate the velocity of the handle
  dxh = (double)(xh - xh_prev) / 0.001;
  // Calculate the filtered velocity of the handle using an infinite impulse response filter
  dxh_filt = 0.9*dxh + 0.1*dxh_prev; 
    
  // Record the position and velocity
  xh_prev = xh;
  dxh_prev = dxh;
 
  //*************************************************************
  //*** Section 2.  Rendering Algorithms (Place your own here!)********************
  //*************************************************************

switch (functionNumber)
   {
    case 0: // no force
       force = 0; 
     break;
     case 1: //virtual wall
       if (xh > x_wall)
       {
          force = -k_wall * (xh - x_wall);
       }
      else
      {
        force = 0;
      }
     break;
     case 2: //linear damping
       force = -b_linear * dxh_filt; 
      break;
     case 3:
       force = -k_spring*xh;
     break;
    default:
     force = 0;
   }
//update variables to send to software:
   xAPrint = String((int)(xh*constant));
   fAPrint = String((int)(force*500));



   //*************************************************************
  //*** Section 3.  Transform Force to Torque at motor ********************
  //*************************************************************
  Tp = rp/rs * rh * force;    // Compute the required motor pulley torque (Tp) to generate that force
 
  //*************************************************************
  //*** Section 4. Force output (do not change) *****************
  //*************************************************************
  
  // Determine correct direction for motor torque
  if(force > 0) { 
    digitalWrite(dirPin, HIGH);
  } else {
    digitalWrite(dirPin, LOW);
  }

  // Compute the duty cycle required to generate Tp (torque at the motor pulley)
  duty = sqrt(abs(Tp)/0.0183);

  // Make sure the duty cycle is between 0 and 100%
  if (duty > 1) {            
    duty = 1;
  } else if (duty < 0) { 
    duty = 0;
  }  
  output = (int)(duty* 255);   // convert duty cycle to output signal
  analogWrite(pwmPin,output);  // output the signal  

if (Serial.available() > 1) 
 {
        // read the incoming buffer:
        Serial.readBytesUntil(255,serialInputBuffer,lengthInputBuffer);
        functionNumber = serialInputBuffer[0];
        
  }
  
  if(count > 10) { //only send data up sometimes
    
    Serial.print(xAPrint);
    Serial.print(",");
    Serial.print(fAPrint);
    Serial.print("\n");
    
   
    count = 0;
  } else {
    count ++; 
  }




    
}

// --------------------------------------------------------------
// Function to set PWM Freq -- DO NOT EDIT
// --------------------------------------------------------------
void setPwmFrequency(int pin, int divisor) {
  byte mode;
  if(pin == 5 || pin == 6 || pin == 9 || pin == 10) {
    switch(divisor) {
      case 1: mode = 0x01; break;
      case 8: mode = 0x02; break;
      case 64: mode = 0x03; break;
      case 256: mode = 0x04; break;
      case 1024: mode = 0x05; break;
      default: return;
    }
    if(pin == 5 || pin == 6) {
      TCCR0B = TCCR0B & 0b11111000 | mode;
    } else {
      TCCR1B = TCCR1B & 0b11111000 | mode;
    }
  } else if(pin == 3 || pin == 11) {
    switch(divisor) {
      case 1: mode = 0x01; break;
      case 8: mode = 0x02; break;
      case 32: mode = 0x03; break;
      case 64: mode = 0x04; break;
      case 128: mode = 0x05; break;
      case 256: mode = 0x06; break;
      case 1024: mode = 0x7; break;
      default: return;
    }
    TCCR2B = TCCR2B & 0b11111000 | mode;
  }
}


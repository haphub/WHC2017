/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * This work was originally developed by Stanford Universty. The current version has been
 * modified by Steven Ding and Colin Gallacher. The current work is intented to function 
 * with the hAPI developed for the World Haptics Conference Student Innovation Challenge.
 * 
 * The following code is subject to the 
 * 
 * GNU General Public License v3.0 
 * GNU GPLv3
 * Permissions of this strong copyleft license are conditioned on making available 
 * complete source code of licensed works and modifications, which include larger 
 * works using a licensed work, under the same license. Copyright and license notices 
 * must be preserved. Contributors provide an express grant of patent rights.
 * 
  * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * 
 * Change Log: 
 * 
 * 
 * 1) The file was partitioned for readability and usability
 * 2) A communication protocol was established to receive 10 bytes from the computer and transmit 9 bytes from the device while 
 * maintaining needed information throughput
 * 3) The Timer1 was repurposed to regularize the sampling time of the haptic simulation to 1kHz. 
 * 4) Added the sensor reading to the interrupt to make sure we never miss sensor counts.
 * 5) Changed default virtual environment to be No Force
 * 
 *  * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 *  
 * Instructions:
 * 
 * This code archatecture consists of three files. The Hapkit_arduino.ino file handles the communication of data between an external 
 * computer handling the user interface. An API (hAPI) has been developed in Java to work in processing and handle communication between 
 * the Hapkit and the user interface. 
 * 
 * Users should place their programmed virtual environments inside of the Hapkit_Kinematics_Simulation folder in section 2. By modifying the 
 * included cases or including your own cases you have the ability to create virtual environments that will be simulated on the Hapkit.
 * 
 * The Hapkit recieves data from the external user interface that instructs the Hapkit about which virtual environment it should render as well 
 * as providing additional information about the amplitude and frequency (ask Melisa to better explain this as the desired use case as it was described to me as 
 * 'simulating math'.
 * 
 * Users can update which function they want to render by passing in the appropriate arguments from the user interface. The Hapkit expects to receive
 * a data structure consisting of [ byte arg 1, byte arg2, float arg3, float arg 4]. These arguments instruct (arg1) what type of communication the device  
 * is receiving, (arg2) what function the user wants to render virtually (arg3) the Amplitude and (arg4) the Frequency. The Hapkit then simulates the virtual environment dependent on the 
 * incoming arguments and will return a data packet consiting of [byte arg1, float arg2, float arg3], where arg1 returns the rendered function as a check, 
 * arg2 is the device angle/position, and arg3 is a force/torque. These values can be used in the user interface to program cool effects! 
 * 
 * The hAPI is the portal to the user interface on an external computer. It can handle the construction and respective communication   
 * of the data as well as setting up the respective paramters required to use a Haptic Paddle. 
 * 
 * A Processing example will be included so that users can see how the virtual environments can be programmed to interface with the Hapkit!
 * 
 * email crgallacher@gmail.com with any questions. 
 * 
 * Happy Coding :) 
 * 
 * 
 */

#include "Hapkit_arduino.h"
#include "Hapkit_Kinematics_Simulation.h"

byte      motors_active[4]; 
int       number_of_motors;  

actuator  Motor;

// communication variables
byte      cmd_code;
byte      device_function = 0;

float     outdata[2];
float     incoming_data[2];

bool      reply = false;

// timing and debug variables
int       timer1_flag = 0;
int       ledPin = 13;

//Ball model
typedef struct 
{
    float x;
    float mass;
    float velocity;
} Ball;
Ball ball;
//constant velocity when the ball bounces so to not introduce too much energy into the system
const float BALL_BOUNCE_VELOCITY = 0.02; //m/s? //MELISA SWITCH THIS I DON'T KNOW THE UNITS
const float GRAVITY = 0.0098; //m/s^2? MELISA DOUBLE CHECK UNITS

boolean startSim = false;
boolean applyForce = false;
int forceCounter = 0;

// You don't really want to touch any of this code as it just handles the setup of the device.
//The only part users may want to explore is the passing of variables commented in the code below. 

void setup() {
  noInterrupts();

  ball.x = 0;
  ball.velocity = 0;
  ball.mass = 0.005; //kg! MELISA DOUBLE CHECK UNITS

  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, LOW);
  standard_hapkit_setup(&Motor);
  kinematics_simulation_setup();
  Serial.begin(57600);
  //don't run simulation unless processing is active. This is important for dynamic simulation
  // in which graphics guide action. Helps protect device. Can change for debugging.
   startSim = false;
  applyForce = false;
  forceCounter = 0;
  
  interrupts();
}

void loop() {
  if(timer1_flag){
    timer1_flag = 0;
    //Serial.println(ball.x);
    if(Serial.available() > 0){

      reply = true;
      startSim = true;
      
      cmd_code = command_instructions(Serial.read(), &number_of_motors, motors_active);

      device_function = receive_data(incoming_data);

      freq = incoming_data[0];
      amplitude = incoming_data[1];
    }
    else{

      // compute position
      computePosition();
      
      // rendering algorithm
      //renderingAlgorithm(device_function, &Motor);
      
      // We're using a custom rendering algorithm
      //MELISA START HERE
      
      if (ball.x > xh)
      {
        //update with gravity
        ball.velocity -= GRAVITY*0.001; //MELISA DOUBLECHECK DIRECTION
        
        ball.x += ball.velocity*0.001;
         
      } else {
        //force = (ball.x - xh) * (-k_spring); //what do we want the ball to display when it hits?
        applyForce = true;
        ball.velocity = BALL_BOUNCE_VELOCITY;
        ball.x = ball.x + ball.velocity*0.001;
        
      }
      if (applyForce && startSim)
      {
        if (forceCounter < 3)
        {
          force = -5;
          forceCounter = forceCounter + 1;
        }
        else
        {
          force = 0;
          applyForce = false;
          forceCounter = 0;
        }
      }
      else
      {
        force = 0;
      }
      Tp = rp/rs * rh * force; // compute the required motor pulley torque (Tp) to generate that force
  
      forceOutput(&Motor, force, Tp);
      

      if(reply){
        reply = false;
        
        //Variables the adventurous user could play with. 
        outdata[0] = xh;
        outdata[1] = ball.x;
        
        send_data(device_function, outdata);
      }
    }
  }
}

ISR(TIMER1_OVF_vect){
  TCNT1 = TIMER1_COUNTER;
  timer1_flag = 1;
  // update sensor position
      updateSensorPosition();
}

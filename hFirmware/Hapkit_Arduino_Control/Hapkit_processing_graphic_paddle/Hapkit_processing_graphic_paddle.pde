/*
code by Colin Gallacher and Steven Ding

The following code is subject to the 
 * 
 * GNU General Public License v3.0 
 * GNU GPLv3
 * Permissions of this strong copyleft license are conditioned on making available 
 * complete source code of licensed works and modifications, which include larger 
 * works using a licensed work, under the same license. Copyright and license notices 
 * must be preserved. Contributors provide an express grant of patent rights.


To come: 
Instructions

L


*/


import processing.serial.*;

Board paddle_link;
Device  paddle;
DeviceType degreesOfFreedom;

int baseFrameRate = 40;
byte commType = 0;

byte device_function = 1;
byte[] positions = {1, 1, 0, 0};



float freq = 0;
float amplitude = 0;

int counter=0; 
PShape HapticPaddle, Base, Force, Logo; 
float[] angle ;
float[] torque;

void setup(){
  size(1000 , 1000, P2D);
  noStroke();
  frameRate(baseFrameRate);
  
  paddle_link = new Board(Serial.list()[1], 57600);
  paddle = new Device(degreesOfFreedom.HapticPaddle, device_function, paddle_link);
  
   HapticPaddle = loadShape("hapticPaddle.svg");
   Base = loadShape("hapticPaddleBase.svg"); 
   Force = loadShape("Force.svg");
   Logo = loadShape("StanfordHapkit.svg"); 
   
  
}


void draw(){

  
  if(paddle_link.data_available()){
    
    paddle.receive_data();
    angle = paddle.mechanisms.get_angle();
    torque = paddle.mechanisms.get_torque(); 
    //println(angle[0]);
    println(torque[0]);
    //println(torque[1]); 

    // //angle= sin(counter*3.14159/180.0);
     //angle =0; 

    draw_static(); 
    update_handle(angle[0]*10); 
    update_force(torque[0], angle[0]*10); 
  
  }
  else{
     paddle.set_parameters(device_function, freq, amplitude);
    paddle.send_data();
  }
}

void draw_static()
{
      background(255); 
     shape(Logo,0,0);
     pushMatrix(); 
     translate(width/2, height/2); 
     shape(Base, -Base.width/2, 0); 
     popMatrix();  
}

void update_handle(float angle) {
  // Rotate the shape around the z-axis each time the mouse is pressed
  pushMatrix(); 
  
   translate(width/2, height/2); 
  rotate(angle); 
  translate(-HapticPaddle.width/2, -HapticPaddle.height/2);
    shape(HapticPaddle,0,0);
    //counter++; 
    //translate(0, -HapticPaddle.height/4); 
  //HapticPaddle.translate(width/2*cos(.1), height/2*sin(.1)); 
  popMatrix(); 

}

void update_force(float force, float angle) {
  // Rotate the shape around the z-axis each time the mouse is pressed
  pushMatrix(); 
   translate(width/2, height/2); 
  rotate(angle); 
     
  translate(0, -Force.height/2-HapticPaddle.height/4);
  force = -force; 
  float scaleFactor = 3f; 
  scale(force/scaleFactor);
  translate(0, -Force.height/2*force/(2*scaleFactor)); 
  shape(Force,0,0); 
  popMatrix(); 

}
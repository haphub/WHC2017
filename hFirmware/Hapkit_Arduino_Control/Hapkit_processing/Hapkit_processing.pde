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


/*


import processing.serial.*;

Board paddle_link;
Device  paddle;
DeviceType degreesOfFreedom;

int baseFrameRate = 50;
byte commType = 0;

byte device_function = 3;
byte[] positions = {1, 1, 0, 0};

float[] in_data;

float freq = 0;
float amplitude = 0;


void setup(){
  size(200, 200);
  noStroke();
  frameRate(baseFrameRate);
  
  paddle_link = new Board(Serial.list()[0], 57600);
  paddle = new Device(degreesOfFreedom.HapticPaddle, device_function, paddle_link);
}

void draw(){
  
  if(paddle_link.data_available()){
    
    paddle.receive_data();
    in_data = paddle.mechanisms.get_angle();
    println(in_data.length);
    //println(in_data[1]); 
    
  }
  else{
    
    //device_function = 0;
    paddle.set_parameters(device_function, freq, amplitude);
    paddle.send_data();
    
  }
}
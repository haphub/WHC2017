
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
*/

import processing.serial.*;

Board paddle_link;
Device  paddle;
DeviceType degreesOfFreedom;
int baseFrameRate = 40;
byte device_function = 0;


void setup(){
  size(200, 200);
  noStroke();
  frameRate(baseFrameRate);
  
  paddle_link = new Board(this, Serial.list()[0], 57600);
  paddle = new Device(degreesOfFreedom.HapticPaddle, device_function, paddle_link);
}

long currentTime; 
long oldTime; 

void draw(){
  currentTime = millis(); 
  if((currentTime-oldTime) > 5000){
   device_function = (byte)(device_function+1);  
   oldTime = currentTime; 
   
   if(device_function > 3) device_function = (byte) 0; 
   
  }
  
    paddle.set_parameters(device_function, 0.0f, 0.0f);
    paddle.send_data();
  
}
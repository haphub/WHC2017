
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
int baseFrameRate = 50;
byte commType = 0;
byte device_function = 2;
byte[] positions = {1, 1, 0, 0};
float[] in_data;
float freq = 0;
float amplitude = 0;
void setup() {
  size(200, 200);
  noStroke();
  frameRate(baseFrameRate);

  String serial_port = "";
  for (int i = 0; i < Serial.list().length; i++)
  {
    print(Serial.list()[i]);
    //if (Serial.list()[i].substring("VID:PID403:6001"
  }
  paddle_link = new Board(Serial.list()[1], 57600);
  paddle = new Device(degreesOfFreedom.HapticPaddle, device_function, paddle_link);
}

long currentTime; 
long oldTime; 

void draw() {
  currentTime = millis(); 
  if ((currentTime-oldTime) > 5000) {
    device_function = (byte)(device_function+1);  
    oldTime = currentTime; 

    if (device_function > 3) device_function = (byte) 0;
  }


  if (paddle_link.data_available()) {

    paddle.receive_data();
    in_data = paddle.mechanisms.get_angle();
    //println(in_data.length);
    print(in_data[0]);
    print(",");
    in_data = paddle.mechanisms.get_torque();
    println(in_data[0]);
    

  } else {

    //device_function = 0;
    paddle.set_parameters(device_function, freq, amplitude);
    paddle.send_data();
  }
}
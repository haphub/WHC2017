
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
int baseFrameRate = 50;
byte commType = 0;
byte device_function = 2;
float[] in_data;

//graphical parametres
int graphics_y = 100;
int paddle_width = 20;
int paddle_height = 50;
int ball_radius = 20;
int paddle_position = 100;
int ball_position = 200;

void setup() {
  size(900,500);
  noStroke();
  frameRate(baseFrameRate);

  String serial_port = "";
  for (int i = 0; i < Serial.list().length; i++)
  {
    print(Serial.list()[i]);
    //if (Serial.list()[i].substring("VID:PID403:6001"
  }
  paddle_link = new Board(Serial.list()[1], 57600);
  paddle = new Device(DeviceType.HapticPaddle, device_function, paddle_link);
}

void draw() {

  if (paddle_link.data_available()) {
    
    paddle.receive_data();
    in_data = paddle.mechanisms.get_angle();
    //print(in_data[0]);
    //print(",");
    //in_data = paddle.mechanisms.get_torque();
    //println(in_data[0]);
    paddle_position = (int)(100 * paddle.mechanisms.get_angle()[0]); //MELISA UPDATE HERE
    ball_position = (int)(100 * paddle.mechanisms.get_torque()[0]); //MELISA UPDATE HERE
  
  } 
  
  background(255);
  fill(255,0,0);
  rect(paddle_position-paddle_width/2, graphics_y-paddle_height/2, paddle_width, paddle_height);
  fill(0,0,255);
  ellipse(ball_position, graphics_y, ball_radius*2, ball_radius*2);
  
    
}
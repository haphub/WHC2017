import processing.serial.*;

// communication vairable declarations 
byte device_function = 0;
float[] in_data = new float[2];
float[] out_data = new float[2];

// hapkit status variables
float freq = 120;
float amplitude = 0.4;

// simulation environment variables
float force;
float torque;

// system initialization variables
int baseFrameRate = 100;
int function = 0;
Board_Simple paddle_link;


void setup(){
  size(200, 200);
  noStroke();
  frameRate(baseFrameRate);
  
  paddle_link = new Board_Simple(Serial.list()[0], 57600);
}

void draw(){
  
  if(paddle_link.data_available()){
    
   
    in_data = paddle_link.receive_data(device_function);
     
    // update simulation environment variables only if communication is expected  
    if(function == 1){
      force = in_data[0];
      torque = in_data[1];
    }
      
    if(paddle_link.get_functionMatch()){
      function = 1;
    }
    else{
      function = 0;
    }
    
    //println(in_data[0] + ", " + in_data[1]);
  }
  
  else{
    switch(function){
      case 0: // send hapkit initialization and parameters
        out_data[0] = freq;
        out_data[1] = amplitude;
        paddle_link.send_data(device_function, out_data);
        break;
      case 1: // request state data from hapkit
        paddle_link.send_data_request();
        break;
      default:
        break;
    }
  
  }
}
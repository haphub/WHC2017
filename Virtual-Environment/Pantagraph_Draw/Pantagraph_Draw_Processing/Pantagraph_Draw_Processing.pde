/**
 *******************************************************************************
 * @file       Pantagraph_Draw_Processing
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


/* library imports *************************************************************/
import processing.serial.*;

/* Serial definition ***********************************************************/
Serial port;

/* global dimensions definitions ***********************************************/
int l = 100;
int L = 150;
int d = 40;

/* Encoder parameter definitions **********************************************/
int Enc1_offset = 180;
int Enc1_positions = 13856;
int Enc1_resolution = 13824;

int Enc2_offset = 0;
int Enc2_positions = 13856;
int Enc2_resolution = 13824;

/* Serial communication block definitions ************************************/
byte[] init_packet = new byte[25];
byte[] inBuffer = new byte[8];
byte[] segments = new byte[4];

/* Graphics object primitives definitions ************************************/
PShape kite, circle1, circle2;


/**
 * @brief    Main setup function, defines parameters and hardware setup
 * @note     None
 * @param    None
 * @return   None 
 */
void setup(){
  size(600, 400, P2D);
  background(255);
  frameRate(100);
  createKite();
  
  port = new Serial(this, Serial.list()[0], 0);
  port.buffer(8); 
  
  initializePacket();
  port.write(init_packet);  
}

/**
 * @brief    Main draw function, updates frame at perscribed frame rate
 * @note     None
 * @param    None
 * @return   None 
 */
void draw(){
  
  if(inBuffer != null){
      forwardKinematics();
      background(255); // To clean up the left-overs of drawings from the previous loop!

      shape(kite); // Display the kite
      shape(circle1);
      shape(circle2);  
  }

}

/**
 * @brief    Serial event handler, reads the number of incoming bytes dictated by 
 *           buffer length
 * @note     None
 * @param    port: the serial port where an event is being observed
 * @return   None 
 */
void serialEvent(Serial port){
  port.readBytes(inBuffer);
}


/* Initialization functions -----------------------------------------------------*/

/**
 * @brief    Format the device initialization packet
 * @note     Currently under prototype
 * @param    None
 * @return   None 
 */
void initializePacket(){
  init_packet[0] =  0;
  
  for(int i = 0; i < 6; i++){
    
    switch(i){
      case 0:
        segments = IntegerToBytes(Enc1_offset);
        break;
      case 1:
        segments = IntegerToBytes(Enc1_positions);
        break;
      case 2:
        segments = IntegerToBytes(Enc1_resolution);
        break;
      case 3:
        segments = IntegerToBytes(Enc2_offset);
        break;
      case 4:
        segments = IntegerToBytes(Enc2_positions);
        break;
      case 5:
        segments = IntegerToBytes(Enc2_resolution);
        break;

    }
    
    System.arraycopy(segments, 0, init_packet, i*4+1, 4);
  }
}


/**
 * @brief    Specifies the parameters for a pentagraph kite object
 * @note     Currently under prototype
 * @param    None
 * @return   None 
 */
void createKite(){
  kite = createShape();
  kite.beginShape();
  kite.fill(255);
  kite.stroke(0);
  kite.strokeWeight(2);
  
  kite.vertex(width/2, 2*height/3);
  kite.vertex(width/2, 2*height/3);
  kite.vertex(width/2, 2*height/3);
  kite.vertex(width/2+d, 2*height/3);
  kite.vertex(width/2+d, 2*height/3);
  kite.endShape(CLOSE);
  
  circle1 = createShape(ELLIPSE, width/2, 2*height/3, d/5, d/5);
  circle1.setStroke(color(0));
  
  circle2 = createShape(ELLIPSE, width/2+d, 2*height/3, d/5, d/5);
  circle2.setStroke(color(0));
}


/* Physics/dynamics functions ---------------------------------------------------*/

/**
 * @brief    Determine the forward kinematic physics for pentagraph system 
 * @note     Currently under prototype
 * @param    None
 * @return   None 
 */
void forwardKinematics(){
  float th1, th2;

  float inByte1 = 0.0; 
  float inByte2 = 0.0;
  
  System.arraycopy(inBuffer, 0, segments, 0, 4);
  inByte1 = float(BytesToInteger(segments));
  
  System.arraycopy(inBuffer, 4, segments, 0, 4);
  inByte2 = float(BytesToInteger(segments));
  
  th1=PI/180.0*inByte1;      //convert to radians
  th2=PI/180.0*inByte2; //convert to radians (after subtracting 80 degrees that we had added on the ".ino" side.) 
      
  // Forward Kinematics
  float c1=cos(th1);
  float c2=cos(th2);
  float s1=sin(th1);
  float s2=sin(th2);
    
  float xA=l*c1;
  float yA=l*s1;
  float xB=d+l*c2;
  float yB=l*s2;
  float R=pow(xA,2) +pow(yA,2);
  float S=pow(xB,2)+pow(yB,2);
  float M=(yA-yB)/(xB-xA);
  float N=0.5*(S-R)/(xB-xA);
  float a=pow(M,2)+1;
  float b=2*(M*N-M*xA-yA);
  float c=pow(N,2)-2*N*xA+R-pow(L,2);
  float Delta=pow(b,2)-4*a*c;
  float y_E=(-b+sqrt(Delta))/(2*a);
  float x_E=M*y_E+N;
  
  kite.setVertex(1,width/2+l*cos(th1), 2*height/3-l*sin(th1)); // Vertex A with th1 from encoder reading
  kite.setVertex(3,width/2+d+l*cos(th2), 2*height/3-l*sin(th2)); // Vertex B with th2 from encoder reading
  kite.setVertex(2,width/2+x_E, 2*height/3-y_E); // Vertex E from Fwd Kin calculations
  
}



/* Helper functions -------------------------------------------------------------*/

/**
 * @brief    Translates a 32-bit integer into an array of four bytes
 * @note     None
 * @param    val: 32-bit signed integer value
 * @return   four byte representation of the 32-bit number 
 */
byte[] IntegerToBytes(int val){
  
  byte[] segments = new byte[4];
  
  segments[3] = (byte)((val >> 24) & 0xff);
  segments[2] = (byte)((val >> 16) & 0xff);
  segments[1] = (byte)((val >> 8) & 0xff);
  segments[0] = (byte)((val) & 0xff);

  return segments;
}


/**
 * @brief    Translates an array of four bytes into a signed integer format
 * @note     None
 * @param    segment: the input array of four bytes
 * @return   Translated 32-bit signed integer value 
 */
int BytesToInteger(byte[] segment){
  
  int val = 0;
  
  val = (val | (segment[3] & 0xff)) << 8;
  val = (val | (segment[2] & 0xff)) << 8;
  val = (val | (segment[1] & 0xff)) << 8;
  val = (val | (segment[0] & 0xff)); 
  
  return val;
}
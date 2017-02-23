
/**
 *******************************************************************************
 * @file       Pantagraph_Draw_Processing.pde
 * @author     
 * @version    V0.0.2
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


/* global dimensions definitions ***********************************************/
int l = 100;
int L = 150;
int d = 40;
float th1, th2, x_E, y_E;


/* Serial communication block definitions ************************************/
communication haply_link;
device haply;
DeviceType pentagraph;

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
  frameRate(1000);
  createKite();
  
  haply_link = new communication(Serial.list()[0]);
  haply = new device(pentagraph.TwoDOF, haply_link);
  
  haply.set_actuator_parameters(1, 180, 13824, 1);
  haply.set_actuator_parameters(2, 0, 13824, 2);
  
  haply.device_set_parameters();
  haply.device_read_request();
}



/**
 * @brief    Main draw function, updates frame at perscribed frame rate
 * @note     None
 * @param    None
 * @return   None 
 */
void draw(){
  
  if(haply_link.data_available()){
    haply.device_read_angles();
    
    forwardKinematics(haply.motor1.angle, haply.motor2.angle);
    update_animation(th1, th2, x_E, y_E);
    //println(haply.motor1.angle);
    //println(haply.motor2.angle);
  }
  else{
    haply.device_read_request();
  }
  

}


/* Initialization functions -----------------------------------------------------*/

/**
 * @brief    Specifies the parameters for a pentagraph kite animation
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

void forwardKinematics(float angle1, float angle2){
  th1 = PI/180*angle1;
  th2 = PI/180*angle2;
  
  // Forward Kinematics
  float c1 = (float)cos(this.th1);
  float c2 = (float)cos(this.th2);
  float s1 = (float)sin(this.th1);
  float s2 = (float)sin(this.th2);
  
  float xA = this.l*c1;
  float yA = this.l*s1;
  float xB = this.d+l*c2;
  float yB = this.l*s2;
  float R = (float)pow(xA,2) + (float)pow(yA,2);
  float S = (float)pow(xB,2) + (float)pow(yB,2);
  float M = (yA-yB)/(xB-xA);
  float N = (float)0.5*(S-R)/(xB-xA);
  float a = (float)pow(M,2)+1;
  float b = 2*(M*N-M*xA-yA);
  float c = (float)pow(N,2)-2*N*xA+R-(float)pow(L,2);
  float Delta = (float)pow(b,2)-4*a*c;
    
  y_E = (-b+(float)sqrt(Delta))/(2*a);
  x_E = M*y_E+N;    
}

void update_animation(float th1, float th2, float x_E, float y_E){
  kite.setVertex(1,width/2+l*cos(th1), 2*height/3-l*sin(th1)); // Vertex A with th1 from encoder reading
  kite.setVertex(3,width/2+d+l*cos(th2), 2*height/3-l*sin(th2)); // Vertex B with th2 from encoder reading
  kite.setVertex(2,width/2+x_E, 2*height/3-y_E); // Vertex E from Fwd Kin calculations
  
  background(255); // To clean up the left-overs of drawings from the previous loop!
  shape(kite); // Display the kite
  shape(circle1);
  shape(circle2); 
}
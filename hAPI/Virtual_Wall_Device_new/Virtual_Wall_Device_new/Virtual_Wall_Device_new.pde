
/**
 *******************************************************************************
 * @file       Pantagraph_Processing.pde
 * @author    
 * @version    V0.1.0
 * @date       27-February-2017
 * @brief      Prototype tests for encoder input for Pantagraph application
 *******************************************************************************
 * @attention
 *
 *
 *******************************************************************************
 */

/* library imports *************************************************************/
import processing.serial.*;
  
/* Device block definitions ************************************/
Device          pentagraph;
byte            deviceID = 5;
Board           pentagraph_link;
DeviceType      degreesOfFreedom;
Mechanisms      NewMech = new NewExampleMech();

/* Graphics object primitives definitions ************************************/
int             l                = 2*50;
int             L                = 2*70;
int             d                = 2*20;

int             y_wall           = 150;
int             k_wall           = 80;

float           f_x              = 0;
float           f_y              = 0;

float           p_Wall;

float[]         params           = new float[2];
float[]         coordinates      = new float[2];

PShape          kite, circle1, circle2;

int             baseFrameRate    = 1000;
int             animation_count  = baseFrameRate/60;
int             haptics_count    = baseFrameRate/1000;

/**
 * @brief    Main setup function, defines parameters and hardware setup
 */
void setup(){
  size(600, 400, P2D);
  background(255);
  frameRate(baseFrameRate);
  createKite();
  
  pentagraph_link = new Board(Serial.list()[0], 0);
  pentagraph = new Device(degreesOfFreedom.TwoDOF, deviceID, pentagraph_link);
  
  pentagraph.set_actuator_parameters(1, 180, 13824, 2);
  pentagraph.set_actuator_parameters(2, 0, 13824, 1);
  
  pentagraph.set_new_mechanism(NewMech);
  float[] parameters = {(float)l, (float)L, (float)d, 2*1000};
  pentagraph.mechanisms.set_mechanism_parameters(parameters);
	
  pentagraph.device_set_parameters();
}
//

/**
 * @brief    Main draw function, updates frame at perscribed frame rate
 */
void draw(){
  
  if(pentagraph_link.data_available()){

    pentagraph.device_read_angles();
    
    /* forward kinematics calculation */
    params[0] = pentagraph.motors[0].get_angle(); 
    params[1] = pentagraph.motors[1].get_angle();
    pentagraph.mechanisms.forwardKinematics(params);
    
    coordinates = pentagraph.mechanisms.get_coordinate();
        
    p_Wall = coordinates[1] - y_wall;
  
    	/* */
    if(p_Wall > 0){
      f_y = -k_wall*p_Wall;
      params[0] = f_x; 
      params[1] = f_y;
      pentagraph.mechanisms.torqueCalculation(params);
      
      params = pentagraph.mechanisms.get_torque();
      pentagraph.motors[0].set_torque(params[0]);
      pentagraph.motors[1].set_torque(params[1]);
	  } 
	}

  if(frameCount % animation_count == 0){
    params = pentagraph.mechanisms.get_angle();
    coordinates = pentagraph.mechanisms.get_coordinate();
    update_animation(params[0], params[1], coordinates[0], coordinates[1]);
  }
	
  
  if(frameCount % haptics_count == 0){
    pentagraph.device_write_torques();
  }
  
  //if(frameCount % baseFrameRate == 0){
  //  	println(frameCount / (millis()/1000f));
  //}
}


/* Graphical and physics functions -----------------------------------------------------*/

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

void update_animation(float th1, float th2, float x_E, float y_E){
	kite.setVertex(1,width/2+l*cos(th1), 2*height/3-l*sin(th1)); // Vertex A with th1 from encoder reading
	kite.setVertex(3,width/2+d+l*cos(th2), 2*height/3-l*sin(th2)); // Vertex B with th2 from encoder reading
	kite.setVertex(2,width/2+x_E, 2*height/3-y_E); // Vertex E from Fwd Kin calculations
  
	background(255); // To clean up the left-overs of drawings from the previous loop!
  
	shape(kite); // Display the kite
	shape(circle1);
	shape(circle2); 
	stroke(0);
  
	line(0, 2*height/3-y_wall, width, 2*height/3-y_wall);
}
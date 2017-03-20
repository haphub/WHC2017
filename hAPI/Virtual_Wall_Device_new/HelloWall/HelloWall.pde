
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
Device          haply_2DoF;
byte            deviceID = 5;
Board           haply_board;
DeviceType      degreesOfFreedom;
//Mechanisms      NewMech = new NewExampleMech();

/* Graphics Parameters ************************************/
int             baseFrameRate     = 1000;
int             animation_count   = baseFrameRate/501;
int             haptics_count     = baseFrameRate/1000;


PShape          pantograph, joint1, joint2, handle;

int             l                 = 2*50;
int             L                 = 2*70;
int             d                 = 2*20;

/*Physics parameters*/

int             y_wall            = 150;
int             k_wall            = 80;

float           f_x               = 0;
float           f_y               = 0;

float           p_Wall;


// generic data for a 2DOF device
float[]         angles            = new float[2];
float[]         coordinates       = new float[2];

float[]         forces           = new float[2]; 
float[]         torques           = new float[2]; 


/**
 * @brief    Main setup function, defines parameters and hardware setup
 */
void setup(){
  
  /*Setup for the graphic display window and drawing objects*/
  
  size(600, 400, P2D);
  background(255);
  frameRate(baseFrameRate);
  createpantograph();
  
  /* Initialization of the Board, Device, and Device Components*/ 
  
  //BOARD
  haply_board = new Board(Serial.list()[0], 0);
  
  //DEVICE
  haply_2DoF = new Device(degreesOfFreedom.TwoDOF, deviceID, haply_board);
  
  haply_2DoF.device_set_parameters();
 
}

      
/**
 * @brief    Main draw function, updates frame at perscribed frame rate
 */
void draw(){
  
 
  if(haply_board.data_available()){

  /*** GET END-EFFECTOR POSITION (TASK SPACE)****/ 
    haply_2DoF.device_read_angles();
    /* forward kinematics calculation */
    angles[0] = haply_2DoF.encoders[0].get_angle(); 
    angles[1] = haply_2DoF.encoders[1].get_angle();
    haply_2DoF.mechanisms.forwardKinematics(angles);
    coordinates = haply_2DoF.mechanisms.get_coordinate();
    
		
/*** PHYSICS OF THE SIMULATION ****/ 

		p_Wall = coordinates[1] - y_wall;
  
    if(p_Wall > 0){
      f_y = -k_wall*p_Wall;
      forces[0] = f_x; 
      forces[1] = f_y;
      haply_2DoF.mechanisms.torqueCalculation(forces);
      
      torques = haply_2DoF.mechanisms.get_torque();
      haply_2DoF.motors[0].set_torque(torques[0]);
      haply_2DoF.motors[1].set_torque(torques[1]);
		} 
    else{
      haply_2DoF.motors[0].set_torque(0);
      haply_2DoF.motors[1].set_torque(0);
    }
  }

	/******* ANIMATION TIMER ********/ 
  if(frameCount % animation_count == 0){
    angles = haply_2DoF.mechanisms.get_angle();
    coordinates = haply_2DoF.mechanisms.get_coordinate();
    update_animation(angles[0], angles[1], coordinates[0], coordinates[1]);
  }
	
  /********** HAPTICS TIMER *************/ 
	
  if(frameCount % haptics_count == 0){
    haply_2DoF.device_write_torques();
  }
  
  
}


/* Graphical and physics functions -----------------------------------------------------*/

/**
 * @brief    Specifies the parameters for a haply_2DoF pantograph animation
 * @note     Currently under prototype
 * @param    None
 * @return   None 
 */
 
void createpantograph(){
	pantograph = createShape();
	pantograph.beginShape();
	pantograph.fill(255);
	pantograph.stroke(0);
	pantograph.strokeWeight(2);
  
	pantograph.vertex(width/2, 2*height/3);
	pantograph.vertex(width/2, 2*height/3);
	pantograph.vertex(width/2, 2*height/3);
	pantograph.vertex(width/2+d, 2*height/3);
	pantograph.vertex(width/2+d, 2*height/3);
	pantograph.endShape(CLOSE);
  
	joint1 = createShape(ELLIPSE, width/2, 2*height/3, d/5, d/5);
	joint1.setStroke(color(0));
  
	joint2 = createShape(ELLIPSE, width/2+d, 2*height/3, d/5, d/5);
	joint2.setStroke(color(0));

  line(0, 2*height/3-y_wall, width, 2*height/3-y_wall);
  


}

void update_animation(float th1, float th2, float x_E, float y_E){
	pantograph.setVertex(1,width/2+l*cos(th1), 2*height/3-l*sin(th1)); // Vertex A with th1 from encoder reading
	pantograph.setVertex(3,width/2+d+l*cos(th2), 2*height/3-l*sin(th2)); // Vertex B with th2 from encoder reading
	pantograph.setVertex(2,width/2+x_E, 2*height/3-y_E); // Vertex E from Fwd Kin calculations  
	background(255); // To clean up the left-overs of drawings from the previous loop!
  
	shape(pantograph); // Display the pantograph
	shape(joint1);
	shape(joint2); 
  
  line(0, 2*height/3-y_wall, width, 2*height/3-y_wall);
  
  handle = createShape(ELLIPSE, width/2+x_E, 2*height/3-y_E, d, d);
  handle.setStroke(color(0));
  shape(handle); 
  
	stroke(0);
  

}



  ////ENCODERS
  //haply_2DoF.set_encoder_parameters(1, 180, 13824, 1);
  //haply_2DoF.set_encoder_parameters(2, 0, 13824, 2);
  
  ////MOTORS
  //haply_2DoF.set_actuator_parameters(1, 1);
  //haply_2DoF.set_actuator_parameters(2, 2);
  
  //MECHANISM
  //haply_2DoF.set_new_mechanism(NewMech);
  //float[] parameters = {(float)l, (float)L, (float)d, 2000 }; //device link parameters
  //haply_2DoF.mechanisms.set_mechanism_parameters(parameters);
  
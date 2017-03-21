
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
long             baseFrameRate     = 1000;
long             animation_count   = 0;
long             haptics_count     = 0;


PShape          pantograph, joint1, joint2, handle, line1;

int             l                 = 2*50;
int             L                 = 2*70;
int             d                 = 2*20;
int             r_ee              = d/3; 

PVector          device_origin    = new PVector (0, 0) ; 

/*Physics Simulation parameters*/

PVector        f_wall             = new PVector(0,0); 
float          k_wall             = 80.0; 
PVector        pen_wall           = new PVector(0,0); 
PVector        pos_wall           = new PVector(0, 150); 


// generic data for a 2DOF device
// joint space
PVector        angles            = new PVector(0,0);
PVector        torques           = new PVector(0,0);

//task space
PVector         pos_ee           = new PVector(0,0);
PVector         f_ee             = new PVector(0,0); 


/**
 * @brief    Main setup function, defines parameters and hardware setup
 */
void setup(){
  
  /*Setup for the graphic display window and drawing objects*/
  
  size(600, 400, P2D);
  background(255);
  frameRate(baseFrameRate);
   /* Initialization of the Board, Device, and Device Components*/ 
  
  //BOARD
  haply_board = new Board(Serial.list()[0], 0);
  
  //DEVICE
  haply_2DoF = new Device(degreesOfFreedom.TwoDOF, deviceID, haply_board);
  
  haply_2DoF.device_set_parameters();
  
  //graphics
  device_origin.add(width/2, height/4 );
  createpantograph();
}


      
/**
 * @brief    Main draw function, updates frame at perscribed frame rate
 */
void draw(){
  scale(1,-1);
  translate(0,-height); 
  
 
  if(haply_board.data_available()){

  /*** GET END-EFFECTOR POSITION (TASK SPACE)****/ 
    haply_2DoF.device_read_angles();
   
  /* forward kinematics calculation */
    angles.x = haply_2DoF.encoders[0].get_angle(); 
    angles.y = haply_2DoF.encoders[1].get_angle();
    haply_2DoF.mechanisms.forwardKinematics(angles.array());
    pos_ee.set( haply_2DoF.mechanisms.get_coordinate());
		
/*** PHYSICS OF THE SIMULATION ****/ 

   f_wall.set(0,0); 
   
   pen_wall.set(0, (pos_wall.y - (pos_ee.y+r_ee))); 
   
   if(pen_wall.y < 0){
     f_wall = f_wall.add((pen_wall.mult(-k_wall)));
   }
   
   f_ee = (f_wall.copy()).mult(-1); 
  }
  
    haply_2DoF.mechanisms.torqueCalculation(f_ee.array());
    torques.set(haply_2DoF.mechanisms.get_torque());
    haply_2DoF.motors[0].set_torque(torques.x);
    haply_2DoF.motors[1].set_torque(torques.y);

  /******* ANIMATION TIMER ********/ 
  if((frameCount - animation_count) > 8){
    angles.set(haply_2DoF.mechanisms.get_angle());
    pos_ee.set(haply_2DoF.mechanisms.get_coordinate());
    update_animation(angles.x, angles.y, pos_ee.x, pos_ee.y);
    animation_count = frameCount; 
  }
  
  /********** HAPTICS TIMER *************/ 
  
  if((frameCount - haptics_count) > 1){
    haply_2DoF.device_write_torques();
    haptics_count=frameCount; 
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
  pantograph.fill(0);
  pantograph.stroke(255);
  pantograph.strokeWeight(2);
  
  
  pantograph.vertex(device_origin.x, device_origin.y);
  pantograph.vertex(device_origin.x, device_origin.y);
  pantograph.vertex(device_origin.x, device_origin.y);
  pantograph.vertex(device_origin.x+d, device_origin.y);
  pantograph.vertex(device_origin.x+d, device_origin.y);
  pantograph.endShape(CLOSE);
  
  joint1 = createShape(ELLIPSE, device_origin.x, device_origin.y, d/5, d/5);
  joint1.setStroke(color(255));
  
  joint2 = createShape(ELLIPSE, device_origin.x+d, device_origin.y, d/5, d/5);
  joint2.setStroke(color(255));

  handle = createShape(ELLIPSE, device_origin.x, device_origin.y, 2*r_ee, 2*r_ee);
  handle.setStroke(color(255));
  
  
  line1 = createShape(LINE, device_origin.x-200, device_origin.y+pos_wall.y, device_origin.x+200,device_origin.y+pos_wall.y);
  line1.setStroke(color(255));

}

void update_animation(float th1, float th2, float x_E, float y_E){
      background(0); // To clean up the left-overs of drawings from the previous loop!

  pantograph.setVertex(1,device_origin.x+l*cos(th1), device_origin.y+l*sin(th1)); // Vertex A with th1 from encoder reading
  pantograph.setVertex(3,device_origin.x+d+l*cos(th2), device_origin.y+l*sin(th2)); // Vertex B with th2 from encoder reading
  pantograph.setVertex(2,device_origin.x+x_E, device_origin.y+y_E); // Vertex E from Fwd Kin calculations  
  
  shape(pantograph); // Display the pantograph
  shape(joint1);
  shape(joint2); 
  shape(line1); 
  shape(handle,x_E, y_E); 
  stroke(255); 
  
  
  


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
  
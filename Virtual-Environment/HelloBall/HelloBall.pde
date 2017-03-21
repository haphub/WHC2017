
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
int             baseFrameRate     = 500;
int             animation_count   = baseFrameRate/50;
int             haptics_count     = baseFrameRate/500;


PShape          pantograph, joint1, joint2, handle;
PShape          ball; 


int             l                 = 2*50;
int             L                 = 2*70;
int             d                 = 2*20;
int             r_ee              = d/3; 

int             r_ball            = d/2; 

PVector          device_origin    = new PVector (0, 0) ; 

/*Physics Simulation parameters*/

// dynamics of ball
long            oldTimer          = 0; 
int             m_ball            = 5; //grams
int             k_ball            = 2000; //grams/s^2
float           pen_ball          = 0.0; // mm
float           b_air             = .2; // grams/s
PVector         f_gravity         = new PVector(0, -8000); // mm/s^2


// Initial Conditions
PVector         pos_ball          = new PVector(0,100);  // mm
PVector         vel_ball          = new PVector(0,0); // mm/s

PVector         f_ball            = new PVector(0,0); // uN
PVector         f_contact         = new PVector(0,0);
PVector         f_damping         = new PVector(0,0); 

// wall static-contacts

PVector        f_wall             = new PVector(0,0); 
float          k_wall             = 10000.0; 
float          b_wall             = 10; 
PVector        pen_wall           = new PVector(0,0); 
PVector        pos_wall_left      = new PVector(-150, 0); 
PVector        pos_wall_bottom    = new PVector(d/2, 75); 
PVector        pos_wall_right     = new PVector(150+d, 0); 

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


//Contact Forces


		PVector vec_ee2ball = (pos_ball.copy()).sub(pos_ee);
    float vec_ee2Ball_magnitude = vec_ee2ball.mag(); 
    pen_ball = vec_ee2Ball_magnitude - (r_ball+r_ee);
    
    //println(pen_ball); 
  
  // ball forces
    if(pen_ball<0){
       f_contact= vec_ee2ball.normalize();
       f_contact= f_contact.mult(-k_ball*pen_ball);  // since pen_ball is negative k_ball must be negative to ensure the force acts along the end-effector to the ball
      
		}
    else{
      f_contact.set(0,0); 
    }
  
// forces due to damping

    f_damping = (vel_ball.copy()).mult(-b_air); 
    
// forces due to wall

  f_wall.set(0,0); 
  
  pen_wall.set((pos_ball.x-r_ball) - pos_wall_left.x, 0); 

  if(pen_wall.x < 0 ){
    f_wall = f_wall.add((pen_wall.mult(-k_wall))).add((vel_ball.copy()).mult(-b_wall));
    }
    
    pen_wall.set(0, (pos_ball.y-r_ball) - pos_wall_bottom.y); 
    
  if(pen_wall.y < 0){
    f_wall = f_wall.add((pen_wall.mult(-k_wall))).add((vel_ball.copy()).mult(-b_wall));
  }
  
  pen_wall.set((pos_ball.x+r_ball) - pos_wall_right.x, 0); 

  if(pen_wall.x > 0 ){
    f_wall = f_wall.add((pen_wall.mult(-k_wall))).add((vel_ball.copy()).mult(-b_wall));
    }
    
    
    //println(f_wall); 
// sum of forces 

    f_ball = (f_contact.copy()).add(f_gravity).add(f_damping).add(f_wall); 
    f_ee = (f_contact.copy()).mult(-1); 
    
    haply_2DoF.mechanisms.torqueCalculation(f_ee.array());
    torques.set(haply_2DoF.mechanisms.get_torque());
    haply_2DoF.motors[0].set_torque(torques.x);
    haply_2DoF.motors[1].set_torque(torques.y);
    

// INTEGRATE THE ACCELERATION TO GET THE STATES OF THE BALL
long currentTimer = millis(); 
float dt = (float)(currentTimer - oldTimer); 
//println(dt); 
//dt = dt/1000; 
//dt = (dt < 0.001 )? 0.002 : dt; 
//println(dt);
dt=.002; 
//println(dt);
pos_ball = (((f_ball.copy()).div(2*m_ball)).mult(dt*dt)).add(((vel_ball.copy()).mult(dt))).add(pos_ball);
vel_ball = (((f_ball.copy()).div(m_ball)).mult(dt)).add(vel_ball); 

oldTimer = currentTimer; 

}

	/******* ANIMATION TIMER ********/ 
  if(frameCount % animation_count == 0){
    angles.set(haply_2DoF.mechanisms.get_angle());
    pos_ee.set(haply_2DoF.mechanisms.get_coordinate());
    update_animation(angles.x, angles.y, pos_ee.x, pos_ee.y);
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
  
  
	pantograph.vertex(device_origin.x, device_origin.y);
	pantograph.vertex(device_origin.x, device_origin.y);
	pantograph.vertex(device_origin.x, device_origin.y);
	pantograph.vertex(device_origin.x+d, device_origin.y);
	pantograph.vertex(device_origin.x+d, device_origin.y);
	pantograph.endShape(CLOSE);
  
	joint1 = createShape(ELLIPSE, device_origin.x, device_origin.y, d/5, d/5);
	joint1.setStroke(color(0));
  
	joint2 = createShape(ELLIPSE, device_origin.x+d, device_origin.y, d/5, d/5);
	joint2.setStroke(color(0));


}

void update_animation(float th1, float th2, float x_E, float y_E){
  
	pantograph.setVertex(1,device_origin.x+l*cos(th1), device_origin.y+l*sin(th1)); // Vertex A with th1 from encoder reading
	pantograph.setVertex(3,device_origin.x+d+l*cos(th2), device_origin.y+l*sin(th2)); // Vertex B with th2 from encoder reading
	pantograph.setVertex(2,device_origin.x+x_E, device_origin.y+y_E); // Vertex E from Fwd Kin calculations  
	background(255); // To clean up the left-overs of drawings from the previous loop!
  
	shape(pantograph); // Display the pantograph
	shape(joint1);
	shape(joint2); 
  
  line(device_origin.x+pos_wall_left.x, device_origin.y+pos_wall_bottom.y, device_origin.x+pos_wall_left.x, device_origin.y+300);
  line(device_origin.x+pos_wall_left.x, device_origin.y+pos_wall_bottom.y, device_origin.x+pos_wall_right.x,device_origin.y+pos_wall_bottom.y);
  line(device_origin.x+pos_wall_right.x, device_origin.y+pos_wall_bottom.y, device_origin.x+pos_wall_right.x, device_origin.y+300);

  
  handle = createShape(ELLIPSE, device_origin.x+x_E, device_origin.y+y_E, 2*r_ee, 2*r_ee);
  handle.setStroke(color(0));
  shape(handle); 
  
  ball = createShape(ELLIPSE, device_origin.x+pos_ball.x, device_origin.y+pos_ball.y, 2*r_ball, 2*r_ball);
  ball.setStroke(color(0));
  shape(ball); 
  
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
  
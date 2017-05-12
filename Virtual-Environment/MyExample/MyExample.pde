import processing.serial.*;


// Reference from HelloBall (may need to copy other files from HelloBall as well
Device          haply_2DoF;
byte            deviceID = 5;
Board           haply_board;
DeviceType      degreesOfFreedom;

/* Graphics Parameters ************************************/
int             baseFrameRate     = 500;
long             animation_count   = 0; 
long             haptics_count     = 0; 
long             count             = 0; 

PShape          pantograph, joint1, joint2, handle, Earth, Mars, Mercury;  

int             l                 = 2*50;        
int             L                 = 2*70;
int             d                 = 2*20;
int             r_ee              = d/3;     //radius of handle

PVector          device_origin    = new PVector (0, 0) ; 

//dynamics of planets
long            oldTimer          = 0; 

// generic data for a 2DOF device
// joint space
PVector        angles            = new PVector(0,0);
PVector        torques           = new PVector(0,0);

//task space
PVector         pos_ee           = new PVector(0,0);

int planetno = 3;
Planet[] planets = new Planet[planetno];
 PShape[] planetshape = new PShape[planetno];

//==================ClassPlanet================================
class Planet{
  float radius;              //radius
  float mass;                //mass
  int k = 5000;                //some constant
  float pen = 0.0;            //distance between planet and handle
  float accel = 0;            //acceleration
  PVector position;          //position
  PVector temposition;        //temperary position
  PVector restposition;       //rest position
  PVector velocity = new PVector(0,0);          //velocity
  PVector f = new PVector(0,0);            //force acting on planet
  float damping = 0.98;      //damping force
  PVector contact = new PVector(0,0);      //contact force
  PVector f_ee = new PVector(0,0);         //
  boolean hit = false;             //hit by handle
  PVector vec_ee2Planet = new PVector(0,0);
  color colorOfPlanet;
  
  //Constructor
  Planet(float r, float m, PVector pos, color c){
    radius = r;
    mass = m;
    position = pos;
    temposition = pos;
    restposition = pos;
    colorOfPlanet = c;
  }
  
  void update(){
    //contact force
    hit = isHit(position);
    if(hit){
      contact = vec_ee2Planet.normalize();
      contact = contact.mult(-k * pen);
      
    }else{
      contact.set(0,0);
    }
      
    //sum of forces
    //f = (contact.copy()).add(damping);  //edit damping
    f = contact;
    //f = (temposition.sub(restposition)).mult(-k);
    f_ee = (contact.copy()).mult(-1);
    
    
    haply_2DoF.mechanisms.torqueCalculation(f_ee.array());
    torques.set(haply_2DoF.mechanisms.get_torque());
    haply_2DoF.motors[0].set_torque(torques.x);
    haply_2DoF.motors[1].set_torque(torques.y);
    
    // INTEGRATE THE ACCELERATION TO GET THE STATES OF THE BALL
    long currentTimer = count; 
    float dt = (float)(currentTimer - oldTimer); 
    println(dt);
    
    dt=.002;
    
    velocity = setVel(dt);
    temposition = setPos(dt);
    
    oldTimer = currentTimer; 
  }
  
  boolean isHit(PVector pos){
    vec_ee2Planet = (pos.copy()).sub(pos_ee);
    float vec_ee2Planet_magnitude = vec_ee2Planet.mag();
    pen = vec_ee2Planet_magnitude - (radius + r_ee);
    if(pen < 0){
      return true;
    }else{
      return false;
    }
  }
  
  PVector setPos(float dt){
    PVector pos = (((f.copy()).div(2*mass)).mult(dt*dt)).add(((velocity.copy()).mult(dt))).add(position);
    //PVector pos = temposition.add(velocity);
    return pos;
  }
  
  PVector setVel(float dt){
    PVector vel = ((((f.copy()).div(mass)).mult(dt)).add(velocity)).mult(damping);
    return vel;
  }
  
  //void display(color colorOFPlanet){
  //  fill(colorOFPlanet);
  //  ellipse(position.x, position.y, radius*2, radius*2);
  //}

}

//==================Functions==================================

void setup() {
  
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
  
  //create planets
  planets[0] = new Planet(d/2, 5, new PVector(-50,100), color(7,80,175));    //create Earth
  planets[1] = new Planet((d/2)*0.532, 5*0.107, new PVector(50,100), color(196,69,27));    //create Mars
  planets[2] = new Planet((d/2)*0.383, 5*0.0553, new PVector(75,100), color(219,209,206));    //create Mercury
  
  createpantograph();
}

        
/**
 * @brief    Main draw function, updates frame at perscribed frame rate
 */
void draw(){
  count = millis(); 
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
    
            for(Planet planet: planets){
              planet.update();
              
            }

}

  /******* ANIMATION TIMER ********/ 
  if((count-animation_count) > 16){
    angles.set(haply_2DoF.mechanisms.get_angle());
    pos_ee.set(haply_2DoF.mechanisms.get_coordinate());
    update_animation(angles.x, angles.y, pos_ee.x, pos_ee.y);
  }
  
  /********** HAPTICS TIMER *************/ 
  
  if((count - haptics_count) > 1){
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
  
  
  for(int i=0; i<planetno; i++){
    planetshape[i] = createShape(ELLIPSE, device_origin.x, device_origin.y, 2*planets[i].radius, 2*planets[i].radius);
    planetshape[i].setFill(planets[i].colorOfPlanet);
  }


}

void update_animation(float th1, float th2, float x_E, float y_E){
    background(0); // To clean up the left-overs of drawings from the previous loop!

  
  pantograph.setVertex(1,device_origin.x+l*cos(th1), device_origin.y+l*sin(th1)); // Vertex A with th1 from encoder reading
  pantograph.setVertex(3,device_origin.x+d+l*cos(th2), device_origin.y+l*sin(th2)); // Vertex B with th2 from encoder reading
  pantograph.setVertex(2,device_origin.x+x_E, device_origin.y+y_E); // Vertex E from Fwd Kin calculations  
  
  shape(pantograph); // Display the pantograph
  shape(joint1);
  shape(joint2); 
  shape(handle,x_E, y_E); 
    stroke(255); 
  
  for(int i=0; i<planetno; i++){
    shape(planetshape[i], planets[i].temposition.x, planets[i].temposition.y);
  }

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
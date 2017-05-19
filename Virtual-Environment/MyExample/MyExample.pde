import processing.serial.*;


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
int             s                 = 10;      //size of planet
float           g                 = 9.8;    //gravity of planet
int             m                 = 5;      //mass of planet

PVector          device_origin    = new PVector (0, 0) ; 

//Text to be displayed
String intro = "Hit one of the nine planets with the handle";
String intro2 = "and feel their mass and gravity in scale";
String displayed = "";
PFont font;
boolean display = true;
int interval = 8000; 
int time;

//dynamics of planets
long            oldTimer          = 0; 
float           b_air             = .2; // grams/s
// generic data for a 2DOF device
// joint space
PVector        angles            = new PVector(0,0);
PVector        torques           = new PVector(0,0);

//task space
PVector         pos_ee           = new PVector(0,0);
PVector         f_ee             = new PVector(0,0);         //force to handle

int planetno = 9;
Planet[] planets = new Planet[planetno];
PShape[] planetshape = new PShape[planetno];

//==================ClassPlanet================================
class Planet{
  float radius;              //radius
  float mass;                //mass
  int k = 2000;                //some constant
  float gravity;                //gravity
  float pen = 0.0;            //distance between planet and handle
  PVector position;          //position
  PVector temposition;        //temperary position
  PVector velocity = new PVector(0,0);          //velocity
  PVector f = new PVector(0,0);            //force acting on planet
  PVector damping  = new PVector(0,0);      //damping force
  PVector contact = new PVector(0,0);      //contact force
  boolean hit = false;             //hit by handle
  PVector vec_ee2Planet = new PVector(0,0);
  color colorOfPlanet;
  
  //Constructor
  Planet(float r, float m, float gra, PVector pos, color c){
    radius = r;
    mass = m;
    gravity = gra;
    position = pos;
    temposition = pos;
    colorOfPlanet = c;
  }
  
  void update(){
    // INTEGRATE THE ACCELERATION TO GET THE STATES OF THE BALL
    long currentTimer = count; 
    float dt = (float)(currentTimer - oldTimer); 
    println(dt);
    dt=.002;
       
    //contact force
    hit = isHit();
    if(hit){
      contact = vec_ee2Planet.normalize();
      contact = contact.mult(-k * pen);
      
    }else{
      contact.set(0,0);
    }
    
    //forces due to damping
    damping = (velocity.copy()).mult(-b_air);
      
    //sum of forces
    f = (contact.copy()).add(damping);
    velocity = (((f.copy()).div(mass)).mult(dt)).add(velocity);
    
    if((velocity.x > -1 || velocity.x < 1) && (velocity.y > -1 || velocity.y < 1)){
      
      float fx = -gravity * (temposition.x - position.x);
      float fy = -gravity * (temposition.y - position.y);
      f = new PVector(fx, fy);
      velocity = (((f.copy()).div(mass)).mult(dt)).add(velocity);
    }
    
    temposition = (((f.copy()).div(2*mass)).mult(dt*dt)).add(((velocity.copy()).mult(dt))).add(temposition);
    f_ee = (contact.copy()).mult(-1);
    
    
    haply_2DoF.mechanisms.torqueCalculation(f_ee.array());
    torques.set(haply_2DoF.mechanisms.get_torque());
    haply_2DoF.motors[0].set_torque(torques.x);
    haply_2DoF.motors[1].set_torque(torques.y);
    
    oldTimer = currentTimer;
  }
  
  boolean isHit(){
    vec_ee2Planet = (temposition.copy()).sub(pos_ee);
    float vec_ee2Planet_magnitude = vec_ee2Planet.mag();
    pen = vec_ee2Planet_magnitude - (radius + r_ee);
    if(pen < 0){
      return true;
    }else{
      return false;
    }
  }
  
}

//==================Functions==================================

void setup() {
  
  /*Setup for the graphic display window and drawing objects*/
  
  size(600, 400, P2D);
  font = createFont("arial", 20);
  background(0);
  displayed = intro;
  time = millis();
  textFont(font);
  fill(255);
  frameRate(baseFrameRate);
   
  
  /* Initialization of the Board, Device, and Device Components*/ 
  
  //BOARD
  haply_board = new Board(Serial.list()[0], 0);
  
  //DEVICE
  haply_2DoF = new Device(degreesOfFreedom.TwoDOF, deviceID, haply_board);
  
  haply_2DoF.device_set_parameters();
               
  device_origin.add(width/2, height/4 );
  
  //create planets
  planets[0] = new Planet((s/2)*0.383, m*0.0553, g*0.378, new PVector(-75,70), color(219,209,206));    //create Mercury
  planets[1] = new Planet((s/2)*0.949, m*0.815, g*0.907, new PVector(-70,80), color(201,168,90));  //create Venus
  planets[2] = new Planet((s/2), m, g, new PVector(-60,100), color(7,80,175));    //create Earth
  planets[3] = new Planet((s/2)*0.532, m*0.107, g*0.377, new PVector(-30,130), color(196,69,27));    //create Mars
  planets[4] = new Planet((s/2)*11.21, m*317.8, g*2.36, new PVector(0,200), color(239,226,196)); //create Jupiter
  planets[5] = new Planet((s/2)*9.45, m*95.2, g*0.916, new PVector(70,200), color(247,236,178)); //create Saturn
  planets[6] = new Planet((s/2)*4.01, m*14.5, g*0.889, new PVector(100, 130), color(209,243,249)); //create Uranus
  planets[7] = new Planet((s/2)*3.88, m*17.1, g*1.12, new PVector(120, 100), color(77,112,183)); //create Neptune
  planets[8] = new Planet((s/2)*0.186, m*0.0025, g*0.071, new PVector(150, 80), color(255,255,255)); //create Pluto
  
  createpantograph();
}

        
/**
 * @brief    Main draw function, updates frame at perscribed frame rate
 */
void draw(){
  background(0);
 
  if(display){
   fill(255);
   text(intro, width/2 - textWidth(displayed)/2, height/2);
   text(intro2, width/2 - textWidth(displayed)/2, height/2 + 30);
   if(millis() - time > interval){
     display = false;
   }
  }else{
  
    count = millis(); 
    scale(1,-1);
    translate(0,-height); 
    
   for(int i =0; i< planetno; i++){
     drawHelper(i);
   }
  }
    
}

//so that the handle can experience force from each planet
void drawHelper(int i){
  if(haply_board.data_available()){

   /*** GET END-EFFECTOR POSITION (TASK SPACE)****/ 
            haply_2DoF.device_read_angles();
            
            /* forward kinematics calculation */
            angles.x = haply_2DoF.encoders[0].get_angle(); 
            angles.y = haply_2DoF.encoders[1].get_angle();
            haply_2DoF.mechanisms.forwardKinematics(angles.array());
            pos_ee.set( haply_2DoF.mechanisms.get_coordinate());

              planets[i].update();
            
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
    noStroke();
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
import processing.serial.*;

 //====spring======
  class Spring { 
  // Screen values 
  float xpos, ypos;
  float tempxpos, tempypos; 
  int size = 20; 
  boolean over = false; 
  boolean move = false; 

  // Spring simulation constants 
  float mass;       // Mass 
  float k = 0.2;    // Spring constant 
  float damp;       // Damping 
  float rest_posx;  // Rest position X 
  float rest_posy;  // Rest position Y 

  // Spring simulation variables 
  //float pos = 20.0; // Position 
  float velx = 0.0;   // X Velocity 
  float vely = 0.0;   // Y Velocity 
  float accel = 0;    // Acceleration 
  float force = 0;    // Force 

  Spring[] friends;
  int me;

  // Constructor
  Spring(float x, float y, float s, float d, float m, 
  float k_in, Spring[] others, int id) { 
    xpos = tempxpos = x; 
    ypos = tempypos = y;
    rest_posx = x;
    rest_posy = y;
    size = (int)s;
    damp = d; 
    mass = m; 
    k = k_in;
    friends = others;
    me = id;
  } 

  void update() { 
    if (move) { 
      rest_posy = mouseY; 
      rest_posx = mouseX;
    } 

    force = -k * (tempypos - rest_posy);  // f=-ky 
    accel = force / mass;                 // Set the acceleration, f=ma == a=f/m 
    vely = damp * (vely + accel);         // Set the velocity 
    tempypos = tempypos + vely;           // Updated position 

    force = -k * (tempxpos - rest_posx);  // f=-ky 
    accel = force / mass;                 // Set the acceleration, f=ma == a=f/m 
    velx = damp * (velx + accel);         // Set the velocity 
    tempxpos = tempxpos + velx;           // Updated position 


    if ((overEvent() || move) && !otherOver() ) { 
      over = true;
    } else { 
      over = false;
    }
  } 

  // Test to see if mouse is over this spring
  boolean overEvent() {
    float disX = tempxpos - mouseX;
    float disY = tempypos - mouseY;
    if (sqrt(sq(disX) + sq(disY)) < size/2 ) {
      return true;
    } else {
      return false;
    }
  }

  // Make sure no other springs are active
  boolean otherOver() {
    for (int i=0; i<num; i++) {
      if (i != me) {
        if (friends[i].over == true) {
          return true;
        }
      }
    }
    return false;
  }

  void display() { 
    fill(7,80,175);
    ellipse(tempxpos, tempypos, size, size);
  } 

  void pressed() { 
    if (over) { 
      move = true;
    } else { 
      move = false;
    }
  } 

  void released() { 
    move = false; 
    rest_posx = xpos;
    rest_posy = ypos;
  }
} 
//========endOfSpring==================
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

PShape          pantograph, joint1, joint2, handle;
Spring         Earth, Mars, Mercury;             //add more later, these are for testing 


int             l                 = 2*50;        
int             L                 = 2*70;
int             d                 = 2*20;
int             r_ee              = d/3;     //radius of handle

float             r_Earth            = d/2;     
float             r_Mars            = (d/2) * 0.532;     
float             r_Mercury            = (d/2) * 0.383;     

PVector          device_origin    = new PVector (0, 0) ; 

//dynamics of planets
long            oldTimer          = 0; 
int             m_Earth            = 5; //grams    may change
int             k_Earth            = 2000; //grams/s^2  may change
float           pen_Earth          = 0.0; // mm
float           b_air             = .2; // grams/s
//PVector         f_gravity         = new PVector(0, -8000); // mm/s^2    setting in space
int             m_Mars            = 5; //grams    may change
int             k_Mars            = 2000; //grams/s^2  may change
float           pen_Mars          = 0.0; // mm

int             m_Mercury            = 5; //grams    may change
int             k_Mercury            = 2000; //grams/s^2  may change
float           pen_Mercury          = 0.0; // mm


//Initial Conditions
PVector         pos_Earth          = new PVector(-50,100);  // mm
PVector         vel_Earth          = new PVector(0,0); // mm/s
PVector         f_Earth            = new PVector(0,0); // uN

PVector         pos_Mars          = new PVector(50,100);  // mm
PVector         vel_Mars          = new PVector(0,0); // mm/s
PVector         f_Mars            = new PVector(0,0); // uN

PVector         pos_Mercury          = new PVector(75,100);  // mm
PVector         vel_Mercury          = new PVector(0,0); // mm/s
PVector         f_Mercury            = new PVector(0,0); // uN

PVector         f_dampingEarth    = new PVector(0,0);
PVector         f_contactEarth    = new PVector(0,0);

PVector         f_dampingMars    = new PVector(0,0);
PVector         f_contactMars    = new PVector(0,0);

PVector         f_dampingMercury    = new PVector(0,0);
PVector         f_contactMercury    = new PVector(0,0);

// generic data for a 2DOF device
// joint space
PVector        angles            = new PVector(0,0);
PVector        torques           = new PVector(0,0);

//task space
PVector         pos_ee           = new PVector(0,0);
PVector         f_eeEarth             = new PVector(0,0);
PVector         f_eeMars             = new PVector(0,0);
PVector         f_eeMercury             = new PVector(0,0);

//======spring=======
int num = 3;
Spring[] springs = new Spring[num];


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
            
             
    
/*** PHYSICS OF THE SIMULATION ****/ 


//Contact Forces


    PVector vec_ee2Earth = (pos_Earth.copy()).sub(pos_ee);
    PVector vec_ee2Mars = (pos_Mars.copy()).sub(pos_ee);
    PVector vec_ee2Mercury = (pos_Mercury.copy()).sub(pos_ee);
    
    float vec_ee2Earth_magnitude = vec_ee2Earth.mag();
    float vec_ee2Mars_magnitude = vec_ee2Mars.mag();
    float vec_ee2Mercury_magnitude = vec_ee2Mercury.mag();
    
    pen_Earth = vec_ee2Earth_magnitude - (r_Earth+r_ee);
    pen_Mars = vec_ee2Mars_magnitude - (r_Mars+r_ee);
    pen_Mercury = vec_ee2Mercury_magnitude - (r_Mercury+r_ee);
     
  
  // planet forces
  //Earth
    if(pen_Earth<0){
       f_contactEarth= vec_ee2Earth.normalize();
       f_contactEarth= f_contactEarth.mult(-k_Earth*pen_Earth);  // since pen_Earth is negative k_ball must be negative to ensure the force acts along the end-effector to the ball
      
    }
    else{
      f_contactEarth.set(0,0); 
    }
    //Mars
    if(pen_Mars<0){
       f_contactMars= vec_ee2Mars.normalize();
       f_contactMars= f_contactMars.mult(-k_Mars*pen_Mars);  // since pen_ball is negative k_ball must be negative to ensure the force acts along the end-effector to the ball
      
    }
    else{
      f_contactMars.set(0,0); 
    }
    //Mercury
    if(pen_Mercury<0){
       f_contactMercury= vec_ee2Mercury.normalize();
       f_contactMercury= f_contactMercury.mult(-k_Mercury*pen_Mercury);  // since pen_ball is negative k_ball must be negative to ensure the force acts along the end-effector to the ball
      
    }
    else{
      f_contactMercury.set(0,0); 
    }
  
// forces due to damping

    f_dampingEarth = (vel_Earth.copy()).mult(-b_air);     
    f_dampingMars = (vel_Mars.copy()).mult(-b_air);     
    f_dampingMercury = (vel_Mercury.copy()).mult(-b_air);     
    
 
// sum of forces 

    f_Earth = (f_contactEarth.copy()).add(f_dampingEarth); 
    f_eeEarth = (f_contactEarth.copy()).mult(-1);
    
    f_Mars = (f_contactMars.copy()).add(f_dampingMars); 
    f_eeMars = (f_contactMars.copy()).mult(-1);
    
    f_Mercury = (f_contactMercury.copy()).add(f_dampingMercury); 
    f_eeMercury = (f_contactMercury.copy()).mult(-1);
    
    haply_2DoF.mechanisms.torqueCalculation(f_eeEarth.array());
    haply_2DoF.mechanisms.torqueCalculation(f_eeMars.array());
    haply_2DoF.mechanisms.torqueCalculation(f_eeMercury.array());
    torques.set(haply_2DoF.mechanisms.get_torque());
    haply_2DoF.motors[0].set_torque(torques.x);
    haply_2DoF.motors[1].set_torque(torques.y);
    
    for (Spring spring : springs) { 
    spring.update(); 
    spring.display();
  }
    

// INTEGRATE THE ACCELERATION TO GET THE STATES OF THE BALL
long currentTimer = count; 
float dt = (float)(currentTimer - oldTimer); 
println(dt); 
//dt = dt/1000; 
//dt = (dt < 0.001 )? 0.002 : dt; 
//println(dt);
dt=.002; 
//println(dt);
pos_Earth = (((f_Earth.copy()).div(2*m_Earth)).mult(dt*dt)).add(((vel_Earth.copy()).mult(dt))).add(pos_Earth);
vel_Earth = (((f_Earth.copy()).div(m_Earth)).mult(dt)).add(vel_Earth); 

pos_Mars = (((f_Mars.copy()).div(2*m_Mars)).mult(dt*dt)).add(((vel_Mars.copy()).mult(dt))).add(pos_Mars);
vel_Mars = (((f_Mars.copy()).div(m_Mars)).mult(dt)).add(vel_Mars); 

pos_Mercury = (((f_Mercury.copy()).div(2*m_Mercury)).mult(dt*dt)).add(((vel_Mercury.copy()).mult(dt))).add(pos_Mercury);
vel_Mercury = (((f_Mercury.copy()).div(m_Mercury)).mult(dt)).add(vel_Mercury); 

oldTimer = currentTimer; 

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
  
  //Earth = createShape(ELLIPSE, device_origin.x, device_origin.y, 2*r_Earth, 2*r_Earth);
  //Earth.setStroke(color(255));
  Earth = new Spring(pos_Earth.x, pos_Earth.y, 2*r_Earth, 10.0, 10.0, 0.1, springs, 0);
  springs[0] = Earth;
  //Earth.setFill(color(7,80,175));
  
  //Mars = createShape(ELLIPSE, device_origin.x, device_origin.y, 2*r_Mars, 2*r_Mars);
  //Mars.setStroke(color(255));
  Mars = new Spring(pos_Mars.x, pos_Mars.y, 2*r_Mars, 10.0, 10*0.107, 0.1, springs, 1);
  springs[1] = Mars;
  //Mars.setFill(color(196, 69, 27));
  
  //Mercury = createShape(ELLIPSE, device_origin.x, device_origin.y, 2*r_Mercury, 2*r_Mercury);
  //Mercury.setStroke(color(255));
  Mercury = new Spring(pos_Mercury.x, pos_Mercury.y, 2*r_Mercury, 10.0, 10*0.0553, 0.1, springs, 2);
  springs[2] = Mercury;
  //Mercury.setFill(color(219, 209, 206));


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
  //shape(Earth, pos_Earth.x, pos_Earth.y); 
  
  //stroke(255);
  //shape(Mars, pos_Mars.x, pos_Mars.y); 
  //stroke(255);
  //shape(Mercury, pos_Mercury.x, pos_Mercury.y); 
  //stroke(255);
  

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
  
  
 
/* library imports *****************************************************************************************************/ //<>// //<>// //<>// //<>// //<>// //<>// //<>// //<>// //<>// //<>// //<>// //<>// //<>// //<>// //<>// //<>// //<>//

import processing.serial.*;
import com.dhchoi.CountdownTimer;
import com.dhchoi.CountdownTimerService;

/* Device block definitions ********************************************************************************************/
Device            haply_2DoF;
byte              deviceID                   = 5;
Board             haply_board;
DeviceType        degreesOfFreedom;
boolean           rendering_force                 = false;


/* Simulation Speed Parameters ****************************************************************************************/
final long        SIMULATION_PERIOD          = 1; //ms
final long        HOUR_IN_MILLIS             = 36000000;
CountdownTimer    haptic_timer;
float             dt                        = SIMULATION_PERIOD/1000.0; 


/* generic data for a 2DOF device */
/* joint space */
PVector           angles                    = new PVector(0, 0);
PVector           torques                   = new PVector(0, 0);

/* task space */
PVector           pos_ee                    = new PVector(0, 0);
PVector           pos_ee_last               = new PVector(0, 0); 
PVector           f_ee                      = new PVector(0, 0); 

/* Graphic objects */
float pixelsPerCentimeter= 40.0; //this is the resolution of my screen divided by the number of centimeters  i.e. a 1600px x 800px display with a 40 cm screen -> 40 pixels/cm
FWorld world;  
HVirtualCoupling s; 
PImage haply_avatar;  

float worldWidth = 16.0;  
float worldHeight = 10.0; 


float edgeTopLeftX = 0.0; 
float edgeTopLeftY =-2.0; 
float edgeBottomRightX = worldWidth; 
float edgeBottomRightY = worldHeight; 

int d = 30; //size
int m = 5;  //mass
int planetno = 9;
float[] planetsRadius = new float[planetno];
float[] planetsMass = new float[planetno];
PImage[] planetsPics = new PImage[planetno]; 





  float posX; 
  float posY;



FCompound sat; 

void setup() {
  
  
  size(1000, 800); // (worldWidth*pixelsPerCentimeter, worldHeight*pixelsPerCentimeter) must input as number


  planetsRadius[0] = (d/2)*0.383;
  planetsRadius[1] = (d/2)*0.949;
  planetsRadius[2] = d/2;
  planetsRadius[3] = d/2*.532; 
  planetsRadius[4] = d/2*11.21; 
    //planetsRadius[4] = d/2*10.21; 

  planetsRadius[5] = d/2*9.45 ; 
  planetsRadius[6] = d/2*4.01; 
  planetsRadius[7] = d/2*3.88; 
  planetsRadius[8] = d/2*0.186; 
  
  planetsMass[0] = m*0.0553;
  planetsMass[1] = m*0.815;
  planetsMass[2] = m;
  planetsMass[3] = m*0.107;
  planetsMass[4] = m*217.8;
  planetsMass[5] = m*95.2;
  planetsMass[6] = m*14.5;
  planetsMass[7] = m*17.1;
  planetsMass[8] = m*.0025;
 
  planetsPics[0] = loadImage("../img/mercury.png"); 
  planetsPics[1] = loadImage("../img/venus.png"); 
  planetsPics[2] = loadImage("../img/earth.png"); 
  planetsPics[3] = loadImage("../img/mars.png"); 
  planetsPics[4] = loadImage("../img/jupiterhc.png"); 
  planetsPics[5] = loadImage("../img/saturn.png"); 
  planetsPics[6] = loadImage("../img/uranus.png"); 
  planetsPics[7] = loadImage("../img/neptune.png"); 
  planetsPics[8] = loadImage("../img/pluto.png"); 
 
  /* BOARD */
  haply_board = new Board(this, "COM5", 0);

  /* DEVICE */
  haply_2DoF = new Device(degreesOfFreedom.HaplyTwoDOF, deviceID, haply_board);

  hAPI_Fisica.init(this); 
  hAPI_Fisica.setScale(pixelsPerCentimeter); 
  world = new FWorld();

 //<>//


// Setup the Virtual Coupling Contact Rendering Technique




  world.setGravity((0.0), (0.0)); //1000 cm/(s^2)
  //world.setEdges((edgeTopLeftX), (edgeTopLeftY), (edgeBottomRightX), (edgeBottomRightY)); 
  world.remove(world.bottom); 
  world.setEdgesRestitution(.4);
  world.setEdgesFriction(0.5);
  
  
  //key =51; ; 
  //   posX= worldWidth/2.0; 
  // posY= worldHeight+planetsRadius[2]/2.0-3; 
  
  ////world.clear(); 
  //  // Insert Different sized circle objects
  //planet = new FCircle(planetsRadius[2]);
  //planet.setPosition(posX, posY);
  // planetsPics[2].resize((int)(hAPI_Fisica.worldToScreen(planetsRadius[2])), (int)(hAPI_Fisica.worldToScreen(planetsRadius[2])));
  // planet.attachImage(planetsPics[2]); 
  //planet.setStatic(true); 
  //world.add(planet);   
  
  

  
  s= new HVirtualCoupling((2)); 
  s.h_avatar.setFill(255,0,0); 
  s.init(world, edgeTopLeftX+worldWidth/2, edgeTopLeftY+2); 
  haply_avatar = loadImage("../img/sat.png"); 
  haply_avatar.resize((int)(hAPI_Fisica.worldToScreen(2)), (int)(hAPI_Fisica.worldToScreen(2)));
  s.h_avatar.attachImage(haply_avatar); 


  
  world.draw();
  


  haptic_timer = CountdownTimerService.getNewCountdownTimer(this).configure(SIMULATION_PERIOD, HOUR_IN_MILLIS).start();
  
  frameRate(60); 
}

void draw() {
  background(0); 
   
  if(!rendering_force){
    
    //s.drawContactVectors(this); 
    
   }
    world.draw();
    //world.drawDebug();  
    
    
    
}

/**********************************************************************************************************************
 * Haptics simulation event, engages state of physical mechanism, calculates and updates physics simulation conditions
 **********************************************************************************************************************/ 

void onTickEvent(CountdownTimer t, long timeLeftUntilFinish){
  
  rendering_force = true;
   
  if (haply_board.data_available()) {
    /* GET END-EFFECTOR STATE (TASK SPACE) */
        
    angles.set(haply_2DoF.get_device_angles()); 
    pos_ee.set( haply_2DoF.get_device_position(angles.array()));
    pos_ee.set(pos_ee.copy().mult(100)); 
    
  }

  s.setToolPosition(edgeTopLeftX+worldWidth/2-(pos_ee).x+1.0, edgeTopLeftY+(pos_ee).y);  
  s.updateCouplingForce();
 
 
  f_ee.set(-s.getVCforceX(), s.getVCforceY());
  PVector fg = new PVector(0,0);
 float G = 70000;
 float fg_scaler; 
 
 
   PVector planetPos = new PVector(posX, posY);
   PVector shipPos = new PVector(s.getToolPositionX(), s.getToolPositionY()); 
   PVector planetToShip = (shipPos.copy().sub(planetPos));
    //println(planetToShip.mag());
    //println(shipPos); 

   fg_scaler = G*mass/(planetToShip.mag()*planetToShip.mag()); 
   fg = planetToShip.normalize(); 
   fg.mult(-fg_scaler); 
  
  s.h_avatar.addForce(fg.x, fg.y); 
  
  f_ee.div(10000); //
  haply_2DoF.set_device_torques(f_ee.array());
  torques.set(haply_2DoF.mechanisms.get_torque());
  haply_2DoF.device_write_torques();
  
  
  world.step(1.0f/1000.0f);
  
  rendering_force = false;
}


/* Timer control event functions **************************************************************************************/

/**
 * haptic timer reset
 */
void onFinishEvent(CountdownTimer t){
  println("Resetting timer...");
  haptic_timer.reset();
  haptic_timer = CountdownTimerService.getNewCountdownTimer(this).configure(SIMULATION_PERIOD, HOUR_IN_MILLIS).start();
}
float mass; 

FCircle planet; 
void keyPressed() {
  
  if(48<key && key<58){
  world.remove(planet); 
  
  float radius; 
  

   radius = planetsRadius[key-49]; 
   mass = planetsMass[key-49]; 
    posX= worldWidth/2.0; 
    posY= worldHeight+planetsRadius[key-49]/2.0-3; 
   

  //world.clear(); 
    // Insert Different sized circle objects
  planet = new FCircle(radius);
  //planet.setSize(radius); 
  planet.setPosition(posX, posY);
  planet.setFill(random(255),random(255),random(255));
  planet.setStatic(true); 
  
  //println(key-49); 
  //if(((key-49) != 4) && ((key -49) != 5)){
   planetsPics[key-49].resize((int)(hAPI_Fisica.worldToScreen(radius)), (int)(hAPI_Fisica.worldToScreen(radius)));
   planet.attachImage(planetsPics[key-49]); 
  //}
  
  world.add(planet);  
  
   
  }
  
     
}
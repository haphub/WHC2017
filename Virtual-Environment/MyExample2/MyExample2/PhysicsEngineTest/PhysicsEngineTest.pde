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
FCircle g; 
FCircle e; 
HVirtualCoupling s; 
PImage haply_avatar;  

float worldWidth = 16.0;  
float worldHeight = 10.0; 


float edgeTopLeftX = 0.0; 
float edgeTopLeftY =-2.0; 
float edgeBottomRightX = worldWidth; 
float edgeBottomRightY = worldHeight; 

int d = 5; //size
int m = 5;  //mass
int planetno = 9;
float[] planetsRadius = new float[planetno];
float[] planetsMass = new float[planetno];
float[] planetsPosX = new float[planetno];
float[] planetsPosY = new float[planetno];



void setup() {
  



  
  size(640, 400); // (worldWidth*pixelsPerCentimeter, worldHeight*pixelsPerCentimeter) must input as number


  planetsRadius[0] = (d/2)*0.383;
  planetsRadius[1] = (d/2)*0.949;
  planetsRadius[2] = d/2;
  
  planetsMass[0] = m*0.0553;
  planetsMass[1] = m*0.815;
  planetsMass[2] = m;
  
  planetsPosX[0] = edgeTopLeftX+2;    //cm
  planetsPosX[1] = edgeTopLeftX+6;
  planetsPosX[2] = edgeTopLeftX+10;
  
  planetsPosY[0] = edgeTopLeftY+5;
  planetsPosY[1] =  edgeTopLeftY+8;
  planetsPosY[2] =  edgeTopLeftY+4;
  /* BOARD */
  haply_board = new Board(this, "COM4", 0);

  /* DEVICE */
  haply_2DoF = new Device(degreesOfFreedom.HaplyTwoDOF, deviceID, haply_board);

  hAPI_Fisica.init(this); 
  hAPI_Fisica.setScale(pixelsPerCentimeter); 
  world = new FWorld();

 //<>//
  // Insert Different sized circle objects
  FCircle e1 = new FCircle(1);
  e1.setPosition(planetsPosX[0], planetsPosY[0]);
  e1.setFill(random(255),random(255),random(255));
  e1.setStatic(true); 
  world.add(e1); 
  
  FCircle e2 = new FCircle(planetsRadius[1]);
  e2.setPosition(planetsPosX[1], planetsPosY[1]); 
  e2.setFill(random(255),random(255),random(255));
  e2.setStatic(true); 
  world.add(e2); 
  
  FCircle e3 = new FCircle(planetsRadius[2]);
  e3.setPosition(planetsPosX[2], planetsPosY[2]);
  e3.setFill(random(255),random(255),random(255));
  e3.setStatic(true); 
  world.add(e3);
  


// Setup the Virtual Coupling Contact Rendering Technique

  s= new HVirtualCoupling((.6)); 
  s.h_avatar.setFill(255,0,0); 
  s.init(world, edgeTopLeftX+worldWidth/2, edgeTopLeftY+2); 
  haply_avatar = loadImage("../img/Haply_avatar.png"); 
  haply_avatar.resize((int)(hAPI_Fisica.worldToScreen(.6)), (int)(hAPI_Fisica.worldToScreen(.6)));
  s.h_avatar.attachImage(haply_avatar); 


  world.setGravity((0.0), (1000.0)); //1000 cm/(s^2)
  //world.setEdges((edgeTopLeftX), (edgeTopLeftY), (edgeBottomRightX), (edgeBottomRightY)); 
  world.setEdgesRestitution(.4);
  world.setEdgesFriction(0.5);
  
  world.draw();
  
  haptic_timer = CountdownTimerService.getNewCountdownTimer(this).configure(SIMULATION_PERIOD, HOUR_IN_MILLIS).start();
  
  frameRate(60); 
}

void draw() {
  background(255); 
   
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
 float G = 1000;
 
 
 for(int i =1; i<2; i++){
   PVector planetPos = new PVector(planetsPosX[i], planetsPosY[i]);
   PVector shipPos = new PVector(s.getToolPositionX(), s.getToolPositionY()); 
   PVector planetToShip = (shipPos.copy().sub(planetPos));
    println(planetPos);
    println(shipPos); 

   fg = fg.copy().add(( planetToShip.copy().normalize().mult((-G*planetsMass[i])/pow(planetToShip.mag(),2))));  
 
 
 }

 fg.set(-fg.x, fg.y); 
 println(fg); 
 f_ee.add(fg);
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
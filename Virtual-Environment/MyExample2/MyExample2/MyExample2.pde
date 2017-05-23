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
float[] planetsRadius = new float[planetno];
float[] planetsMass = new float[planetno];
float[] planetsPosX = new float[planetno];
float[] planetsPosy = new float[planetno];

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
  
  
  createpantograph();
}
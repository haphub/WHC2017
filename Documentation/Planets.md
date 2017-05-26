# Planets
### Motivation

The traditional way for children to learn physics is from textbooks and doing exercises. However, force is just a magnitude in textbooks. How does it really feel in reality? In order to allow children to better understand physics, how objects and forces interact with each other, the Haply device comes into play. This example program is going to show the eight planets and Pluto on the screen. By moving the Haply device handle to control the spaceship closer to any one of them, the player is able to experience different gravitational forces pulling the spaceship towards the planet. This may spark children's interest in learning more about the planets, or even in learning programming, to make their own little physics games.

### Demo: how to run the code

1. Connect the Haply device to the computer using the USB wire
2. Open the Planets.pde
3. Click on the play button on the top left corner
![Image of play button](https://github.com/haphub/WHC2017/blob/hailey_dev/Documentation/DemoRun.png)
4. This screen will be shown
![Image of play screen](https://github.com/haphub/WHC2017/blob/hailey_dev/Documentation/DemoPlanet0.png)
5. Now, you can press from 1-9 to experience the gravitational force of different planets
![Image of Venus](https://github.com/haphub/WHC2017/blob/hailey_dev/Documentation/DemoPlanet2.png) Venus is shown when 2 is pressed
![Image of Earth](https://github.com/haphub/WHC2017/blob/hailey_dev/Documentation/DemoPlanet3.png) Earth is shown when 3 is pressed

### Hardware explanation (optional)

### Code explanation

This session is going to explain the code step by step. (text after // and within /* and */ are comments on the code, they do not affect how the code runs)

There are libraries with handy built-in functions that we can use directly. To use those libraries, we need to import them.
```sh
import processing.serial.*;
import com.dhchoi.CountdownTimer;
import com.dhchoi.CountdownTimerService;
```
To define the device block so we can connect the program to the device.
```sh
Device            haply_2DoF;
byte              deviceID                   = 5;
Board             haply_board;
DeviceType        degreesOfFreedom;
boolean           rendering_force                 = false;
```
We will also need to create some speed parameters because the screen is going to be updated by the device constantly.
```sh
final long        SIMULATION_PERIOD          = 1; //ms
final long        HOUR_IN_MILLIS             = 36000000;
CountdownTimer    haptic_timer;
float             dt                        = SIMULATION_PERIOD/1000.0; 
```
We will want some parameters for storing the status of the device, the angles of the joints connecting the handle to the motor, the torques, the position of the handle, and the force produced by the motor.
```sh
/* joint space */
PVector           angles                    = new PVector(0, 0);
PVector           torques                   = new PVector(0, 0);

/* task space */
PVector           pos_ee                    = new PVector(0, 0);
PVector           pos_ee_last               = new PVector(0, 0); 
PVector           f_ee                      = new PVector(0, 0); 
```
We will need to define some graphic objects using these code.
```sh
float pixelsPerCentimeter= 40.0; //this is the resolution of my screen divided by the number of centimeters  i.e. a 1600px x 800px display with a 40 cm screen -> 40 pixels/cm
FWorld world;  
HVirtualCoupling s; 
PImage haply_avatar; 
```
Then the size of the screen.
```sh
float worldWidth = 16.0;
float worldHeight = 10.0;
```
Here, we set the x and y values of the positions of top left corner and bottom right corner.
```sh
float edgeTopLeftX = 0.0; 
float edgeTopLeftY =-2.0; 
float edgeBottomRightX = worldWidth; 
float edgeBottomRightY = worldHeight; 
```
Since the diameter and mass of the planets are scaled according to the Earth, we are going to set these values of the Earth first.
```sh
int d = 30;  //cm
int m = 5;
```
We need three arrays, one for storing the radius of the planets, one for storing the mass of the planets, and last one for storing the images of the planets. By using arrays, we can update the screen to display the planets and the force the player will experience in a for-loop.
```sh
int planetno = 9;
float[] planetsRadius = new float[planetno];
float[] planetsMass = new float[planetno];
PImage[] planetsPics = new PImage[planetno]; 
```
We also need two parameters to store the planet x-position and y-position. The FCompound sat will be the spaceship we are controling.
```sh
float posX;
float posY;
FCompound sat;
```
Now, we are going to write the first function of this program to set things up. 
```sh
void setup(){

}
```
Inside this function (within the curly parenthesis), we need to set the size. 
```sh
size(1000,800);  // (worldWidth*pixelsPerCentimeter, worldHeight*pixelsPerCentimeter) must input as number
```
After setting the size, we store some radius of planets inside the planetsRadius array.
```sh
planetsRadius[0] = (d/2)*0.383;
planetsRadius[1] = (d/2)*0.949;
planetsRadius[2] = d/2;
planetsRadius[3] = d/2*.532; 
planetsRadius[4] = d/2*11.21; 
planetsRadius[5] = d/2*9.45 ; 
planetsRadius[6] = d/2*4.01; 
planetsRadius[7] = d/2*3.88; 
planetsRadius[8] = d/2*0.186;
```
Then we store the mass.
```sh
planetsMass[0] = m*0.0553;
planetsMass[1] = m*0.815;
planetsMass[2] = m;
planetsMass[3] = m*0.107;
planetsMass[4] = m*217.8;
planetsMass[5] = m*95.2;
planetsMass[6] = m*14.5;
planetsMass[7] = m*17.1;
planetsMass[8] = m*.0025;
```
Then the images of the planets.
```sh
planetsPics[0] = loadImage("../img/mercury.png"); 
planetsPics[1] = loadImage("../img/venus.png"); 
planetsPics[2] = loadImage("../img/earth.png"); 
planetsPics[3] = loadImage("../img/mars.png"); 
planetsPics[4] = loadImage("../img/jupiterhc.png"); 
planetsPics[5] = loadImage("../img/saturn.png"); 
planetsPics[6] = loadImage("../img/uranus.png"); 
planetsPics[7] = loadImage("../img/neptune.png"); 
planetsPics[8] = loadImage("../img/pluto.png"); 
```
To set up the board and the device: ("COM3" should be changed to the Arduino Due you are using. Check it by clicking "windows + x" for windows users, then select Device Manager, click on Ports(COM & LPT) then check the name of the device)
```sh
/* BOARD */
haply_board = new Board(this, "COM3", 0);

/* DEVICE */
haply_2DoF = new Device(degreesOfFreedom.HaplyTwoDOF, deviceID, haply_board);
hAPI_Fisica.init(this); 
hAPI_Fisica.setScale(pixelsPerCentimeter); 
world = new FWorld();   //create a new FWorld object
```
Then we set up the environment, such as the gravity, the edges friction that restrict you from going over the edge. We remove the world bottom because some of the planets are too big to be able to display the entire planet.
```sh
world.setGravity((0.0), (0.0)); //1000 cm/(s^2)
world.remove(world.bottom); 
world.setEdgesRestitution(.4);
world.setEdgesFriction(0.5);
```
Next, we will set up the virtual coupling contact rendering technique. In addition, we load the spaceship image to use it to represents the handle.
```sh
s= new HVirtualCoupling((2)); 
s.h_avatar.setFill(255,0,0); 
s.init(world, edgeTopLeftX+worldWidth/2, edgeTopLeftY+2); 
haply_avatar = loadImage("../img/sat.png"); 
haply_avatar.resize((int)(hAPI_Fisica.worldToScreen(2)), (int)(hAPI_Fisica.worldToScreen(2)));
s.h_avatar.attachImage(haply_avatar); 
```
We call the draw function to draw images on screen.
```sh
world.draw();
```
The last part of setup() is to set the timer and the frame rate.
```sh
haptic_timer = CountdownTimerService.getNewCountdownTimer(this).configure(SIMULATION_PERIOD, HOUR_IN_MILLIS).start();
  
frameRate(60); 
```
After the setup() function, we are going to code for the draw() function.
```sh
void draw(){

}
```
Inside draw(), we set the background black in colour.
```sh
background(0);
```
After that, we call draw() again.
```sh
world.draw();
```
We will create the onTickEvent() to calculate and update the physics simulation conditions. Here, we are getting the angles and position of the handle.
```sh
void onTickEvent(CountdownTimer t, long timeLeftUntilFinish){
  
    rendering_force = true;

    if (haply_board.data_available()) {
        /* GET END-EFFECTOR STATE (TASK SPACE) */
        angles.set(haply_2DoF.get_device_angles()); 
        pos_ee.set( haply_2DoF.get_device_position(angles.array()));
        pos_ee.set(pos_ee.copy().mult(100)); 
    }
}
```
After the if-statement, we calculate the gravitational force, which is F = (G * M) / r^2. G is the gravitational constant, M is the mass of the planet, and r is the distance between the planet and the spaceship. Because we need r for the calculation, we also need to get the positions of the planet and the spaceship. 
```sh
s.setToolPosition(edgeTopLeftX+worldWidth/2-(pos_ee).x+1.0, edgeTopLeftY+(pos_ee).y);  
s.updateCouplingForce();

f_ee.set(-s.getVCforceX(), s.getVCforceY());
PVector fg = new PVector(0,0);
float G = 70000;
float fg_scaler; 

PVector planetPos = new PVector(posX, posY);
PVector shipPos = new PVector(s.getToolPositionX(), s.getToolPositionY()); 
PVector planetToShip = (shipPos.copy().sub(planetPos));

fg_scaler = G*mass/(planetToShip.mag()*planetToShip.mag()); 
fg = planetToShip.normalize(); 
fg.mult(-fg_scaler); 

s.h_avatar.addForce(fg.x, fg.y); 
```
In order to allow the player to feel the force, we convert the magnitude we calculated to force produced by the device motor.
```sh
f_ee.div(10000); 
haply_2DoF.set_device_torques(f_ee.array());
torques.set(haply_2DoF.mechanisms.get_torque());
haply_2DoF.device_write_torques();

world.step(1.0f/1000.0f);

rendering_force = false;
```

We need the onFinishEvent() function as well to reset the timer.
```sh
void onFinishEvent(CountdownTimer t){
  haptic_timer.reset();
  haptic_timer = CountdownTimerService.getNewCountdownTimer(this).configure(SIMULATION_PERIOD, HOUR_IN_MILLIS).start();
}
```
Let's add two parameters for planet mass and planet.
```sh
float mass;
FCircle planet;
```
The last function we need for the program is keyPressed(). It creates a planet, set it's position and colour, and displays the corresponding planet image when key 1-9 is pressed. By setting the planet static, the position of the planet is always the same and would not change even if the spaceship hits the planet.
```sh
void keyPressed() {
  
  if(48<key && key<58){
  world.remove(planet); 
  
  float radius; 
  
   radius = planetsRadius[key-49]; 
   mass = planetsMass[key-49]; 
    posX= worldWidth/2.0; 
    posY= worldHeight+planetsRadius[key-49]/2.0-3; 
   
  planet = new FCircle(radius);
  planet.setPosition(posX, posY);
  planet.setFill(random(255),random(255),random(255));
  planet.setStatic(true); 
  
 planetsPics[key-49].resize((int)(hAPI_Fisica.worldToScreen(radius)), (int)(hAPI_Fisica.worldToScreen(radius)));
   planet.attachImage(planetsPics[key-49]); 
  
  world.add(planet);  

  }
  
}
```
### Further development

If you are interested in this program, there are more features can be added to further develop it. Some suggestions on what to do, for example: 
- You may enable the player to pick up a shuttlecock, a tennis ball and a basketball on different planets, and experience different weights. 
- You may enable the player to throw a ball on different planets and see how much force is needed to land on a certain distance.


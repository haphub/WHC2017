import processing.serial.*;

int           l = 2*50;
int           L = 2*70;
int           d = 2*20;

PShape        kite1, circle1_1, circle2_1;
PShape        kite2, circle1_2, circle2_2;

int           baseFrameRate = 1000;
int           animation_count = baseFrameRate/60;
int           haptics_count = baseFrameRate/1000;

Device        penta1;
byte          penta1ID = 5;

Device        penta2;
byte          penta2ID = 4;

Board         pentagraph_link;
DeviceType    degreesOfFreedom;

int           y_wall = 150;
int           k_wall = 80;

float         f_x = 0;
float         f_y = 0;

float         p_Wall1, p_Wall2;

int           send_key = 1;
int           receive_key = 0;



void setup(){
  size(600, 400, P2D);
  background(255);
  frameRate(baseFrameRate);
  createKite1();
  createKite2();
  
  pentagraph_link = new Board(Serial.list()[0], 0);
  
  penta1 = new Device(degreesOfFreedom.TwoDOF, penta1ID, pentagraph_link);
  penta1.set_actuator_parameters(1, 180, 13824, 2);
  penta1.set_actuator_parameters(2, 0, 13824, 1);
  penta1.set_base_mechanism(l, L, d);
  penta1.device_set_parameters();
  
  penta2 = new Device(degreesOfFreedom.TwoDOF, penta2ID, pentagraph_link);
  penta2.set_actuator_parameters(1, 180, 13824, 4);
  penta2.set_actuator_parameters(2, 0, 13824, 3);
  penta2.set_base_mechanism(l, L, d);
  penta2.device_set_parameters();
  
}

void draw(){
  
  if(pentagraph_link.data_available()){
    
    if(receive_key == 0 && send_key == 0){
      penta1.device_read_angles();
      send_key = 1;
    }
    
    if(receive_key == 1 && send_key == 1){
      penta2.device_read_angles();
      send_key = 0;
    }
    
    
    penta1.base_forwardKinematics();
    penta2.base_forwardKinematics();
    
    p_Wall1 = penta1.baseMechanism.y_E - y_wall;
    p_Wall2 = penta2.baseMechanism.y_E - y_wall;
  
    if(p_Wall1 > 0){
      f_y=-k_wall*p_Wall1;
      penta1.base_TorqueCalculations(f_x, f_y, 2*1000);
    } 
    
    if(p_Wall2 > 0){
      f_y=-k_wall*p_Wall2;
      penta2.base_TorqueCalculations(f_x, f_y, 2*1000);
    } 
  }

  if(frameCount % animation_count == 0){
    update_environment();
    update_kite1(penta1.baseMechanism.th1, penta1.baseMechanism.th2, penta1.baseMechanism.x_E, penta1.baseMechanism.y_E);
    update_kite2(penta2.baseMechanism.th1, penta2.baseMechanism.th2, penta2.baseMechanism.x_E, penta2.baseMechanism.y_E);
    
  }
  
  
  if(frameCount % haptics_count == 0){
    
    if(send_key == 0 && receive_key == 1){
      penta1.device_write_torques();
      receive_key = 0;
    }
    
    if(send_key == 1 && receive_key == 0){
      penta2.device_write_torques();
      receive_key = 1;
    }
    
    
  }
  
}


void createKite1(){
  kite1 = createShape();
  kite1.beginShape();
  kite1.fill(255);
  kite1.stroke(0);
  kite1.strokeWeight(2);
  
  kite1.vertex(width/4, 2*height/3);
  kite1.vertex(width/4, 2*height/3);
  kite1.vertex(width/4, 2*height/3);
  kite1.vertex(width/4+d, 2*height/3);
  kite1.vertex(width/4+d, 2*height/3);
  kite1.endShape(CLOSE);
  
  circle1_1 = createShape(ELLIPSE, width/4, 2*height/3, d/5, d/5);
  circle1_1.setStroke(color(0));
  
  circle2_1 = createShape(ELLIPSE, width/4+d, 2*height/3, d/5, d/5);
  circle2_1.setStroke(color(0));
}


void createKite2(){
  kite2 = createShape();
  kite2.beginShape();
  kite2.fill(255);
  kite2.stroke(0);
  kite2.strokeWeight(2);
  
  kite2.vertex(3*width/4, 2*height/3);
  kite2.vertex(3*width/4, 2*height/3);
  kite2.vertex(3*width/4, 2*height/3);
  kite2.vertex(3*width/4+d, 2*height/3);
  kite2.vertex(3*width/4+d, 2*height/3);
  kite2.endShape(CLOSE);
  
  circle1_2 = createShape(ELLIPSE, 3*width/4, 2*height/3, d/5, d/5);
  circle1_2.setStroke(color(0));
  
  circle2_2 = createShape(ELLIPSE, 3*width/4+d, 2*height/3, d/5, d/5);
  circle2_2.setStroke(color(0));
}


void update_kite1(float th1, float th2, float x_E, float y_E){
  kite1.setVertex(1,width/4+l*cos(th1), 2*height/3-l*sin(th1)); // Vertex A with th1 from encoder reading
  kite1.setVertex(3,width/4+d+l*cos(th2), 2*height/3-l*sin(th2)); // Vertex B with th2 from encoder reading
  kite1.setVertex(2,width/4+x_E, 2*height/3-y_E); // Vertex E from Fwd Kin calculations
  
}


void update_kite2(float th1, float th2, float x_E, float y_E){
  kite2.setVertex(1,3*width/4+l*cos(th1), 2*height/3-l*sin(th1)); // Vertex A with th1 from encoder reading
  kite2.setVertex(3,3*width/4+d+l*cos(th2), 2*height/3-l*sin(th2)); // Vertex B with th2 from encoder reading
  kite2.setVertex(2,3*width/4+x_E, 2*height/3-y_E); // Vertex E from Fwd Kin calculations
  
}


void update_environment(){
  background(255); // To clean up the left-overs of drawings from the previous loop!
  
  shape(kite1); // Display the kite
  shape(circle1_1);
  shape(circle2_1); 
  stroke(0);
  
  shape(kite2); // Display the kite
  shape(circle1_2);
  shape(circle2_2); 
  stroke(0);
  
  line(0, 2*height/3-y_wall, width, 2*height/3-y_wall);
}
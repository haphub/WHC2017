import static java.lang.Math.*;

public class HapticPaddle extends Mechanisms{
 
   private float     amplitude = 0.0f;
   private float     frequenchy = 0.0f; 
   
   private float     th1; 
   private float     tau1; 
                   
  
  
  public HapticPaddle(){
  }
  
  public void forwardKinematics(float[] angles){
  }
  
  public void torqueCalculation(float[] force){
  }
  
  public void forceCalculation(){
  }
  
  public void positionControl(){
  }
  
  public void inverseKinematics(){
  }
  
  
  public void set_mechanism_parameters(float[] parameters){
    amplitude = parameters[0]; 
    frequenchy = parameters[1]; 
    
  }
  
  public float[] get_coordinate(){
    float temp[] = {0, 0};
        return temp;
  }
  
  public float[] get_torque(){
         float temp[] = {0, 0};
        return temp;
  }
  
  public float[] get_angle(){
    float temp[] = {0, 0};
        return temp;
  }

}
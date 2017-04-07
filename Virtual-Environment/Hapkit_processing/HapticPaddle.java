import static java.lang.Math.*;

public class HapticPaddle extends Mechanisms{

  private float angle;
  private float torque;
  
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
    angle = parameters[0];
    torque = parameters[1];
  }
  
  public float[] get_coordinate(){
    float temp[] = {0, 0};
        return temp;
  }
  
  public float[] get_torque(){
    float temp[] = {torque};
        return temp;
  }
  
  public float[] get_angle(){
    float temp[] = {angle};
        return temp;
  }

}
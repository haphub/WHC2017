import static java.lang.Math.*;

public class OneDoF extends Mechanisms {
 
 
    /* TwoDoF Field Objects */ 
  
  private   float     l; 
  
  public float th1; 
  
  public float tau1; 
  
  public float fx; 
  
  public float gain; 
  
  
  public OneDoF(){
   
   this.l= .05; //meters
    
  }
  
  
  
    /**
  * @brief    two degree of freedom forwardKinematics calculations
  * @param   angle1: relative angle seen at encoder1/actuator2
  * @param  angle2: relative angle seen at encoder2/actuator2
  */ 
  public void set_mechanism_parameters(float l){
   
    this.l_length=l;  
  }
  
  /**
  * @brief    two degree of freedom forwardKinematics calculations
  * @param   angle1: relative angle seen at encoder1/actuator2
  * @param  angle2: relative angle seen at encoder2/actuator2
  */ 
  public void forwardKinematics(){
    
   
  }
   
  /**
  * @brief    Torque calculations in 2D space
  * @param   f_x: end effector x position
  * @param  f_y: end effector y position
  * @param   divisor: scaling divisor
  */ 
  public void torqueCalculation(){
    
  }
 
  
  
}

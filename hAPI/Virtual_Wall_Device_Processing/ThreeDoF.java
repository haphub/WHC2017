import static java.lang.Math.*;

public class ThreeDoF extends Mechanisms {
 
 
    /* TwoDoF Field Objects */ 
  
  private   float     l,L,d; 
  
  public float th1, th2, th3; 
  
  public float tau1, tau2, tau3; 
  
  public float fx, fy, fz; 
  
  public float gain; 
  
  
    /**
  * @brief    two degree of freedom forwardKinematics calculations
  * @param   angle1: relative angle seen at encoder1/actuator2
  * @param  angle2: relative angle seen at encoder2/actuator2
  */ 
  public void set_mechanism_parameters(float l, float L, float d){
   
    this.l_length=l; 
    this.L_length=L; 
    this.d_length=d; 
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

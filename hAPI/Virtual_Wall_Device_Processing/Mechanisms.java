/**
 ***********************************************************************************************
 * @file       Mechanisms.java
 * @author     
 * @version    V0.1.0
 * @date       01-March-2017
 * @brief      Mechanisms class definition, supports base mechanism designs
 ***********************************************************************************************
 * @attention
 *
 *
 ***********************************************************************************************
 */

/* imports ************************************************************************************/
import static java.lang.Math.*;

abstract public class Mechanisms{
		


  	
 /**
	* @brief    two degree of freedom forwardKinematics calculations
	* @param 	angle1: relative angle seen at encoder1/actuator2
	* @param	angle2: relative angle seen at encoder2/actuator2
	*/ 
	public abstract void forwardKinematics();
	

	
 /**
	* @brief    Torque calculations in 2D space
	* @param 	f_x: end effector x position
	* @param	f_y: end effector y position
	* @param 	divisor: scaling divisor
	*/ 
	public abstract void torqueCalculation();

// /**
//  * @brief    Torque calculations in 2D space
//  * @param   f_x: end effector x position
//  * @param  f_y: end effector y position
//  * @param   divisor: scaling divisor
//  */ 
//  public abstract void forceCalculation();
//  
//   /**
//  * @brief    Torque calculations in 2D space
//  * @param   f_x: end effector x position
//  * @param  f_y: end effector y position
//  * @param   divisor: scaling divisor
//  */ 
//  public abstract void positionControl();
//
//
//
//}
//
//
//   /**
//  * @brief    two degree of freedom forwardKinematics calculations
//  * @param   angle1: relative angle seen at encoder1/actuator2
//  * @param  angle2: relative angle seen at encoder2/actuator2
//  */ 
//  public abstract void inverseKinematics();
  

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

public abstract class Mechanisms{
		
   /**
	* @brief    forwardKinematics calculations
	*/
	public abstract void forwardKinematics(float[] angles);
	
   /**
	* @brief    Torque calculations 
	*/ 
	public abstract void torqueCalculation(float[] forces);
	
	public abstract void forceCalculation();
	
	public abstract void positionControl();
	
	public abstract void inverseKinematics();
	
	
	public abstract void set_mechanism_parameters(float[] parameters);
	
	public abstract float[] get_coordinate();
	
	public abstract float[] get_torque();
	
	public abstract float[] get_angle();
	

}

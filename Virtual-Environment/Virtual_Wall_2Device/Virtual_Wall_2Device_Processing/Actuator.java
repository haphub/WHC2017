/**
 ***********************************************************************************************
 * @file       Actuator.java
 * @author     
 * @version    V0.1.0
 * @date       01-March-2017
 * @brief      Actuator class definition
 ***********************************************************************************************
 * @attention
 *
 *
 ***********************************************************************************************
 */
 
public class Actuator{
	
	/* Actuator object fields */
	public float encoder_offset		 = 0;
	public float encoder_resolution  = 0;
	public float angle 				 = 0;
	public float torque 			 = 0;
	public int 	 actuator_position 	 = 0;
	
 /**
	* @brief    base constructor for Actuator object, only sets up initial position
	*/
	public Actuator(){
		this(0, 0, 1);
	}
	
 /**
	* @brief    base constructor for Actuator object
	* @param 	offset: Actuator encoder offset in degrees
	* @param	resolution: Actuator encoder resolution
	* @param	position: physical actuator port location on board
	*/
	public Actuator(float offset, float resolution, int position){
		this.encoder_offset = offset;
		this.encoder_resolution = resolution;
		this.actuator_position = position;
	}
}

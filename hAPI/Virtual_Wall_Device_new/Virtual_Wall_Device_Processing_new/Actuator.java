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
	private float encoder_offset	  = 0;
	private float encoder_resolution  = 0;
	private float angle 			  = 0;
	private float torque 			  = 0;
	private int   actuator_port	 	  = 0;
	
   /**
	* @brief    base constructor for Actuator object, only sets up ports
	*/
	public Actuator(){
		this(0, 0, 1);
	}
	
   /**
	* @brief    base constructor for Actuator object
	* @param 	offset: Actuator encoder offset in degrees
	* @param	resolution: Actuator encoder resolution
	* @param	port: physical actuator port location on board
	*/
	public Actuator(float offset, float resolution, int port){
		this.encoder_offset = offset;
		this.encoder_resolution = resolution;
		this.actuator_port = port;
	}
	
	public void set_offset(float offset){
		encoder_offset = offset;
	}
	
	public void set_resolution(float resolution){
		encoder_resolution = resolution;
	}
	
	public void set_port(int port){
		actuator_port = port;
	}
	
	public void set_angle(float angle){
		this.angle = angle;
	}
	
	public void set_torque(float torque){
		this.torque = torque;
	}
	
	public float get_offset(){
		return encoder_offset;
	}
	
	public float get_resolution(){
		return encoder_resolution;
	}
	
	public int get_port(){
		return actuator_port;
	}
	
	public float get_angle(){
		return angle;
	}
	
	public float get_torque(){
		return torque;
	}
	
}

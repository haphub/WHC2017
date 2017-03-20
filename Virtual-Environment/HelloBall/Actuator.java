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
	private float torque 			  = 0;
	private int   actuator_port	 	  = 0;
	
   /**
	* @brief    base constructor for Actuator object, only sets up ports
	*/
	public Actuator(){
		this(1);
	}
	
   /**
	* @brief    base constructor for Actuator object
	* @param 	offset: Actuator encoder offset in degrees
	* @param	resolution: Actuator encoder resolution
	* @param	port: physical actuator port location on board
	*/
	public Actuator(int port){
		this.actuator_port = port;
	}
	
	
	public void set_port(int port){
		actuator_port = port;
	}
	
	public void set_torque(float torque){
		this.torque = torque;
	}
	
	public int get_port(){
		return actuator_port;
	}
	
	public float get_torque(){
		return torque;
	}
	
}

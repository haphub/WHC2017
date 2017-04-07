/**
 ***********************************************************************************************
 * @file       Sensor.java
 * @author     
 * @version    V0.1.0
 * @date       01-March-2017
 * @brief      Sensor class definition
 ***********************************************************************************************
 * @attention
 *
 *
 ***********************************************************************************************
 */
 
public class Sensor{
	
	/* Sensor object fields */
	private float encoder_offset	  = 0;
	private float encoder_resolution  = 0;
	private float angle 			  = 0;
	private int   encoder_port	 	  = 0;
	
   /**
	* @brief    base constructor for Sensor object, only sets up ports
	*/
	public Sensor(){
		this(0, 0, 1);
	}
	
   /**
	* @brief    base constructor for Sensor object
	* @param 	offset: encoder offset in degrees
	* @param	resolution: encoder resolution
	* @param	port: physical encoder port location on board
	*/
	public Sensor(float offset, float resolution, int port){
		this.encoder_offset = offset;
		this.encoder_resolution = resolution;
		this.encoder_port = port;
	}
	
	public void set_offset(float offset){
		encoder_offset = offset;
	}
	
	public void set_resolution(float resolution){
		encoder_resolution = resolution;
	}
	
	public void set_port(int port){
		encoder_port = port;
	}
	
	public void set_angle(float angle){
		this.angle = angle;
	}
	
	public float get_offset(){
		return encoder_offset;
	}
	
	public float get_resolution(){
		return encoder_resolution;
	}
	
	public int get_port(){
		return encoder_port;
	}
	
	public float get_angle(){
		return angle;
	}
	

}
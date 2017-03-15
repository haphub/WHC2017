/**
 ***********************************************************************************************
 * @file       Board.java
 * @author    
 * @version    V0.1.0
 * @date       01-March-2017
 * @brief      Board class definition
 ***********************************************************************************************
 * @attention
 *
 *
 ***********************************************************************************************
 */

 /* imports ************************************************************************************/
import processing.core.PApplet;
import processing.serial.*;

public class Board{

	/* Board object fields */
	Serial 	port;
	PApplet applet;	
	
	private byte 	deviceID;
	private int 	number_of_parameters = 0;
	private byte[]  encoder_positions   = {0, 0, 0, 0};
	private byte[]  actuator_positions   = {0, 0, 0, 0};
	
	
   /**
	* @brief    base constructor for Board object
	* @param 	portName: serial port name for connection
	* @param	speed: serial data speed
	*/
	public Board(String portName, int speed){
		this.applet = new PApplet();
		port = new Serial(applet, portName, speed);
		port.clear();
	}
	
   
 /* Data transmission functions ****************************************************************/
	
   /**
	* @brief    send data over serial port
	* @param 	type: communication type
	* @param	deviceID: ID of device sending data
	* @param	positions: actuator positions of sending device
	* @param	data: main data payload
	*/
	public void send_data(byte type, byte deviceID, byte[] positions, float[] data){
		
		
		byte[] outData = new byte[2 + 4*data.length];
		byte[] segments = new byte[4];
		
		outData[0] = format_header(type, positions);
		outData[1] = deviceID;
		this.deviceID = deviceID;
		
		
		int j = 2;
		for(int i = 0; i < data.length; i++){
			segments = FloatToBytes(data[i]);
			System.arraycopy(segments, 0, outData, j, 4);
			j = j + 4;
		}
		
		this.port.write(outData);
	}
	
	
   /**
	* @brief    receive data over serial port
	* @param 	type: communication type
	* @param	deviceID: ID of device sending data
	* @param	positions: actuator positions of sending device
	* @return   data: main data payload
	*/
	public float[] receive_data(byte type, byte deviceID, byte[] positions){
		
		int size = set_buffer_length(type, positions);
		
		byte[] inData = new byte[1 + 4*size];
		byte[] segments = new byte[4];
		
		float[] data = new float[size];
		
		
		
		this.port.readBytes(inData);
		
		if(inData[0] != deviceID){
			System.err.println("Error, another device expects this data!");
		}
		
		int j = 1;
		
		for(int i = 0; i < size; i++){
			System.arraycopy(inData, j, segments, 0, 4);
			data[i] = BytesToFloat(segments);
			j = j + 4;
		}
		
		return data;
	}
	
	
 /* Serial connection status functions *********************************************************/
   
   /**
	* @brief    checks to see if data is available
	* @return   if data is available
	*/
	public boolean data_available(){
		
		boolean available = false;
		
		if(port.available() > 0){
			available = true;
		}
		
		return available;
	}
	
   
 /* Data formatting and status checking functions **********************************************/
   
   /**
	* @brief    set serial buffer length for incoming data
	* @param 	length: number of bytes expected in read buffer
	*/
	private void set_buffer(int length){
		this.port.buffer(length);
	}
	
	
   /**
	* @brief    determines how much data is incoming and sets buffer lengths accordingly
	* @param 	type: communication type
	* @param	positions: port positions of sending device
	* @return   number of active motors
	*/
	private int set_buffer_length(byte type, byte[] positions){
		
		int m_active = 0;
		
		for(int i = 0; i < 4; i++){
			if(positions[i] > 0){
				m_active++;
			}
		}
		
		switch(type){
			case 0: // setup command
				port_check(encoder_positions, positions);
				set_buffer(5);	
				m_active = 1;
				break;	
			case 1: // read encoder data
				set_buffer(1+4*m_active);
				break;
		}
		
		return m_active;
	}
	
	
   /**
	* @brief    determines if the desired ports are in use and prints warnings accordingly
	* @param	desired_positions: port positions of sending device
	*/
	private void port_check(byte[] set_positions byte[] desired_positions){
		
		for( int i = 0; i < 4; i++){
			if(set_positions[i] > 0 && desired_positions[i] > 0){
				System.err.println("Warning, hardware actuator " + i + " was in use and will be overridden");
			}
			
			set_positions[i] = desired_positions[i];
		}
	}
	
	
   /**
	* @brief    formats header byte for transmission over serial
	* @param 	type: communication type
	* @param	positions: actuator positions of sending device
	* @return   formatted header byte
	*/ 
	private byte format_header(byte type, byte[] positions){
		
		int header = 0;
    
		for(int i = 0; i < positions.length; i++){
	    
			header = header >> 1;		
  
			if(positions[i] > 0){
				header = header | 0x0008;
			}
		}

		header = header | (type << 4);
		
		return (byte)header;
	}
	
	
   /**
	* @brief    translates a float point number to its raw binary across four bytes
	* @param 	val: floating point numberf

		return segments;
  
	}

   /**
	* @brief    translates a binary of a float point to actual float point
	* @param 	segment: array containing raw binary of floating point
	* @return   translated floating point number
	*/ 	
	private float BytesToFloat(byte[] segment){
  
		int temp = 0;
  
		temp = (temp | (segment[3] & 0xff)) << 8;
		temp = (temp | (segment[2] & 0xff)) << 8;
		temp = (temp | (segment[1] & 0xff)) << 8;
		temp = (temp | (segment[0] & 0xff)); 
  
		float val = Float.intBitsToFloat(temp);
  
		return val;
	}	
	
}
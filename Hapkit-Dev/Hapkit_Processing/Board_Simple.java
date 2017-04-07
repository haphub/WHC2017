import processing.core.PApplet;
import processing.serial.*;


public class Board_Simple{

  Serial port;
  PApplet applet;
  
  private byte device_function;
  private int receive_amount = 2;
  private boolean functionMatch = false;
  
  public Board_Simple(String portName, int speed){
    this.applet = new PApplet();
    port = new Serial(applet, portName, speed);
    port.buffer(9);
    port.clear();
  }
  

 /* Communication functions ********************************************************************/  
 /**
  * @brief    Send data to Hapkit
  * @param    device_function: type of function to carry out
  * @param    data: data to be transmitted
  * @return   none
  */ 
  public void send_data(byte device_function, float[] data){
    
    byte[] outData = new byte[2+ 4*data.length];
    byte[] segments = new byte[4];
    
    this.device_function = device_function;
    
    outData[0] = 0;
    outData[1] = device_function;
    
    int j = 2;
    for(int i = 0; i < data.length; i++){
      segments = FloatToBytes(data[i]);
      System.arraycopy(segments, 0, outData, j, 4);
      j = j + 4;
    }
    
    port.write(outData);
  }
  
  
 /**
  * @brief    Send one byte to Hapkit requesting data
  * @return   none
  */ 
  public void send_data_request(){
    byte[] request = new byte[1];
    request[0] = 1;
    port.write(request);
  }
  
  
 /**
  * @brief    Receive data from Hapkit 
  * @param    device_function: type of function to carry out
  * @return   simulation parameters
  */ 
  public float[] receive_data(byte device_function){
    
    byte[] inData = new byte[1 + 4*receive_amount];
    byte[] segments = new byte[4];
    float[] data = new float[receive_amount];
    
    port.readBytes(inData);
    
    if(inData[0] != device_function){
      functionMatch = false;
    }
    else{
      functionMatch = true;
    }
    
    int j = 1;
    for(int i = 0; i < receive_amount; i++){
      System.arraycopy(inData, j, segments, 0, 4);
      data[i] = BytesToFloat(segments);
      j = j + 4;
    }
    
    return data;
  }
  
  
 /* Data formatting and status checking functions **********************************************/
 /**
  * @brief    Determine if returned funcion is expected function 
  * @return   function match boolean
  */ 
  public boolean get_functionMatch(){
    return functionMatch;
  }
  
  
 /**
  * @brief    Determine if there is data to be read from hapkit 
  * @return   data available boolean
  */ 
  public boolean data_available(){
    boolean available = false;
    
    if(port.available() > 0){
      available = true;
    }
    
    return available;
  }
 
 
  
 /**
  * @brief    translates a float point number to its raw binary across four bytes
  * @param   val: floating point number
  * @return   array of 4 bytes containing raw binary of floating point number
  */ 
  private byte[] FloatToBytes(float val){
  
    byte[] segments = new byte[4];
  
    int temp = Float.floatToRawIntBits(val);
  
    segments[3] = (byte)((temp >> 24) & 0xff);
    segments[2] = (byte)((temp >> 16) & 0xff);
    segments[1] = (byte)((temp >> 8) & 0xff);
    segments[0] = (byte)((temp) & 0xff);

    return segments;
  }

 /**
  * @brief    translates a binary of a float point to actual float point
  * @param   segment: array containing raw binary of floating point
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
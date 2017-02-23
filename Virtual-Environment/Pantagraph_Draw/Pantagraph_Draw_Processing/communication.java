import processing.core.PApplet;
import processing.serial.*;

public class communication{
	
	// fields
	Serial port;
	PApplet applet;	
	
	private int number_of_parameters = 0;
	private byte[] actuator_positions = {0, 0, 0, 0};
	
	
	// constructors
	public communication(String portName){
		this.applet = new PApplet();
		port = new Serial(applet, portName, 0);
		port.clear();
	}
	
	
	// methods
	// set buffer length for recieve data
	public void set_buffer(int length){
		this.port.buffer(length);
	}
	
	// 
	public boolean data_available(){
		
		boolean available = false;
		
		if(port.available() > 0){
			available = true;
		}
		
		return available;
	}
	
	public void SetParameters(byte[] positions, float[] data){
		int actuators = port_check(positions);
		byte[] device_set = new byte[5];
		byte[] actuator_parameters = new byte[4*actuators*2+1]; // 2 is because only sending offset and resolution params
		byte[] segments = new byte[4];
		
		int j = 1;
		
		// first send actuator position setup
		device_set[0] = 0;
		System.arraycopy(positions, 0, device_set, 1, 4);
		this.port.write(device_set);
		
		// do byte conversion and send data
		actuator_parameters[0] = 1;
		for(int i = 0; i < data.length; i++){
			segments = FloatToBytes(data[i]);
			System.arraycopy(segments, 0, actuator_parameters, j, 4);
			j = j + 4;
		}
		this.port.write(actuator_parameters);	
	}
	
	public void ReadRequest(byte[] positions){
		int actuators = port_count(positions);
		byte[] device_set = new byte[5];
		
		// first send actuator position setup
		device_set[0] = 0;
		System.arraycopy(positions, 0, device_set, 1, 4);
		this.port.write(device_set);
		
		// prepare buffer to receive data
		set_buffer(4*actuators);
		byte[] request = {2};
		this.port.write(request);
	}
	
	public float[] ReadAngles(byte[] positions){
		int actuators = port_count(positions); 
		float[] angles = new float[actuators];
		byte[] data = new byte[4*actuators];
		byte[] segments = new byte[4];
		
		this.port.readBytes(data);
		
		int j = 0;
		for(int i = 0; i < actuators; i++){
			System.arraycopy(data, j, segments, 0, 4);
			angles[i] = BytesToFloat(segments);
			j = j + 4;
		}
		
		return angles;
	}
	
/* 	// sets up (instruction, data parameters, data)
	public void send_data(PacketType instruction, byte[] positions, float[] data){
		
		int actuators = 
		
		switch(instruction){
			case SetParameters:
				break;
			case CommandTorques:
				break;
			default:
				System.err.println("Incorrect send data packet type");
				break;
		}
	}
	
	// read request, read data
	public void read_data(){
	}
	 */
	
	
	private int port_check(byte[] positions){
		int count = 0;
		
		for( int i = 0; i < 4; i++){
			if(actuator_positions[i] > 0 && positions[i] > 0){
				System.err.println("Warning, hardware actuator " + i + " was in use and will be overridden");
			}
			
			if(positions[i] > 0){
				count++;
			}
			
			actuator_positions[i] = positions[i];
		}
		
		return count;
	}

  private int port_count(byte[] positions){
    int count = 0;
    
    for(int i = 0; i < 4; i++){
      if(positions[i] > 0){
        count++;
      }
    }
    
    return count;
  }
	
	// conversion functions 
	private byte[] FloatToBytes(float val){
  
		byte[] segments = new byte[4];
  
			int temp = Float.floatToRawIntBits(val);
  
			segments[3] = (byte)((temp >> 24) & 0xff);
			segments[2] = (byte)((temp >> 16) & 0xff);
			segments[1] = (byte)((temp >> 8) & 0xff);
			segments[0] = (byte)((temp) & 0xff);

		return segments;
	}
	
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
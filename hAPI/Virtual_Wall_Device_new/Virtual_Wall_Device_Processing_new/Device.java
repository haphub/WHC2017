/**
 ***********************************************************************************************
 * @file       Device.java
 * @author    
 * @version    V0.1.0
 * @date       01-March-2017
 * @brief      Device class definition
 ***********************************************************************************************
 * @attention
 *
 *
 ***********************************************************************************************
 */
 
public class Device{

	/* Communication object fields */
	public 	DeviceType 		DOF;
	public 	byte 			deviceID;
	public 	Board 	      	deviceLink;
	
	public  Actuator[]    	motors;	
	public Sensor[]         encoders; 	
	public 	Mechanisms 		mechanisms;
	
	public  byte[] 			actuator_positions = {0, 0, 0, 0};
	public byte[]			encoder_positions = {0, 0, 0, 0}; 
	private byte 			communicationType;
 	private float[]   		params;
	

   /**
	* @brief    base constructor for Communication object
	* @param 	DOF: device degrees of freedom
	* @param	deviceID: device ID
	* @param	deviceLink: serial link used by device
	*/
	public Device(DeviceType DOF, byte[] ports, byte deviceID, Board deviceLink){
		this.DOF = DOF;
		this.deviceID = deviceID;
		this.deviceLink = deviceLink;
		
		switch(DOF){
  
			case OneDOF:
				motors = new Actuator[1];
				encoder = new Sensor[1]; 
				
				if(ports.length()>(this.DOF+1)){
					 System.err.println("Error: The number of ports used by this device does not match the type of device. ")
				}
				else{
				for(int i=0; i< motors.length(); i++){
					
					this.motors[i] = new Actuator(); 
					this.encoders[i] = new Sensor(); 
					this.set_actuator_parameters(i, this.port[i]);
					this.set_encoder_parameters(i, 180*i, 13824, this.port[i]);
					
				}
				}
					
				this.device_set_parameters();
				mechanisms = new HaplyOneDoFMech();
				params = new float[1];
				break;
			
			case TwoDOF:
				motors = new Actuator[2];
				encoder= new Sensor[2]; 
				
				if(ports.length()>(this.DOF+1)){
					 System.err.println("Error: The number of ports used by this device does not match the type of device. ")
				}
				else{
				for(int i=0; i< motors.length(); i++){
					
					this.motors[i] = new Actuator(); 
					this.encoders[i] = new Sensor();
					this.set_actuator_parameters(i, this.port[i]);
					this.set_encoder_parameters(i, 180*i, 13824, this.port[i]);
				}
				}
					
				
				this.device_set_parameters();
				mechanisms = new HaplyTwoDoFMech();
        		params = new float[2];
				break;
				
			case ThreeDOF:
				motors = new Actuator[3];
				encoder= new Sensor[3]; 
				
				if(ports.length()>(this.DOF+1)){
					 System.err.println("Error: The number of ports used by this device does not match the type of device. ")
				}
				else{
				for(int i=0; i< motors.length(); i++){
					
					this.motors[i] = new Actuator(); 
					this.encoders[i] = new Sensor();
					this.set_actuator_parameters(i, this.port[i]);
					this.set_encoder_parameters(i, 180*i, 13824, this.port[i]);
				}
				}
				
				this.device_set_parameters();
				mechanisms = new HaplyThreeDoFMech();
				params = new float[3];
				break;
				
			case FourDOF:
				motors = new Actuator[4];
				encoder= new Sensor[4]; 
				
				if(ports.length()>(this.DOF+1)){
					 System.err.println("Error: The number of ports used by this device does not match the type of device. ")
				}
				else{
				for(int i=0; i< motors.length(); i++){
					
					this.motors[i] = new Actuator(); 
					this.encoders[i] = new Sensor();
					this.set_actuator_parameters(i, this.port[i]);
					this.set_encoder_parameters(i, 180*i, 13824, this.port[i]);
				}
				}
				
				this.device_set_parameters()
				mechanisms = new HaplyFourDoFMech();
				params = new float[4];
				break;
				
			default:
				System.err.println("Error: Undefined device type!");
				break;
		}

	
	}
	
   
 /* Device setup functions *********************************************************************/
   
   /**
	* @brief    set encoder parameters used by device
	* @param 	encoder: sequential listing of encoder to be setup
	* @param	offset: degrees offset associated with encoder
	* @param 	resolution: encoder encoder resolution
	* @param	position: encoder port position
	*/
	public void set_encoder_parameters(int sensor, float offset, float resolution, int port){
		
		if(port <=0 || port > 4){
			System.err.println("error: encoder position index out of bounds!");
		}
		else{
			switch(encoder){
  
				case 1:
					encoder[0].set_offset(offset);
					encoder[0].set_resolution(resolution);
					encoder[0].set_port(port);
					encoder_assignment(sensor, encoder[0]);
					break;
				case 2:
					encoder[1].set_offset(offset);
					encoder[1].set_resolution(resolution);
					encoder[1].set_port(port);
					encoder_assignment(sensor, encoder[1]);
					break;
				case 3:
					encoder[2].set_offset(offset);
					encoder[2].set_resolution(resolution);
					encoder[2].set_port(port);
					encoder_assignment(sensor, encoder[2]);
					break;
				case 4:
					encoder[3].set_offset(offset);
					encoder[3].set_resolution(resolution);
					encoder[3].set_port(port);
					encoder_assignment(sensor, encoder[3]);
					break;
				default:
					System.err.println("error: encoder index out of bound! refer to limit of constructed device");
					break;
			}
		}
	}
	
	   /**
	* @brief    set actuator parameters used by device
	* @param 	actuator: sequential listing of actuator to be setup
	* @param	position: actuator port position
	*/
	public void set_actuator_parameters(int actuator, int port){
		
		if(port <=0 || port > 4){
			System.err.println("error: actuator position index out of bounds!");
		}
		else{
			switch(actuator){
  
				case 1:
					motors[0].set_port(port);
					encoder_assignment(actuator, motors[0]);
					break;
				case 2:
					motors[1].set_port(port);
					encoder_assignment(actuator, motors[1]);
					break;
				case 3:			
					motors[2].set_port(port);
					encoder_assignment(actuator, motors[2]);
					break;
				case 4:
					motors[3].set_port(port);
					encoder_assignment(actuator, motors[3]);
					break;
				default:
					System.err.println("error: encoder index out of bound! refer to limit of constructed device");
					break;
			}
		}
	}
	
	public void set_new_mechanism(Mechanisms mechanisms){
		this.mechanisms = mechanisms;
	}
	
	
   /**
	* @brief    base mechanism forward kinematics calculations
	* @note		function incomplete, only has equations for 2DOF device !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
	*/
 	public void base_forwardKinematics(){
		
		switch(DOF){
      		case OneDOF:
        		break;
        
			case TwoDOF:
				params[0] = encoders[0].get_angle(); 
        		params[1] = encoders[1].get_angle();
				mechanisms.forwardKinematics(params);
				break;

      		case ThreeDOF:
        		break;
        
      		case FourDOF:
        		break;
		}
	}
	 
	 
	
   /**
	* @brief    base mechanism torque calculations
	* @note		function incomplete, only has equations for 2DOF device !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
	* @param	fx: end effector x location
	* @param	fy: end effector y location
	* @param	divisor: divisor constant to keep force calculations in check 
	*/
 	public void base_TorqueCalculations(float fx, float fy){
		
		//float[] torques;
		
		switch(DOF){
      		case OneDOF:
        		break;
        
			case TwoDOF:
				params[0] = fx;
        		params[1] = fy;
				mechanisms.torqueCalculation(params);

				params = mechanisms.get_torque();
				motors[0].set_torque(params[0]);
				motors[1].set_torque(params[1]);
				break;
      
      		case ThreeDOF:
        		break;
      
      		case FourDOF:
        		break;
		}
	}  
	
	
	
 /* Device communication and control functions *************************************************/
	
   /**
	* @brief    sends setup information over serial
	*/	
	public void device_set_parameters(){
  
		communicationType = 0;
		float[] parameter_data = new float[2*encoders.length];
    
    
		int j = 0;
		for(int i = 0; i < encoder_positions.length; i++){
      
			if(encoder_positions[i] > 0){
				parameter_data[2*j] = encoders[encoder_positions[i]-1].get_offset();
				parameter_data[2*j+1] = encoders[encoder_positions[i]-1].get_resolution();
				j++;
			}
		}

    
		deviceLink.send_data(communicationType, deviceID, encoder_positions, parameter_data);	
	}
	
	
   /**
	* @brief    response over serial for verification
	*/
	public void device_set_verification(){
		
		communicationType = 0;
    
		if(deviceLink.data_available()){
			float[] recieve = deviceLink.receive_data(communicationType, deviceID, encoder_positions);
		}
	}
	
	
   /**
	* @brief    request for encoder data
	*/
	public void device_read_request(){
    communicationType = 1;
		
		float[] encoder_request = new float[encoders.length];

    	int j = 0;
    	for(int i = 0; i < encoder_positions.length; i++){
      
      		if(encoder_positions[i] > 0){
        		encoder_request[j] = 0;
        		j++;
      		}
    	}
		
    	deviceLink.send_data(communicationType, deviceID, encoder_positions, encoder_request);
	}
	
	
   /**
	* @brief    write encoder torques to device
	*/
	public void device_write_torques(){
		
    communicationType = 1;
    
		float[] device_torques = new float[motors.length];
		
    	int j = 0;
    	for(int i = 0; i < actuator_positions.length; i++){
      		if(actuator_positions[i] > 0){
        		device_torques[j] = motors[actuator_positions[i]-1].get_torque();
        		j++;
      		}
    	}

    	deviceLink.send_data(communicationType, deviceID, actuator_positions, device_torques);		
	}
	
	
   /**
	* @brief    receive encoder angles from device
	*/
	public void device_read_angles(){
		
    communicationType = 1;
    
		float[] angle_data = deviceLink.receive_data(communicationType, deviceID, encoder_positions);
		
    	int j = 0;
    	for(int i = 0; i < encoder_positions.length; i++){
      		if(encoder_positions[i] > 0){
        		encoders[encoder_positions[i]-1].set_angle(angle_data[j]);
        		j++;
      		}
    	}
	}
	
	
 /* Device object helper functions *************************************************************/
   
   /**
	* @brief    assigns actuator positions based on actuator port
	*/
	private void actuator_assignment(int actuator, Actuator m){
		
		switch(m.get_port()){
			case 1:
				this.actuator_positions[0] = (byte)actuator;
				break; 
			case 2:
				this.actuator_positions[1] = (byte)actuator;
				break;
			case 3:
				this.actuator_positions[2] = (byte)actuator;
				break;
			case 4:
				this.actuator_positions[3] = (byte)actuator;
				break;
			default:
				System.err.println("Error, actuator position out of bound");
				break;
		}
		
	}
	
	 /* Device object helper functions *************************************************************/
   
   /**
	* @brief    assigns encdoer positions based on encoder port
	*/
	private void sensor_assignment(int encoder, Sensor m){
		
		switch(m.get_port()){
			case 1:
				this.encoder_positions[0] = (byte)encoder;
				break; 
			case 2:
				this.encoder_positions[1] = (byte)encoder;
				break;
			case 3:
				this.encoder_positions[2] = (byte)encoder;
				break;
			case 4:
				this.encoder_positions[3] = (byte)encoder;
				break;
			default:
				System.err.println("Error, encoder position out of bound");
				break;
		}
		
	}
	
}
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
	
	public  Actuator[]    	motors;	
	public  Sensor[]		encoders;

	public 	Board 	      	deviceLink;
	public 	Mechanisms 		mechanisms;
	
	public  byte[] 			actuator_positions = {0, 0, 0, 0};
	public 	byte[]			encoder_positions = {0, 0, 0, 0};
	private byte 			communicationType;
 	private float[]   		params;
	

   /**
	* @brief    base constructor for Communication object
	* @param 	DOF: device degrees of freedom
	* @param	deviceID: device ID
	* @param	deviceLink: serial link used by device
	*/
	public Device(DeviceType DOF, byte deviceID, Board deviceLink){
		this.DOF = DOF;
		this.deviceID = deviceID;
		this.deviceLink = deviceLink;
		
		switch(DOF){
  
			case OneDOF:
				motors = new Actuator[1];
				encoders = new Sensor[1];
				mechanisms = new HaplyOneDoFMech();
        device_component_auto_setup();
				params = new float[1];
				break;
			case TwoDOF:
				motors = new Actuator[2];
				encoders = new Sensor[2];
         mechanisms = new HaplyTwoDoFMech(); 
         device_component_auto_setup(); 
        		params = new float[2];
				break;
			case ThreeDOF:
				motors = new Actuator[3];
				encoders = new Sensor[3];
				mechanisms = new HaplyThreeDoFMech();
        device_component_auto_setup(); 
				params = new float[3];
				break;
			case FourDOF:
				motors = new Actuator[4];
				encoders = new Sensor[4];
				mechanisms = new HaplyFourDoFMech();
        device_component_auto_setup(); 
				params = new float[4];
				break;
			default:
				System.err.println("Error: Undefined device type!");
				break;
		}


	}
	
   
   private void device_component_auto_setup(){
     
      for(int i = 0; i < motors.length; i++){
        motors[i] = new Actuator();
      this.set_actuator_parameters(i+1, i+1);
         }
         
      for(int i = 0; i < encoders.length; i++){
        encoders[i] = new Sensor();
    
       this.set_encoder_parameters(i+1, (i+1)*180, 13824, i+1); 
    }
   }
   
   
 /* Device setup functions *********************************************************************/
   
   /**
	* @brief    set actuator parameters used by device
	* @param 	actuator: sequential listing of actuator to be setup
	* @param	offset: degrees offset associated with actuator
	* @param 	resolution: actuator encoder resolution
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
					actuator_assignment(actuator, motors[0]);
					break;
				case 2:
					motors[1].set_port(port);
					actuator_assignment(actuator, motors[1]);
					break;
				case 3:
					motors[2].set_port(port);
					actuator_assignment(actuator, motors[2]);
					break;
				case 4:
					motors[3].set_port(port);
					actuator_assignment(actuator, motors[3]);
					break;
				default:
					System.err.println("error: actuator index out of bound! refer to limit of constructed device");
					break;
			}
		}
	}
	
	/**
	* @brief    set encoder parameters used by device
	* @param 	actuator: sequential listing of actuator to be setup
	* @param	offset: degrees offset associated with actuator
	* @param 	resolution: actuator encoder resolution
	* @param	position: actuator port position
	*/
	public void set_encoder_parameters(int sensor, float offset, float resolution, int port){
		
		if(port <=0 || port > 4){
			System.err.println("error: encoder position index out of bounds!");
		}
		else{
			switch(sensor){
  
				case 1:
					encoders[0].set_offset(offset);
					encoders[0].set_resolution(resolution);
					encoders[0].set_port(port);
					encoder_assignment(sensor, encoders[0]);
					break;
				case 2:
					encoders[1].set_offset(offset);
					encoders[1].set_resolution(resolution);
					encoders[1].set_port(port);
					encoder_assignment(sensor, encoders[1]);
					break;
				case 3:
					encoders[2].set_offset(offset);
					encoders[2].set_resolution(resolution);
					encoders[2].set_port(port);
					encoder_assignment(sensor, encoders[2]);
					break;
				case 4:
					encoders[3].set_offset(offset);
					encoders[3].set_resolution(resolution);
					encoders[3].set_port(port);
					encoder_assignment(sensor, encoders[3]);
					break;
				default:
					System.err.println("error: actuator index out of bound! refer to limit of constructed device");
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
      
			if(actuator_positions[i] > 0){
				parameter_data[2*j] = encoders[actuator_positions[i]-1].get_offset();
				parameter_data[2*j+1] = encoders[actuator_positions[i]-1].get_resolution();
				j++;
			}
		}

    
		deviceLink.send_data(communicationType, deviceID, actuator_positions, parameter_data);	
	}
	
	
   /**
	* @brief    response over serial for verification
	*/
	public void device_set_verification(){
		
		communicationType = 0;
    
		if(deviceLink.data_available()){
			float[] recieve = deviceLink.receive_data(communicationType, deviceID, actuator_positions);
		}
	}
	
	
   /**
	* @brief    request for encoder data
	*/
	public void device_read_request(){
    communicationType = 1;
		
		float[] encoder_request = new float[motors.length];

    	int j = 0;
    	for(int i = 0; i < encoder_positions.length; i++){
      
      		if(actuator_positions[i] > 0){
        		encoder_request[j] = 0;
        		j++;
      		}
    	}
		
    	deviceLink.send_data(communicationType, deviceID, actuator_positions, encoder_request);
	}
	
	
   /**
	* @brief    write actuator torques to device
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
        		encoders[actuator_positions[i]-1].set_angle(angle_data[j]);
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
	
	
   /**
	* @brief    assigns actuator positions based on actuator port
	*/
	private void encoder_assignment(int encoder, Sensor m){
		
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
				System.err.println("Error, actuator position out of bound");
				break;
		}
		
	}
}

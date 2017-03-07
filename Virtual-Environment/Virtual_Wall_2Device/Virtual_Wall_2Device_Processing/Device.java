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
	public 	  DeviceType 		DOF;
	public 	  byte 			    deviceID;
	
        public    Actuator[]    motors;	

	public 	  Board 	      deviceLink;
	public 	  Mechanisms 		baseMechanism;
	
	public    byte[] 	            actuator_positions = {0, 0, 0, 0};
	private   byte 			    communicationType;
	

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
				break;
			case TwoDOF:
        motors = new Actuator[2];
				break;
			case ThreeDOF:
        motors = new Actuator[3];
				break;
			case FourDOF:
        motors = new Actuator[4];
				break;
			default:
				System.err.println("Error: Undefined device type!");
				break;
		}

    for(int i = 0; i < motors.length; i++){
      motors[i] = new Actuator();
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
	public void set_actuator_parameters(int actuator, float offset, float resolution, int position){
		
		if(position <=0 || position > 4){
			System.err.println("error: actuator position index out of bounds!");
		}
		else{
			switch(actuator){
  
				case 1:
					motors[0].encoder_offset = offset;
					motors[0].encoder_resolution = resolution;
					motors[0].actuator_position = position;
					actuator_assignment(actuator, motors[0]);
					break;
				case 2:
					motors[1].encoder_offset = offset;
					motors[1].encoder_resolution = resolution;
					motors[1].actuator_position = position;
					actuator_assignment(actuator, motors[1]);
					break;
				case 3:
					motors[2].encoder_offset = offset;
					motors[2].encoder_resolution = resolution;
					motors[2].actuator_position = position;
					actuator_assignment(actuator, motors[2]);
					break;
				case 4:
					motors[3].encoder_offset = offset;
					motors[3].encoder_resolution = resolution;
					motors[3].actuator_position = position;
					actuator_assignment(actuator, motors[3]);
					break;
				default:
					System.err.println("error: actuator index out of bound! refer to limit of constructed device");
					break;
			}
		}
	}
	
	
 /**
	* @brief    update actuator torque value
	* @param 	actuator: sequential listing of actuator to be setup
	* @param	torque: torque value for update
	*/
	public void update_actuator_torque(int actuator, float torque){
    
		switch(actuator){
			case 1:
				motors[0].torque = torque;
				break;
			case 2:
				motors[1].torque = torque;
				break;
			case 3:
				motors[2].torque = torque;
				break;
			case 4:
				motors[3].torque = torque;
				break;
			default:
				System.err.println("error: actuator index out of bound!");
				break;
		}
	}
	
	
 /**
	* @brief    setup base mechanism for 1DOF device
	* @param 	l: arm length
	*/
	public void set_base_mechanism(int l){
		baseMechanism = new Mechanisms(l);
	}
	
	
 /**
	* @brief    setup base mechanism for 2DOF device
	* @param 	l: arm1 length
	* @param	L: arm2 length
	* @param	d: motor distance
	*/
	public void set_base_mechanism(int l, int L, int d){
		baseMechanism = new Mechanisms(l, L, d);
	}
	
	
 /**
	* @brief    base mechanism forward kinematics calculations
	* @note		function incomplete, only has equations for 2DOF device !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
	*/
	public void base_forwardKinematics(){
		switch(DOF){
			case TwoDOF:
				baseMechanism.forwardKinematics(motors[0].angle, motors[1].angle);
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
	public void base_TorqueCalculations(float fx, float fy, int divisor){
		switch(DOF){
			case TwoDOF:
				baseMechanism.torqueCalculation(fx, fy, divisor);
				motors[0].torque = baseMechanism.tau1;
				motors[1].torque = baseMechanism.tau2;
				break;
		}
	}
	
	
	
 /* Device communication and control functions *************************************************/
	
 /**
	* @brief    sends setup information over serial
	*/	
	public void device_set_parameters(){
  
		communicationType = 0;
		float[] parameter_data = new float[2*motors.length];
    
    
		int j = 0;
    for(int i = 0; i < actuator_positions.length; i++){
      
      if(actuator_positions[i] > 0){
        parameter_data[2*j] = motors[actuator_positions[i]-1].encoder_offset;
        parameter_data[2*j+1] = motors[actuator_positions[i]-1].encoder_resolution;
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
    
		while(deviceLink.data_available()){
			float[] recieve = deviceLink.receive_data(communicationType, deviceID, actuator_positions);
      System.out.println("Device " + deviceID + "is good!");
		}
	}
	
	
 /**
	* @brief    request for encoder data
	*/
	public void device_read_request(){
    communicationType = 1;
		
		float[] encoder_request = new float[motors.length];

    int j = 0;
    for(int i = 0; i < actuator_positions.length; i++){
      
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
        device_torques[j] = motors[actuator_positions[i]-1].torque;
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
    
		float[] angle_data = deviceLink.receive_data(communicationType, deviceID, actuator_positions);
		
    int j = 0;
    for(int i = 0; i < actuator_positions.length; i++){
      if(actuator_positions[i] > 0){
        motors[actuator_positions[i]-1].angle = angle_data[j];
        j++;
      }
    }
	}
	
	
 /* Device object helper functions *************************************************************/
   
 /**
	* @brief    assigns actuator positions based on actuator port
	*/
	private void actuator_assignment(int actuator, Actuator m){
		
		switch(m.actuator_position){
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
}

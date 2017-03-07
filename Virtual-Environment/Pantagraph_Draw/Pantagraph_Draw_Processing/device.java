
public class device{
		
	// fields
	DeviceType DOF;
	
	public actuator motor1;
	public actuator motor2;
	public actuator motor3;
	public actuator motor4;
	
	public communication deviceLink;
	
	// sets actuator position 
	private byte[] actuator_positions = {0, 0, 0, 0};
	
	// constructor
	public device(DeviceType DOF, communication deviceLink){
		this.DOF = DOF;
		this.deviceLink = deviceLink;
		
		switch(DOF){
			case OneDOF:
				motor1 = new actuator();
				break;
				
			case TwoDOF:
				motor1 = new actuator();
				motor2 = new actuator();
				break;
				
			case ThreeDOF:
				motor1 = new actuator();
				motor2 = new actuator();
				motor3 = new actuator();
				break;
				
			case FourDOF:
				motor1 = new actuator();
				motor2 = new actuator();
				motor3 = new actuator();
				motor4 = new actuator();
				break;
				
			default:
				System.err.println("error: Undefined device type!");
				break;
		}
	}
	
	// methods
	
	// set actuator parameters, constructor for actuator objects in device
	public void set_actuator_parameters(int actuator, float offset, float resolution, int position){
		
		if(position <=0 || position > 4){
			System.err.println("error: actuator position index out of bounds!");
		}
		else{
			switch(actuator){
				case 1:
					motor1.encoder_offset = offset;
					motor1.encoder_resolution = resolution;
					motor1.actuator_position = position;
					actuator_assignment(motor1);
					break;
				case 2:
					motor2.encoder_offset = offset;
					motor2.encoder_resolution = resolution;
					motor2.actuator_position = position;
					actuator_assignment(motor2);
					break;
				case 3:
					motor3.encoder_offset = offset;
					motor3.encoder_resolution = resolution;
					motor3.actuator_position = position;
					actuator_assignment(motor3);
					break;
				case 4:
					motor4.encoder_offset = offset;
					motor4.encoder_resolution = resolution;
					motor4.actuator_position = position;
					actuator_assignment(motor4);
					break;
				default:
					System.err.println("error: actuator index out of bound! refer to limit of constructed device");
					break;
			}
		}
	}
	
	
	// send setup
	public void device_set_parameters(){
		float[] parameter_data;
		
		switch(DOF){
			case OneDOF:
				parameter_data = new float[2];
				parameter_data[0] = motor1.encoder_offset;
				parameter_data[1] = motor1.encoder_resolution;
				deviceLink.SetParameters(actuator_positions, parameter_data);
				break;
				
			case TwoDOF:
				parameter_data = new float[4];
				parameter_data[0] = motor1.encoder_offset;
				parameter_data[1] = motor1.encoder_resolution;
				parameter_data[2] = motor2.encoder_offset;
				parameter_data[3] = motor2.encoder_resolution;
				deviceLink.SetParameters(actuator_positions, parameter_data);
				break;
				
			case ThreeDOF:
				parameter_data = new float[6];
				parameter_data[0] = motor1.encoder_offset;
				parameter_data[1] = motor1.encoder_resolution;
				parameter_data[2] = motor2.encoder_offset;
				parameter_data[3] = motor2.encoder_resolution;
				parameter_data[4] = motor3.encoder_offset;
				parameter_data[5] = motor3.encoder_resolution;
				deviceLink.SetParameters(actuator_positions, parameter_data);
				break;
				
			case FourDOF:
				parameter_data = new float[8];
				parameter_data[0] = motor1.encoder_offset;
				parameter_data[1] = motor1.encoder_resolution;
				parameter_data[2] = motor2.encoder_offset;
				parameter_data[3] = motor2.encoder_resolution;
				parameter_data[4] = motor3.encoder_offset;
				parameter_data[5] = motor3.encoder_resolution;
				parameter_data[6] = motor4.encoder_offset;
				parameter_data[7] = motor4.encoder_resolution;
				deviceLink.SetParameters(actuator_positions, parameter_data);
				break;
		}
	}
	
	public void device_read_request(){
		deviceLink.ReadRequest(actuator_positions);	
	}
	
	// read angles
	public void device_read_angles(){
		
		float[] angle_data = deviceLink.ReadAngles(actuator_positions);
		
		switch(DOF){
			case OneDOF:
				motor1.angle = angle_data[0];
				break;
				
			case TwoDOF:
				motor1.angle = angle_data[0];
				motor2.angle = angle_data[1];
				break;
				
			case ThreeDOF:
				motor1.angle = angle_data[0];
				motor2.angle = angle_data[1];
				motor3.angle = angle_data[2];
				break;
				
			case FourDOF:
				motor1.angle = angle_data[0];
				motor2.angle = angle_data[1];
				motor3.angle = angle_data[2];
				motor4.angle = angle_data[3];
				break;
		}
	}
	
	// write torques

	
	// sets actuator assignments 
	private void actuator_assignment(actuator m){
		
		switch(m.actuator_position){
			case 1:
				this.motor1 = m;
				this.actuator_positions[0] = 1;
				break; 
			case 2:
				this.motor2 = m;
				this.actuator_positions[1] = 1;
				break;
			case 3:
				this.motor3 = m;
				this.actuator_positions[2] = 1;
				break;
			case 4:
				this.motor4 = m;
				this.actuator_positions[3] = 1;
				break;
			default:
				break;
		}
		
	}

}
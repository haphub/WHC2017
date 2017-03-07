
public class actuator{
	
	// fields
	public float encoder_offset = 0;
	public float encoder_resolution = 0;
	public float angle = 0;
	public float torque = 0;
	
	public int actuator_position = 0;
	
	// constructors
	public actuator(){
		this(0, 0, 1);
	}
	
	public actuator(float offset, float resolution, int position){
		this.encoder_offset = offset;
		this.encoder_resolution = resolution;
		this.actuator_position = position;
	}
	

}
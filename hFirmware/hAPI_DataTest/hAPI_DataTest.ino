#include "hAPI_Data.h"
const int numberOfMotors = 3; 

void setup(){
	SerialUSB.begin(9600);

	SerialUSB.println("begin");

	Data outgoingdata(numberOfMotors);
	


	/*SerialUSB.println(sizeof(outgoingdata));
	for (int i = 0; i < sizeof(outgoingdata); i++){
	SerialUSB.println(outgoingdata.payload[i]);
	}*/

	float torques[numberOfMotors] = {3.3, 5.4, 6.2}; 

	

	outgoingdata.Set_Packet_Instructions(RETURN_ANGLES); 
	outgoingdata.Set_Packet_Payload(torques); 

	SerialUSB.println(outgoingdata.Get_Packet_Instructions());

	float *readTorques = outgoingdata.Get_Packet_Payload(); 


	for (int i = 0; i < numberOfMotors; i++)
	{
		SerialUSB.println(*(readTorques+i)); 
	}

}

void loop(){



}
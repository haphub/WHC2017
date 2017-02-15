// hAPI_data_packets.h

#ifndef _HAPI_DATA_PACKETS_h
#define _HAPI_DATA_PACKETS_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif


#endif

enum PACKET_INSTRUCTIONS
{
	SET_HARDWARE_PORTS,
	SET_HARDWARE_IC,
	RETURN_ANGLES,
	COMMAND_TORQUES
}; 


typedef struct Hardware_Ports_Packet
{
	byte packet_instructions;
	byte J3; 
	byte J2;
	byte J5; 
	byte J4; 

} Hardware_Ports_Packet;

class Data_Packet
{
	public:
		Data_Packet(int size); 


	private: 
		struct 
		{

		};

}; 



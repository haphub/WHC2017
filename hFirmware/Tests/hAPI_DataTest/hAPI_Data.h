//// hAPI_Data.h
//
//#ifndef _HAPI_DATA_h
//#define _HAPI_DATA_h
//
//#if defined(ARDUINO) && ARDUINO >= 100
//	#include "arduino.h"
//#else
//	#include "WProgram.h"
//#endif
//
//
//#endif
//
//// hAPI_data_packets.h
//
//#ifndef _HAPI_DATA_PACKETS_h
//#define _HAPI_DATA_PACKETS_h
//
//#if defined(ARDUINO) && ARDUINO >= 100
//#include "arduino.h"
//#else
//#include "WProgram.h"
//#endif
//
//
//#endif

#include <cstdint>

	enum PACKET_INSTRUCTIONS
	{
		SET_HARDWARE_PORTS,
		SET_HARDWARE_IC,
		RETURN_ANGLES,
		COMMAND_TORQUES
	};


	typedef struct Hardware_Ports_Packet
	{
		int8_t packet_instructions;
		int8_t port[4];

	};

	typedef struct Packets
	{
		PACKET_INSTRUCTIONS mPacketInstructions;
		float *mPayload;
	} ;


	class Data
	{
	public:
		Data(int32_t number_of_motors)
			: mNumberOfMotors(number_of_motors)
		{
			mPacket.mPayload = new float[number_of_motors]{};
		}
		~Data(){
			delete[] mPacket.mPayload;
			mPacket.mPayload = 0;
		}

		PACKET_INSTRUCTIONS Get_Packet_Instructions(void){
			return mPacket.mPacketInstructions;
		}

		void Set_Packet_Instructions(PACKET_INSTRUCTIONS pkt_instructions){
			mPacket.mPacketInstructions = pkt_instructions;
		}

		float *Get_Packet_Payload(void){
			return mPacket.mPayload; 

		}

		void Set_Packet_Payload(float data_payload[]){
			mPacket.mPayload = data_payload; 
		}

		Packets Get_Packet(void){
			return mPacket; 
		}

	private:
		Packets mPacket; 
		int32_t mNumberOfMotors;
	};



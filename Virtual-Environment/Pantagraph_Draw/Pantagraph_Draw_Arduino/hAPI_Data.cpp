// 
// 
// 

#include "hAPI_Data.h"


Data_Packet::Data_Packet(int numOfMotors)
{
	for (int i = 0; i < numOfMotors; i++)
		payload[i] = 0.0; 

}
/*	This class defines all the methods needed to read and	*/
/*	store the information from the sensors, such as motor	*/
/*	encoders counts, FT sensor values and indirect measur-	*/
/*	ments as the platform orientation.						*/
/*	Further, this source file includes also the implemen-	*/
/*	tation of the vrpn class.								*/

#include "stdafx.h"//"StdAfx.h"

#include "devMeasurementFns.h"

//	Definition of class global variables.	//
double FTsensorData[6];
double currentTheta[2];
//VECTd& FTsensorData=VECTd(6);
//VECTd& currentTheta=VECTd(2);
//double limbLength[3];
//long Encoders[3];
/********************************************/

//	Definition of class objects pointers.	//
UDP_Communication *UDPcomm;
//rhAnkleDevice  *rhAnkleDev;
//cCommDevice *canCommunication;
/********************************************/


devMeasurementFns::devMeasurementFns(void)
{
	//	Initializations of class pointers.	//
	UDPcomm = new UDP_Communication;
//	rhAnkleDev = new rhAnkleDevice;
//	canCommunication = new cCommDevice ;
}

devMeasurementFns::~devMeasurementFns(void)
{
}

void devMeasurementFns::RequestFTsensorData()
{
	//	This function send a message through UDP to the FT sensor Net Box	//
	//	to request a single value. For details on the Message format refer	//
	//	to the ATI Industrial Automation documentation.						//

	char MessageBuf[BufLenS];
	//	Definition of the message. The message will request a single sensor reading	//
	MessageBuf[0] = 0x12;
	MessageBuf[1] = 0x34;
	MessageBuf[2] = 0x00;
	MessageBuf[3] = 0x02;
	MessageBuf[4] = 0x00;
	MessageBuf[5] = 0x00;
	MessageBuf[6] = 0x00;
	MessageBuf[7] = 0x01;

	UDPcomm->UDPSendMessage(MessageBuf);
}

void devMeasurementFns::BiasFTsensor()
{
	char MessageBuf[BufLenS];
	//	Definition of the message. The message will request a single sensor reading	//
	MessageBuf[0] = 0x12;
	MessageBuf[1] = 0x34;
	MessageBuf[2] = 0x00;
	MessageBuf[3] = 0x42;
	MessageBuf[4] = 0x00;
	MessageBuf[5] = 0x00;
	MessageBuf[6] = 0x00;
	MessageBuf[7] = 0x01;

	UDPcomm->UDPSendMessage(MessageBuf);
}

void devMeasurementFns::ReadFTsensorData()
{
	//	This function sends a data request to the FT sensor Net	//
	//	Box. Then it reads the message sent by the FT sensor. 	//
	//	(See RequestFTsensorData() ).							//

	char RecvBuf[BufLenR];

	//	Request Data	//
	RequestFTsensorData();

	//	Receive the message	//
	UDPcomm->UDPReceieveMessage(RecvBuf);

	//	Decode the message and save x, y, z forces in the first three positions of FTsensorData and	//
	//	x, y, z torques in the last three positions.												//
	for(int i=0;i<6;i++)
	{
		int u = 12+i*4;
		long byte1 = RecvBuf[u];
		u = 13+i*4;
		long byte2 = RecvBuf[u];
		u = 14+i*4;
		long byte3 = RecvBuf[u];
		u = 15+i*4;
		long byte4 = RecvBuf[u];

		byte1=(byte1<<24)&0xFF000000;
		byte2=(byte2<<16)&0xFF0000;
		byte3=(byte3<<8)&0xFF00;

		if(i<=2)	FTsensorData[i] =(double)(byte1 + byte2 +byte3 +byte4 );//	Forces
		else		FTsensorData[i] =(double)(byte1 + byte2 +byte3 +byte4 );//	Torques
	}
//printf("Sensor data Fx= %f Fy= %f \n", FTsensorData[0], FTsensorData[1]);
	//	printf("Sensor data Fx= %f, Fy= %f, Fz= %f, Tx= %f, Ty= %f, Tz= %f,\n", FTsensorData[0], FTsensorData[1], FTsensorData[2], FTsensorData[3], FTsensorData[4], FTsensorData[5]);

}

/*
void devMeasurementFns::GetLimbPositionData(double limbLength[3])
{
	//	This function gets the DC motor encoders counts and convert it	//
	//	to limb lengths in meters.										//
	//long Encoders[3];
	canCommunication->getBroadCastData(BCAST_POSITION,Encoders);

	rhAnkleDev->CountsToLength(JOINT_1,Encoders[0],limbLength[0]);
	rhAnkleDev->CountsToLength(JOINT_2,Encoders[1],limbLength[1]);
	rhAnkleDev->CountsToLength(JOINT_3,Encoders[2],limbLength[2]);
}
*/
void devMeasurementFns::GetFTData(double (&FTData)[6])
{
	//	This function allows to read the FTsensor data from outside the class.	//
	for (int i=0; i<6; i++)
	{
		FTData[i] = FTsensorData[i]/1000000;
	}
	
}
void devMeasurementFns::GetRawFTData(double (&FTData)[6])
{
	//	This function allows to read the FTsensor data from outside the class.	//
	for (int i=0; i<6; i++)
	{
		FTData[i] = FTsensorData[i];
	}

}
/*
void devMeasurementFns::ComputeFDKin()
{
	//	This function gets the limb lengths in meters and	//
	//	computes the mechanism forward kinematics.	The		//
	//	platform oriantation is store in the variable		//
	//	currentTheta.										//
	GetLimbPositionData(limbLength);
	rhAnkleDev->FDKin(limbLength,currentTheta);
}
void devMeasurementFns::GetPlatformOrientation(VECTd& theta)
{
	//	This function allows to read the platform orientation	//
	//	from outside the class.									//
	theta = currentTheta;
}
// ----------------------------------------------------------------------------
// vrpn_nexus implementation 
// ----------------------------------------------------------------------------

devMeasurementFns_vrpn::devMeasurementFns_vrpn()
{
	//	This constructor initializes the vrpn channel	//
	//	the port and starts the server.					//
	channel = 0;
	conn = 0;
	port = vrpn_DEFAULT_LISTEN_PORT_NO;
	init();
}

devMeasurementFns_vrpn::~devMeasurementFns_vrpn()
{
}

void devMeasurementFns_vrpn::shutdown()
{
	//	This function shuts down the vrpn server.	//

	if ( channel ) {
		delete channel;
	}
	if ( conn ) {
		//conn->removeReference();
		delete conn;
	}
}

void devMeasurementFns_vrpn::init()
{
	//	This function shuts down the vrpn server and	//
	//	then starst a new vrpn server. The server name	//
	//	is AnkleDevice@IPaddress:3883.					//
	//	The broadcasted data structure has a size of 6.	//

	shutdown();

	conn = vrpn_create_server_connection(port);
	channel = new vrpn_Analog_Server("AnkleDevice", conn);
	channel->setNumChannels(9);
	
}

int devMeasurementFns_vrpn::setVrpnServerPort(int _port)
{
	//	This function sets the server port.	//
	port = _port;
	init();
	return 0;
}

void devMeasurementFns_vrpn::processData()
{
	//	This function saves the current orientation of the platform	//
	//	in radiants and degrees and the platform torques relative	//
	//	to the two axis of motion. Further, it broadcasts the data	//
	//	on the vrpn channel.										//

	channel->channels()[0] = currentTheta(0);//[radiants] 
	channel->channels()[1] = currentTheta(1);//[radiants] 
	channel->channels()[2] = 180/pi*currentTheta(0);//[degrees] +-40
	channel->channels()[3] = 180/pi*currentTheta(1);//[degrees] +-40
	channel->channels()[4] = FTsensorData(3);//[Nm] -+40
	channel->channels()[5] = FTsensorData(4);//[Nm] -+40
	channel->channels()[6] = Encoders[0];//limbLength[0]; [counts]
	channel->channels()[7] = Encoders[1];//limbLength[1]; [counts]
	channel->channels()[8] = Encoders[2];//limbLength[2]; [counts]

	// 1024 Hz
	channel->report_changes();
	
	//
	conn->mainloop();
	if (!conn->doing_okay()) {
      init();
    }
	
}

*/
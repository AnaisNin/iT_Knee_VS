#pragma once

//#include "rhAnkleDevice.h"
//#include "commdevice.h"
#include "UDP_Communication.h"
//#include <vrpn_Connection.h>
//#include <vrpn_Analog.h>




class devMeasurementFns
{
public:
	devMeasurementFns(void);
	~devMeasurementFns(void);

	void RequestFTsensorData();
	void BiasFTsensor();
	void ReadFTsensorData();
	void GetFTData(double (&FTData)[6]);
	void GetRawFTData(double (&FTData)[6]);
	
	//void GetLimbPositionData(double limbLength[3]);
	//void GetFTData(VECTd& FTData);
	//void ComputeFDKin();
	//void GetPlatformOrientation(VECTd& theta);

};

/*
class devMeasurementFns_vrpn {

	public:
		devMeasurementFns_vrpn();
		~devMeasurementFns_vrpn();

		void processData();
		int setVrpnServerPort(int port);

	private:
		void init();
		void shutdown();
		int port;
		vrpn_Connection * conn;
		vrpn_Analog_Server * channel;
		

	};
*/
extern UDP_Communication *UDPcomm;
//extern rhAnkleDevice  *rhAnkleDev;
//extern cCommDevice *canCommunication;
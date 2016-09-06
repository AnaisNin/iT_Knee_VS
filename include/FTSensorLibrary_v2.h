#ifndef _BOARDLIBRARY_H_
#define _BOARDLIBRARY_H_

#include <winsock2.h>
#include <stdio.h>


#include "phil_FTboard_v2.h"
#include "userConfig.h"
#include "utils.h"

#define MAX_BOARDS 10
#define MAX_FT_SENSORS 10

#define DLL_EXPORT

#if defined DLL_EXPORT
#define DECLDIR __declspec(dllexport)
#else
#define DECLDIR __declspec(dllimport)
#endif

//extern "C": tells the compiler that it is okay to use this in C or C++.
#ifdef __cplusplus
extern "C" {
#endif
	

//FTSensor is the single force torque sensor structure
typedef struct FT_Sensor {	
	// Channel Raw Data
	int ChRaw_Offs[6];//Channel Raw Data with offset applied
	int FT[6];// Force Torque-Data in (SI units * 1000000)
	unsigned short ChRaw[6];//Channel Raw Data without offset

	long temp_Vdc;
	long tStamp;//TimeStamp in milisecconds
	int fault;
	int filt_FT[6];// Filtered Force Torque-Data in (SI units * 1000000)
	
	float ft[6];// Force Torque-Data in SI units
	float filt_ft[6];// Filtered Force-Torque Data in SI units

} FT_Sensor;
FT_Sensor FTSensor[10];

//  All_FT_Sensors is the structure which contains all the data of all possible connected sensors
typedef struct All_FT_Sensors{ 
	FT_Sensor FTSensor[MAX_FT_SENSORS];	
}All_FT_Sensors ;


	//TCP commands
	//All functions return an error flag (-1 if error)
	//DECLDIR int InitLibrary(int boardsCount);
	//DECLDIR int ConnectTCP(int BoardNumber);
	//DECLDIR int SetBCastRate(int BoardNumber, int BCastRate);
	//DECLDIR int StopBCast(int BoardNumber);
	//DECLDIR int GetBCastRate(int BoardNumber);
	//DECLDIR int SetBCastPolicy(int BoardNumber, int BCastPolicy);
	//DECLDIR int GetBCastPolicy(int BoardNumber);
	DECLDIR int CalibrateOffsets(int BoardNumber);
	DECLDIR int GetCalibrationOffsets(int BoardNumber);
	DECLDIR int ConnectTCPDefaultIP(void);
	DECLDIR int ConnectTCPUserIP(void);
	DECLDIR void CloseSockets(void);

	//UDP commands and functions
	//DECLDIR int ConnectUDP(void);
	DECLDIR int ConnectUDPDefaultIP(void);
	DECLDIR int GetFTSensorData(int BoardID, FT_Sensor &MyFTSensorData);
	DECLDIR int GetAllFTSensorsData(All_FT_Sensors &MyFTSensorData);
	//DECLDIR int GetBCastData(CharBuff &packetToReceive);
	DECLDIR int ScanForActiveBoards(void);
	//DECLDIR int GetActiveBoards(int *boardIDs);
	DECLDIR int GetActiveBoardsIDsIPs(int *boardIDs, int boardIPs[MAX_BOARDS][4]);

	
#ifdef __cplusplus
}
#endif

#endif
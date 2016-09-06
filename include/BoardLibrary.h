#ifndef _BOARDLIBRARY_H_
#define _BOARDLIBRARY_H_

#include <winsock2.h>
#include <stdio.h>


#include "phil_board.h"
#include "utils.h"

#define MAX_MOTOR_BOARDS 10
#define MAX_BOARDS 10
#define BASEADDR 70

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


	//TCP

	//All functions return an error flag (-1 if error)

	/**
	 Any client using the library must call this function once before any other one.
	*/
	DECLDIR int InitLibrary(int boardsCount);

	DECLDIR int ConnectTCP(int BoardNumber);
	DECLDIR int SendTCP(SOCKET SckID, CharBuff* packetToBeSent);
	DECLDIR int ReceiveTCP(SOCKET SckID, CharBuff* packetToReceive);

	DECLDIR int GetBoardType(int BoardNumber);
	DECLDIR float GetFirmVersion(int BoardNumber); 
	DECLDIR int SetFirmVersion(int BoardNumber, char* Firm);

	DECLDIR int ClearBoardFault(int BoardNumber);
	DECLDIR int GetBoardFault(int BoardNumber);

	DECLDIR int SetPidGains(int BoardNumber, char gainSet, char * gains);
	DECLDIR int GetPidGains(int BoardNumber, char gainSet, long * GainsBuf); // Returns an error flag. The buffer parameter are filled with the gains request
	DECLDIR int SetPidGainScale(int BoardNumber, char gainSet, char * GainsScale);
	DECLDIR int GetPidGainScale(int BoardNumber, char gainSet, long * GainsBuf); // Returns an error flag. The buffer parameter are filled with the gain scale request
	DECLDIR int SetILimGain(int BoardNumber, char gainSet, long ILim);
	DECLDIR long GetILimGain(int BoardNumber, char gainSet);
	
	DECLDIR int GetPidOffset(int BoardNumber);
	DECLDIR long GetPidError(int BoardNumber);
	DECLDIR int GetPidOutput(int BoardNumber);

	DECLDIR long GetPosition(int BoardNumber);
	DECLDIR int GetVelocity(int BoardNumber);
	DECLDIR int GetTorque(int BoardNumber);

	DECLDIR long GetDesiredPosition(int BoardNumber);
	DECLDIR int GetDesiredVelocity(int BoardNumber);
	DECLDIR int GetDesiredTorque(int BoardNumber);

	DECLDIR int SetAccel(int BoardNumber, int Accel);
	DECLDIR int GetAccel(int BoardNumber);

	DECLDIR int SetMinPosition(int BoardNumber, long MinPosition);
	DECLDIR long GetMinPosition(int BoardNumber);
	DECLDIR int SetMaxPosition(int BoardNumber, long MaxPosition);
	DECLDIR long GetMaxPosition(int BoardNumber);
	DECLDIR int SetMaxVelocity(int BoardNumber, int MaxVelocity);
	DECLDIR int GetMaxVelocity(int BoardNumber);
	DECLDIR int SetMaxTorque(int BoardNumber, int MaxTorque);
	DECLDIR int GetMaxTorque(int BoardNumber);
	DECLDIR int SetMinVelocity(int BoardNumber, int MinVelocity);
	DECLDIR int GetMinVelocity(int BoardNumber);
	
	DECLDIR int SetCurrentLimit(int BoardNumber, int CurrentLimit);
	DECLDIR int GetCurrentLimit(int BoardNumber);

	DECLDIR int SetBCastRate(int BoardNumber, int BCastRate);
	//DECLDIR int SetBCastRate(int BoardNumber, int BCastRate, char TwinSatus)
	DECLDIR int StopBCast(int BoardNumber);
	DECLDIR int GetBCastRate(int BoardNumber);
	DECLDIR int SetBCastPolicy(int BoardNumber, int BCastPolicy);
	DECLDIR int GetBCastPolicy(int BoardNumber);
	//DECLDIR int SetExtraBCastPolicy(int BoardNumber);
	DECLDIR int SetExtraBCastPolicy(int BoardNumber, int ExtraBCastPolicy);
	DECLDIR int GetExtraBCastPolicy(int BoardNumber);


	DECLDIR int SetEncoderLines(int BoardNumber, long EncoderLines);
	DECLDIR long GetEncoderLines(int BoardNumber);
	DECLDIR int SetMotorPoles(int BoardNumber, char poles);
	DECLDIR int GetMotorPoles(int BoardNumber);
	DECLDIR int SetAnalogInputs(int BoardNumber, char analogInputs);
	DECLDIR int DoRecalculatePosition(int BoardNumber, char Calib);
	DECLDIR int GetAnalogInputs(int BoardNumber);

	DECLDIR int CmdUpgrade(int BoardNumber);
	DECLDIR int SaveParamsToFlash(int BoardNumber);
	DECLDIR int LoadParamsFromFlash(int BoardNumber);
	DECLDIR int LoadDefaultParams(int BoardNumber);

	DECLDIR int ControllerRun(int BoardNumber);
    DECLDIR int ControllerIdle(int BoardNumber);
	DECLDIR int EnablePwmPad(int BoardNumber);
	DECLDIR int DisablePwmPad(int BoardNumber);
	
	DECLDIR int DoCalibrate(int BoardNumber, int Direction, long InitPos);
	DECLDIR int SetCalibrationCurrent(int BoardNumber, int i);
	DECLDIR int GetCalibrationCurrent(int BoardNumber);

	DECLDIR int SetAbsoluteZero(int BoardNumber, int AbsZero);
	DECLDIR int GetAbsoluteZero(int BoardNumber);
	DECLDIR int SetTorqueAccel(int BoardNumber, char tAccel);
	DECLDIR int GetTorqueAccel(int BoardNumber);

	DECLDIR int SetMotorType(int BoardNumber, char motorType);
	DECLDIR int GetMotorType(int BoardNumber);

	DECLDIR void CloseSockets(void);

	DECLDIR int SetMinAbsPosition(int BoardNumber, int minAbsPos);
	DECLDIR int GetMinAbsPosition(int BoardNumber);

	DECLDIR int SetMaxAbsPosition(int BoardNumber, int maxAbsPos);
	DECLDIR int GetMaxAbsPosition(int BoardNumber);
	
	// FT Sensor related commands

	DECLDIR int CalibrateOffsets(int BoardNumber);
	DECLDIR int GetCalibrationOffsets(int BoardNumber);

	DECLDIR int SetConversionFactors(int BoardNumber, int channel, int multiplier, int offset);
	DECLDIR int GetConversionFactors(int BoardNumber, char channel, int* FactorsBuf);

	DECLDIR int SetAvarageSamples(int BoardNumber, int samples);
	DECLDIR int GetAvarageSamples(int BoardNumber, int samples);

	DECLDIR int SetMatrixRow(int BoardNumber, char row, float *matrixRow);
	DECLDIR int GetMatrixRow(int BoardNumber, char row_req, char row_recv, float matrixRow[6]);

	// FT Sensor related commands


	//UDP

	DECLDIR int ConnectUDP(void);
	DECLDIR int SendUDP(SOCKET SckID, CharBuff* packetToBeSent); 
	DECLDIR int recvfromTimeOutUDP(SOCKET socket, long sec, long usec);
	DECLDIR int ReceiveUDP(SOCKET SckID, CharBuff* packetToReceive);
	DECLDIR int ReceiveUDPWithTimeOut(SOCKET UDPSckID, CharBuff* packetToReceive);

	DECLDIR int GetBCastData(CharBuff &packetToReceive);
	DECLDIR int GetBCastFTData(FT_Sensor myFTSensor);
	DECLDIR int WriteBCastData(int BoardNumber, FILE * stream);

	DECLDIR int ScanForActiveBoards(void);
	DECLDIR int GetActiveBoards(int *boardIDs);

	DECLDIR int SetDesiredPosition(int DesPos[MAX_BOARDS]);
	DECLDIR int SetDesiredStiffnessDamping(int DesSiff[MAX_BOARDS], int DesDamp[MAX_BOARDS]);
	DECLDIR int SetDesiredVelocity(int DesVel[MAX_BOARDS]);
	DECLDIR int SetDesiredTorque(int DesTor[MAX_BOARDS]);
	DECLDIR int SetPidOffset(int DesOffset[MAX_BOARDS]);

	DECLDIR int StartPositionControl(void);	// start all active boards
	DECLDIR int StartBoardPositionControl(int boardNumber); // start the selected board
	DECLDIR int StopPositionControl(void);  // stop all boards
	
	DECLDIR int StartVelocityControl(void);
	DECLDIR int StartTorqueControl(void);

	DECLDIR int StopMotorUDP(void);

	DECLDIR int SetTorqueOnOff(int BoardNumber, int torqueFlag);
	DECLDIR int SetMotorConfig(int BoardNumber, char  *mConfig);
	DECLDIR int SetMotorConfig2(int BoardNumber, char  *mConfig);

	DECLDIR int GetTorqueOnOff(int BoardNumber);
	DECLDIR int GetMotorConfig(int BoardNumber);
	DECLDIR int GetMotorConfig2(int BoardNumber);

	DECLDIR int SetPDlinkGains(int BoardNumber, char *Gains);
	DECLDIR int GetPDlinkGains(int BoardNumber, long* GainsBuf);

	
#ifdef __cplusplus
}
#endif

#endif
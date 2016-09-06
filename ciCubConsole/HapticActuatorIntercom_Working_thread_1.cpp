// ciCubConsole.cpp : Defines the entry point for the console application.

//
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif

#include "stdafx.h"

#include <winsock2.h>
#include <ws2tcpip.h>
#include <stdio.h>
#include <time.h>

#pragma comment(lib, "Ws2_32.lib")
#include "UDP_Communication.h"
#include "devMeasurementFns.h"

double FTData[6];

#include "stdafx.h"
#include "BoardLibrary.h"

#pragma comment(lib, "BoardLibrary.lib")
#include "BoardLibrary.h"
#include "phil_board.h"
#include "math.h"
#include "conio.h"
#include <iostream>	//yogesh
using namespace std;
#include "shTime.h"
#include <fstream>  //yogesh
#include "legKin.h"
#include "trajectory.h"
#include <windows.h>

//Timer and thread
#include "CPrecisionClock.h"
#include "CThread.h"

#define TRAJ_SAMPLES 50000
#define SAMPLES 1000000
#define PI 3.141592653589793
# define SIMULATION_TIME 3600
#define TSAMPLE 0.001//0.0005
//#define BCAST_POLICY 51455
//#define BCAST_POLICY 52479
//#define BCAST_POLICY 52991

#define BCAST_POLICY 1791
#define EXTRA_BCAST_POLICY 3584
//#define BCAST_POLICY 1
//#define BCAST_RATE 0.0010 //0.0005
#define BCAST_RATE 1

//#define MAX_BOARDS 15
#define Fgain 0.0073 
#define Tgain 1
#define offsetF 46895 //30358 // changed values on 26/2/13
#define offsetT 49150//30675 //
#define gainA 35.53
#define gainB 1.0876e-004
#define gainC 0.00175// 0.007 //original 0.007


#define Cdes 0.008 //// value of damping set manually for testing/////
#define FIRST_LBODY_BOARD 0
#define LAST_LBODY_BOARD 2
#define LEFT_HIP_SAG	5
#define LEFT_HIP_LAT	11
#define LEFT_HIP_ROT	12
#define LEFT_KNEE		13
#define LEFT_ANKLE_SAG	14
#define LEFT_ANKLE_LAT	15

#define RIGHT_HIP_SAG	4
#define RIGHT_HIP_LAT	6
#define RIGHT_HIP_ROT	7
#define RIGHT_KNEE		8
#define RIGHT_ANKLE_SAG	9
#define RIGHT_ANKLE_LAT	10

#define MOVETOHOME_VEL	400

#define DEGTORAD(x)  x*6.28/360.0
#define RADTODEG(x)  x*360.0/6.28
#define RPMTODEGSEC(x)  x*360.0/(100.0*60.0)
#define DEGSECTORPM(x)  100.0*60*x/360.0
#define IN_DEG 1
#define IN_RAD 2

#define USE_MOTOR_ENCODER 1
#define USE_JOINT_ENCODER 2

#define PID_CONTROL 1
#define LQR_CONTROL 2
#define gainSet 1 // for voltage control **Yogesh**
#define ON 1
#define OFF 0
#define FORWARD 1
#define REVERSE 0
#define posMAX 1200000
#define	posMIN 100000

int desVel[MAX_BOARDS]={0,0,0,0,0,0,0,0,0,0,
						0,0,0,0,0,0,0,0,0,0,
						0,0,0,0,0,0,0,0,0,0};

int desPos[MAX_BOARDS]={0,0,0,0,0,0,0,0,0,0,
						0,0,0,0,0,0,0,0,0,0,
						0,0,0,0,0,0,0,0,0,0};

int PosOff[MAX_BOARDS]={0,0,0,0,0,0,0,0,0,0,
						0,0,0,0,0,0,0,0,0,0,
						0,0,0,0,0,0,0,0,0,0};

int Destor[MAX_BOARDS]={0,0,0,0,0,0,0,0,0,0,
						0,0,0,0,0,0,0,0,0,0,
						0,0,0,0,0,0,0,0,0,0};
// 
//int homePos[MAX_BOARDS]={0,0,0,12.904541,12.904541,0,0,-27.460726,14.556184,0,      // in degrees
//						 0,0,-27.460726,14.556184,0,0,0,0,0,0,
//						 0,0,0,0,0,0,0,0,0,0};

int homePos[MAX_BOARDS]={0,0,0,0,0,0,0,0,0,0,      // in degrees
						 0,0,0,0,0,0,0,0,0,0,
						 0,0,0,0,0,0,0,0,0,0};

int homeVel[MAX_BOARDS]={20,20,20,20,20,20,20,20,20,20,  // in degrees
						 20,20,20,20,20,20,20,20,20,20,
						 20,20,20,20,20,20,20,20,20,20};


int activeBoards[MAX_BOARDS]={0,0,0,0,0,0,0,0,0,0,
							  0,0,0,0,0,0,0,0,0,0,
							  0,0,0,0,0,0,0,0,0,0};

float desPosDeg[MAX_BOARDS]={0,0,0,0,0,0,0,0,0,0,      // in degrees
						 0,0,0,0,0,0,0,0,0,0,
						 0,0,0,0,0,0,0,0,0,0};

int noActiveBoards=0;

float traj[MAX_BOARDS][TRAJ_SAMPLES];
int trajPoints;
int trajdt=1;
int curTrajpoint=1;

int b=0;
int counter =0;
shTime timeObject;
double startLoopTime;
double startSimTime;
double curSimTime,starTime;
double ts;
double feq;
double timeudp, t1, t2;
char TogglePrint=1;
float Force;
float Torque;

int desTor;
int pidOutput;
long pidError;
int pid_off[15];

//float Cmatrix[2][2] = { { -0.00449707834668421, -1.16232662072916e-06}, { -1.29645017699755e-05, -4.73804273062362e-05 } }; 
float Cmatrix[2][2] = { { -0.00429463384632091, -5.73945959687287e-07}, { 6.00819012212136e-05, -4.81132325882713e-05 } }; 
	
	

	

//int A[3][4] = { { 0, 0, 0, 0 }, { 0, 0, 0, 0 }, { 0, 0, 0, 0 } };

int desStiff[MAX_BOARDS]={0,0,0,0,0,0,0,0,0,0,
						0,0,0,0,0,0,0,0,0,0,
						0,0,0,0,0,0,0,0,0,0};
int desDamp[MAX_BOARDS]={0,0,0,0,0,0,0,0,0,0,
						0,0,0,0,0,0,0,0,0,0,
						0,0,0,0,0,0,0,0,0,0};

float chirpSignal = 0.0;

long gains[3];
char* Cgains;

int DesOffset[MAX_BOARDS]={0,0,0,0,0,0,0,0,0,0,
						0,0,0,0,0,0,0,0,0,0,
						0,0,0,0,0,0,0,0,0,0}; // for voltage control **Yogesh**

CharBuff packetToReceive;


typedef struct joint_data {
    int id;
	long pos;
	int tpos;
	float tposRAD;
	float finalTime;
	int vel;
	int tvel;
	int tor;
	int ttor;
	int motorEncLimit1;
	int motorEncLimit2;
	int jointEncLimit1;
	int jointEncLimit2;
	float jointLimit1DEG;
	float jointLimit2DEG;
	float jointLimit1RAD;
	float jointLimit2RAD;
	float motorEncLine;
	float jointEncLine;
	float jointPosRAD;
	float jointPosDEG;
	int minvel;
	int maxvel;
	int maxCur;
	int pgain;
	int igain;
	int dgain;
	int ilimit;
	int gainscale;
	int offset;
	int pidOutput;
	long pidError;
	int current;
	int trajDSP;
	long temp_Vdc;
	long tStamp;
	int fault;
	int anInput1;
	int anInput2;
	int anInput3;
	int anInput4;
	int absPos1;
	int absPos2;
	int quickSpeed;
	int realCurrent;
	int isActive;
	int isConnected;
	int controlType;
	long TwinActPos;
	long TwinTargPos;
	long TwinVel;
	int Xaccleration;
	int Yaccleration;
	int Zaccleration;
	float Force;
	float Torque;

	float getJointPos(int fromEncoder, int posUnit)
	{
	
		if(fromEncoder==USE_MOTOR_ENCODER)
			jointPosRAD=jointLimit1RAD + motorEncLine*((float)pos - (float)motorEncLimit1);
		else if(fromEncoder==USE_JOINT_ENCODER)
			jointPosRAD=jointLimit1RAD + jointEncLine*((float)absPos1 - (float)jointEncLimit1);	
		jointPosDEG=RADTODEG(jointPosRAD);

		if(posUnit==IN_RAD)
			return jointPosRAD;
		else if(posUnit==IN_DEG)
			return jointPosDEG;
		else return 0;
	};

	void setJointTargetPos(float targetPos, int posUnit)
	{
		if(posUnit==IN_DEG)
			targetPos=DEGTORAD(targetPos);
		if(motorEncLine!=0)
		{
			tpos=motorEncLimit1 + (int)((targetPos - jointLimit1RAD)/motorEncLine);
		
		}
		else tpos=pos;
			
	};

} jointData;

// status of the main simulation haptics loop
bool simulationRunning = false;

// Precision clock
cPrecisionClock* PrintClock;
cPrecisionClock* mainClock;

// main haptics loop
void updateLoop(void);

//second prototype 
jointData joint[1]={

1,0,0,0,0,0,200,0,0,10000,10000,0,0,60,-60,0,0,0,0,0,0,0,3333,0,1600,16,1000,32000,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,PID_CONTROL,
};

//jointData joint[MAX_BOARDS];

int oldTarget=0;
int r;
int loop=0;
int sampleloop=0;
int ch=0;
int stopSim=0;

int accn[SAMPLES][3]; //created for aceleration on 13-2-13
//11/3/2013 original double data[SAMPLES][12];
int data[SAMPLES][12];
float FT_data[SAMPLES][6];
float sT[SAMPLES];

///////////////////////////////////////////////////////////////////////////////////////////////////////

long temp;
long posPID[3];
long torPID[3];
char pidGains[20];
char sinewaveON=OFF;
int FunctionOffset=0;
char rampON=OFF, rampDirection=FORWARD;
char	storeData=OFF,	deleteData=OFF;
long	dataCounter=0;
float	sineTime=0.5;
int	rampSTEP=1;
int	rampPos=0;
////////////////////////////////////////////////////////////////////////////////////////////////////////
		
void gotoxy( int column, int line )
  {
  COORD coord;
  coord.X = column;
  coord.Y = line;
  SetConsoleCursorPosition(
    GetStdHandle( STD_OUTPUT_HANDLE ),
    coord
	);

  }

float chirpFnc(double currentTime, int f0, int f1, int tf)
{
	float y=0;
	float omega = 0;
	
	omega = 2* PI *(f0 + ((f1 - f0)/tf)*currentTime);
	return y = (sin(omega * currentTime));
}


void setControllerGains()
{


//posPID[0]=0;
//posPID[1]=0;
//posPID[2]=0;

torPID[0]=0;
torPID[1]=0;
torPID[2]=0;

// sprintf_s is changing gains (long datatypes) to char types for SetPidGains function variable char * gains. 
sprintf_s(pidGains, sizeof(pidGains)/sizeof(char), "%d,%d,%d", gains[0],gains[1],gains[2]); 
		temp=SetPidGains(2,1,pidGains);


//sprintf_s(pidGains, sizeof(pidGains)/sizeof(char), "%d,%d,%d", gains[0],gains[1],gains[2]);
//		temp=SetPidGains(2,2,pidGains);

//		GetPidGains(1,1,gains);
//	gotoxy(1,20); printf("p=%d, i=%d, d=%d \n",gains[0],gains[1],gains[2]);

	//	GetPidGains(2,1,gains);
		//gotoxy(1,21); printf("p=%d, i=%d, d=%d \n",gains[0],gains[1],gains[2]);

}
//{
//	int reply=SetPidGains(1,1,pidGains);
//	if(reply!=-1)
//	{
//		sprintf_s(pidGains, sizeof(pidGains)/sizeof(char), "%d,%d,%d", posPID[0],posPID[1],posPID[2]);
//	}
//	
//	return;
//}
/******************************************/
void loadJointDataFromDrivers()
/******************************************/
{
	long temp;
	long r[3];

for(b=FIRST_LBODY_BOARD;b<LAST_LBODY_BOARD;b++){
	if(joint[b].isConnected){
	temp=GetPosition(activeBoards[b]);
	joint[b].pos=temp;
	GetVelocity(b);
	joint[b].vel=temp;
	GetTorque(b);
	joint[b].tor=temp;
	temp=GetMinPosition(activeBoards[b]);
	joint[b].motorEncLimit1=temp;
	temp=GetMaxPosition(activeBoards[b]);
	joint[b].motorEncLimit2=temp;
	temp=GetMinVelocity(activeBoards[b]);
	joint[b].minvel=temp;
	temp=GetMaxVelocity(activeBoards[b]);
	joint[b].maxvel=temp;
	temp=GetCurrentLimit(activeBoards[b]);
	joint[b].current=temp;
	temp=GetPidGains(activeBoards[b],1,r);
	joint[b].pgain=r[0];
	joint[b].igain=r[1];
	joint[b].dgain=r[2];
	temp=GetPidGainScale(activeBoards[b],1,r);
	joint[b].gainscale=r[0];
	temp=GetILimGain(activeBoards[b],1);
	joint[b].ilimit=temp;
	temp=GetPidOffset(activeBoards[b]);
	joint[b].offset=temp;

	printf("b%d,p:%d,minp:%d,maxp:%d,mc:%d,(p:%d,i:%d,d:%d, il:%d,o:%d)\n",
		activeBoards[b],joint[b].pos,joint[b].motorEncLimit1,joint[b].motorEncLimit2,joint[b].current,
		joint[b].pgain,joint[b].igain,joint[b].dgain,joint[b].ilimit,joint[b].offset);

	}
}
printf("\n");
}

///******************************************/
//void initJoints()
///******************************************/
//{
//
//for(b=FIRST_LBODY_BOARD;b<LAST_LBODY_BOARD;b++){
//	joint[b].tpos=joint[b].pos;
//	joint[b].jointLimit1RAD=DEGTORAD(joint[b].jointLimit1DEG);
//	joint[b].jointLimit2RAD=DEGTORAD(joint[b].jointLimit2DEG);
//
//	if(joint[b].motorEncLimit2!=joint[b].motorEncLimit1)
//		joint[b].motorEncLine=(joint[b].jointLimit2RAD - joint[b].jointLimit1RAD)/((float)joint[b].motorEncLimit2 - joint[b].motorEncLimit1);
//	else joint[b].motorEncLine=0;	
//
//	if(joint[b].jointEncLimit2!=joint[b].jointEncLimit1)
//		joint[b].jointEncLine=(joint[b].jointLimit2RAD - joint[b].jointLimit1RAD)/((float)joint[b].jointEncLimit2 - (float)joint[b].jointEncLimit1);	
//	else joint[b].jointEncLine=0;
//
//	printf("b%d,jl1:%.1f,jl2:%.1f,mEncLine:%.9f,jEncLine:%.9f\n",
//			activeBoards[b], joint[b].jointLimit1DEG,joint[b].jointLimit2DEG,
//			joint[b].motorEncLine,joint[b].jointEncLine);
//
//
//	}
//}
//


/******************************************/
int scanSystem()
/******************************************/
{
	int r=0;
	if((r=ScanForActiveBoards())>0){
		noActiveBoards=r;
		GetActiveBoards(activeBoards);
	}
	if( r==-1)
		printf("Scanning broadcast request: Sent failed!\n");
	else if(r==-2)
		printf("Broadcast packet read failed!\n");
	else if(r==-3)
		printf("Scan completed: No active boards found!\n");
	else{
		printf("Scan completed: (%d) Boards found !\n", r);
		for(b=FIRST_LBODY_BOARD;b<LAST_LBODY_BOARD;b++)
			if(activeBoards[b]>0)
				printf("BoardID:%d found at [IP:169.254.89.%d]\n", activeBoards[b],70+activeBoards[b]);
	}
	printf("\n");
	return r;
}

/******************************************/
int connectUDP()
/******************************************/
{
	r=ConnectUDP();
	if(r<0)
		printf("UDP Connection failed!, Error:%d\n",r);	
	else
		printf("UDP Connected, SocketID:%d\n",r);
	printf("\n");
	return r;
}

/******************************************/
void connectTCP()
/******************************************/
{
printf("Creating TCP Connection....\n");
for(b=FIRST_LBODY_BOARD;b<MAX_BOARDS;b++)
	if(activeBoards[b]>0){
		if((r=ConnectTCP(activeBoards[b]))>0){
			printf("BoardID:%d ->TCP Connection established[IP:169.254.89.%d, SocketID:%d ]\n", activeBoards[b],70+activeBoards[b],r);
			joint[b].isConnected=1;
		}
		else{
			printf("BoardID:%d ->TCP Connection failed[IP:169.254.89.%d, SocketID:%d ]\n", activeBoards[b],70+activeBoards[b],r);
			joint[b].isConnected=0;
			Sleep(5000);
			exit(0);
		}
}
	printf("\n");
}

/******************************************/
void startPositionControl()
/******************************************/
{
for(b=FIRST_LBODY_BOARD;b<LAST_LBODY_BOARD;b++)
	if(activeBoards[b]>0){
		r=StartBoardPositionControl(activeBoards[b]);
		if(r!=-1)
			printf("Board %d :Start Position Control succeded, %d\n",activeBoards[b],r);
		else
			printf("Board %d :Start Position Control failed! Error:%d\n",activeBoards[b],r);
	}
	printf("\n");
}

/******************************************/
void stopPositionControl()
/******************************************/
{
	r=StopPositionControl();
	if(r!=-1)
		printf("All Active Boards:Stop Position Control succeded\n");
	else
		printf("All Active Boards:Stop Position Control failed! Error:%d\n",r);
	printf("\n");
}

/******************************************/


/******************************************/
void startTorqueControl(int BoardNumber, int torqueFlag)
{
int B;
int t;
B=BoardNumber;
t=torqueFlag;

		
		r=SetTorqueOnOff(B,t);

		if(r!=-1)
			printf("Board %d :Start Position Control succeded, %d\n",activeBoards[b],r);
		else
			printf("Board %d :Start Position Control failed! Error:%d\n",activeBoards[b],r);
	
	printf("\n");
}

/******************************************/
void moveToHome()
/******************************************/
{

		desPos[0]=160000;
		desVel[0]=300;

		desPos[1]=21000;
		desVel[1]=400;

		r=SetDesiredVelocity(desVel);
		if(r!=-1) printf("Set Desired Velocity succeded\n");
		else printf("Set Desired Velocity failed! Error:%d\n",r);

		r=SetDesiredPosition(desPos);
		if(r!=-1) printf("Set Desired Position succeded\n");
		else printf("Set Desired Position failed! Error:%d\n",r);

		printf("\n");  
	
	
}



/******************************************/
//void setBroadCastRate(int boardNumber, int BCastRate, char TwinSatus)
void setBroadCastRate(int boardNumber, int BCastRate)
/******************************************/
{
	r=SetBCastRate(activeBoards[boardNumber], 2*BCastRate);  // 1=0.5msec 
	//r=SetBCastRate(activeBoards[boardNumber], 2*BCastRate, TwinSatus);  // 1=0.5msec 
	if(r!=-1)
		printf("Board %d :Broadcast request sent succed, Rate:%dHz",activeBoards[boardNumber],(int)(1000.0/(float)BCastRate));
	else
		printf("Board %d :Broadcast request sent failed!, Rate:%dHz",activeBoards[boardNumber],(int)(1000.0/(float)BCastRate));
}


/******************************************/
void setBroadCastPolicy(int boardNumber, int BCastPolicy)
/******************************************/
{
	r=SetBCastPolicy(activeBoards[boardNumber], BCastPolicy);
	if(r!=-1)
		printf("Board %d :Set Broad Cast Policy succeded\n",activeBoards[boardNumber]);
	else
		printf("Board %d :Get Broad Cast Policy failed! Error:%d\n",activeBoards[boardNumber],r);
}


/******************************************/
//void setExtraBroadCastPolicy(int boardNumber)
///******************************************/
//{
//	r=SetExtraBCastPolicy(activeBoards[boardNumber]);
//	if(r!=-1)
//		printf("Board %d :Set Broad Cast Policy succeded\n",activeBoards[boardNumber]);
//	else
//		printf("Board %d :Get Broad Cast Policy failed! Error:%d\n",activeBoards[boardNumber],r);
//}


/******************************************/
void stopBroadCast(int BoardNumber)
/******************************************/
{
	r=StopBCast(activeBoards[BoardNumber]);
	if(r!=-1)
		printf("Board %d :Stop Broadcast request sent succed\n",activeBoards[BoardNumber]);
	else
		printf("Board %d :Stop Broadcast request sent failed!\n",activeBoards[BoardNumber]);
}


/******************************************/
//int getBroadCastData()
//{
//	int bytesReceived=0;
//	int boardNo;
//	int pktRead=0;
//
//	for(int pkt=0;pkt<noActiveBoards+1;pkt++)
//		if((bytesReceived=GetBCastData(packetToReceive))>0)
//		{
//			if(packetToReceive.content[2] == (char)0xbb)   // this is a boradcast packet
//			{
//				boardNo=packetToReceive.content[3];
//				
//				joint[boardNo-1].pos=(unsigned char)packetToReceive.content[4]+((packetToReceive.content[5]<<8)& 0xff00) 
//								  +((packetToReceive.content[6]<<16)& 0xff0000)+((packetToReceive.content[7]<<24)& 0xff000000);
//			
//				if(((packetToReceive.content[9])& 0x80)>0) joint[boardNo-1].vel= ((unsigned char)packetToReceive.content[8]+((packetToReceive.content[9]<<8)& 0xff00))-0xffff;
//			 else joint[boardNo-1].vel= ((unsigned char)packetToReceive.content[8]+((packetToReceive.content[9]<<8)& 0xff00));
//			/*
//				joint[boardNo-1].vel= packetToReceive.content[8]+(packetToReceive.content[9]*256);
//*/
//
//			
//			/*	if(((packetToReceive.content[11])& 0x80)>0) joint[boardNo-1].tor= ((unsigned char)packetToReceive.content[10]+((packetToReceive.content[11]<<8)& 0xff00))-0xffff;
//			 else joint[boardNo-1].tor= ((unsigned char)packetToReceive.content[10]+((packetToReceive.content[11]<<8)& 0xff00));
// */			
//			 joint[boardNo-1].tor= packetToReceive.content[10]+(packetToReceive.content[11]*256);
//
//
//			 if(((packetToReceive.content[13])& 0x80)>0) joint[boardNo-1].pidOutput= ((unsigned char)packetToReceive.content[12]+((packetToReceive.content[13]<<8)& 0xff00))-0xffff;
//			 else joint[boardNo-1].pidOutput= ((unsigned char)packetToReceive.content[12]+((packetToReceive.content[13]<<8)& 0xff00));
//			 
//			 //joint[boardNo-1].pidOutput=(unsigned char)packetToReceive.content[12]+((packetToReceive.content[13]<<8)& 0xff00); 
//								  
//
//				joint[boardNo-1].pidError=(unsigned char)packetToReceive.content[14]+((packetToReceive.content[15]<<8)& 0xff00) 
//								  +((packetToReceive.content[16]<<16)& 0xff0000)+((packetToReceive.content[17]<<24)& 0xff000000);
//
//			joint[boardNo-1].absPos1=(unsigned char)packetToReceive.content[30]+((packetToReceive.content[31]<<8)& 0xff00); 
//			
//		/*		joint[boardNo-1].trajDSP=(unsigned char)packetToReceive.content[16]+((packetToReceive.content[17]<<8)& 0xff00)
//										 +((packetToReceive.content[18]<<16)& 0xff0000)+((packetToReceive.content[19]<<24)& 0xff000000);
//
//
//				joint[boardNo-1].temp_Vdc=(unsigned char)packetToReceive.content[20]+((packetToReceive.content[21]<<8)& 0xff00) 
//								  +((packetToReceive.content[22]<<16)& 0xff0000)+((packetToReceive.content[23]<<24)& 0xff000000);
//
//				joint[boardNo-1].tStamp=(unsigned char)packetToReceive.content[24]+((packetToReceive.content[25]<<8)& 0xff00) 
//								  +((packetToReceive.content[26]<<16)& 0xff0000)+((packetToReceive.content[27]<<24)& 0xff000000);
//
//				joint[boardNo-1].absPos1=(unsigned char)packetToReceive.content[28]+((packetToReceive.content[29]<<8)& 0xff00); 
//			//	joint[boardNo-1].realCurrent=(unsigned char)packetToReceive.content[30]+((packetToReceive.content[31]<<8)& 0xff00);
//				joint[boardNo-1].realCurrent=(unsigned char)packetToReceive.content[30];*/
//			//	joint[boardNo-1].absPos2=(unsigned char)packetToReceive.content[32]+((packetToReceive.content[33]<<8)& 0xff00);  
//		
//			}
//			bytesReceived=0;
//		}	
		
//	
//		
//	
//	if((desTor& 0x8000)>0) desTor=desTor -0xffff;
//			 
//
//		gotoxy(1,4); printf("Stiffness motor:");
//		gotoxy(1,5); printf("DesPos=%d p=%d, vel=%d, tor=%d , linkdeflection=%d          ", desPos[0], joint[0].pos,joint[0].vel,joint[0].tor, joint[0].absPos1);
//		
//
//		gotoxy(1,7); printf("Joint Motor:");    
//		gotoxy(1,8); printf("DesPos=%d, p=%d, vel=%d, tor=%d \n  desTor=%d, pidE=%d, pidO=%d  \n   posG(%d,%d,%d),   TorG(%d,%d,%d)     ",desPos[0],joint[0].pos,joint[0].vel,joint[0].tor,desTor, joint[0].pidError, joint[0].pidOutput, posPID[0],posPID[1],posPID[2],torPID[0],torPID[1],torPID[2]);
//	
//		return 0;
//}
//



/******************************************/
void saveData()
/******************************************/
{
	FILE *fp;
	char filename[256];
	cout<<"enter file name with extension (.txt) \n";
			//////if (_getch()==13)
			//////	fp=fopen("adata.txt","w");
			//////else
			//////{	
			//////	cin>>filename;	
			//////	fp=fopen(filename,"w");
			//////}

	//fp=fopen("FTcalibrationData.txt","w");
	//		//fp=fopen("adata.txt","w");
	//		//fprintf(fp, "time, M1Despos, M1RealPos, M1vel, M1tor, M2Despos, M2RealPos, M2vel, M2tor\n");
	//for(int d=0;d<dataCounter;d++)
	//		//fprintf(fp, "%.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f\n",sT[d], data[d][1], data[d][2], data[d][3], data[d][4], data[d][5], 
	//		//data[d][6], data[d][7], data[d][8]);
	//		//fprintf(fp,"%.4f \t %.4f \t %.4f\t %.4f\t %10.4f\t %10.4f\t  %10.4f\t %10.4f\t \n",sT[d], data[d][2],data[d][3],data[d][4],data[d][7], data[d][9], data[d][10], data[d][11]); //added by Yogesh	
	//		//fprintf(fp,"%.4f \t %.4f \t %.4f\t  \n",sT[d], data[d][2],data[d][3]); //added by Yogesh	
	//		// to print Fz, Tz, Analog i/p 1 and 2	
	//		//fprintf(fp,"%.4f \t %.4f \t %.4f \t %.1f \t %.1f\t \n",sT[d], data[d][8],data[d][9],data[d][10],data[d][11]); //added by Yogesh	
	//		//fprintf(fp,"%.4f \t %.4f \t %.4f \t %.1f \t %.1f\t \n",sT[d], data[d][2],data[d][9],data[d][10],data[d][11]); //time,position,force(counts),force(N),torque //added by Yogesh	
	//		//fprintf(fp,"%.4f \t %.4f \t %.4f \t  %.4f \t %.1f \t %d \t %d \t %d \t \n",sT[d], data[d][2],data[d][3],data[d][9],data[d][10], data[d][5], data[d][6], data[d][7]); //time,damper motor position,main motor position, force(counts),force(N) //added by Yogesh	

	//		//11/3/2013fprintf(fp,"%.4f \t %.4f \t %.4f \t  %.4f \t %.1f \t %d \t %d \t %d \t \n",sT[d], data[d][2],data[d][3],data[d][9],data[d][10], accn[d][0], accn[d][1], accn[d][2]); //time,damper motor position,main motor position, force(counts),force(N), accns //added by Yogesh on 13-2-13		
	//fclose(fp);
	
	//
	////FT sensor Calibration using ATI in the assembly.  No spring element in this case
	//fp=fopen("StiffnessData.txt","w");
	//for(int d=0;d<dataCounter;d++)
	//fprintf(fp,"%.4f \t %d \t %d \t %d \t %d \t %.4f \t %.4f \t %.4f \t %.4f \t %.4f \t %.4f\n",sT[d], data[d][0],data[d][1],data[d][2],data[d][3],FT_data[d][0],FT_data[d][1],FT_data[d][2],FT_data[d][3],FT_data[d][4],FT_data[d][5]);//FT sensor Calibration data
	//fclose(fp);

	//Stiffness of the system with spring element in the assembly
	fp=fopen("StiffnessData.txt","w");
	for(int d=0;d<dataCounter;d++)
	fprintf(fp,"%d \t %d \t %.4f \t %.4f\n", data[d][0],data[d][1],FT_data[d][0],FT_data[d][1]);  //PositionMot1, PositionMot2, Force, torque
	fclose(fp);
}

  void ClearScreen()
    {
    int n;
    for (n = 0; n < 100; n++)
	{gotoxy(1,n); 
	 printf( "                                                                                                           " );
	}
	gotoxy(1,1);
    }
  //---------------------------------------------------------------------------
  void close(void)
  {
	  // stop the simulation
	  simulationRunning = false;

	  PrintClock->stop();
	  mainClock->stop();
	  // wait for graphics and haptics loops to terminate
	  //while (!simulationFinished) { cSleepMs(100); }

	 

  }

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int _tmain(int argc, _TCHAR* argv[])
{
	// simulation in now running
	simulationRunning = true;

	PrintClock = new cPrecisionClock();
	mainClock = new cPrecisionClock();

	int res;
	int mytemp;
	/*char filename[255];
	ifstream infile;
	cout<<"please enter filename with .txt:";
	cin.getline(filename,255);*/

	int controlIsOn=0;
	pid_off[0]=0;
	bool start=false;
	starTime = 0;

	// init the comm library
	InitLibrary(MAX_BOARDS);
	// reset all possible active sockets
	CloseSockets();	
	// Open UDP connection 
	res=connectUDP();
	// scan for motor controllers

	if((res=scanSystem())<0)
	{
		Sleep(2000);
		exit(0);
	}
	
	// connect to motor controllers
	
	connectTCP();

	// load joint data (limits, pid gains etc)
	// loadJointDataFromDrivers();

	// initialize joint variables (calibration etc)
	// initJoints();
	
	// load the trajectory from file(this is for ALex)
	// importTrajectory();

	//for(int b=FIRST_LBODY_BOARD;b<LAST_LBODY_BOARD;b++){
	//	setBroadCastRate(b,BCAST_RATE*1000);    //1 = 0.0005sec
	//	setBroadCastPolicy(b,BCAST_POLICY);    
	//} 

		setBroadCastRate(1,BCAST_RATE);    //Main Motor Board
		setBroadCastPolicy(1,BCAST_POLICY); 

		//setExtraBroadCastPolicy(1,EXTRA_BCAST_POLICY);

		setBroadCastRate(0,BCAST_RATE);    //Damper Motor Board
		setBroadCastPolicy(0,BCAST_POLICY); 

	//setExtraBroadCastPolicy(0,EXTRA_BCAST_POLICY);

	//-------------------------startPositionControl();
	//----------------------------moveToHome();
	
	//desPos[0]=10000;
	//r=SetDesiredPosition(desPos);
	// wait for few secs
	
		int mtype = SetMotorType(1, 0x03);	
	
	Sleep(1000);

	SetAnalogInputs(1, 6);
//	system("cls");

	desVel[0]=2000;
	desVel[1]=5000;
	r=SetDesiredVelocity(desVel);
	//stopPositionControl();
	
	// get simulation run time
	startSimTime = timeObject.getSysTime();
	// run until the defined sim time
	int count=0;

	//Setup the ATI sensor
	devMeasurementFns myUDP; //9/3/2013  		
	UDP_Communication myUDP_1; //9/3/2013  		
	myUDP_1.InitializeWinSock(); //9/3/2013  	

int bytesReceived=0;
	int boardNo;
	int pktRead=0;

for(int pkt=0;pkt<noActiveBoards+1;pkt++)
		if((bytesReceived=GetBCastData(packetToReceive))>0)
		{
			if(packetToReceive.content[2] == (char)0xbb)   // this is a boradcast packet
			{
				boardNo=packetToReceive.content[3];
				
				joint[boardNo-1].pos=(unsigned char)packetToReceive.content[4]+((packetToReceive.content[5]<<8)& 0xff00) 
								  +((packetToReceive.content[6]<<16)& 0xff0000)+((packetToReceive.content[7]<<24)& 0xff000000);
				
			if (((packetToReceive.content[17])& 0x80)>0) joint[boardNo-1].realCurrent=(unsigned char)packetToReceive.content[14]+((packetToReceive.content[15]<<8)& 0xff00) 
								  +((packetToReceive.content[16]<<16)& 0xff0000)+((packetToReceive.content[17]<<24)& 0xff000000)-0xffffffff;
			else
					joint[boardNo-1].realCurrent=(unsigned char)packetToReceive.content[14]+((packetToReceive.content[15]<<8)& 0xff00) 
								  +((packetToReceive.content[16]<<16)& 0xff0000)+((packetToReceive.content[17]<<24)& 0xff000000);
				
			
				/*if(((packetToReceive.content[33])& 0x80)>0) joint[boardNo-1].realCurrent= ((unsigned char)packetToReceive.content[32]+((packetToReceive.content[33]<<8)& 0xff00))-0xffff;
			 else joint[boardNo-1].realCurrent=(unsigned char)packetToReceive.content[32]+((packetToReceive.content[33]<<8)& 0xff00);
				if(((packetToReceive.content[33])& 0x80)>0) 
					joint[boardNo-1].realCurrent= ((unsigned char)packetToReceive.content[32]+((packetToReceive.content[33]<<8)& 0xff00))-0xffff;*/

			 if(((packetToReceive.content[9])& 0x80)>0) joint[boardNo-1].vel= ((unsigned char)packetToReceive.content[8]+((packetToReceive.content[9]<<8)& 0xff00))-0xffff;
			 else joint[boardNo-1].vel= ((unsigned char)packetToReceive.content[8]+((packetToReceive.content[9]<<8)& 0xff00));
			
		/*	 joint[boardNo-1].tor= packetToReceive.content[10]+(packetToReceive.content[11]*256);


			 if(((packetToReceive.content[13])& 0x80)>0) joint[boardNo-1].pidOutput= ((unsigned char)packetToReceive.content[12]+((packetToReceive.content[13]<<8)& 0xff00))-0xffff;
			 else joint[boardNo-1].pidOutput= ((unsigned char)packetToReceive.content[12]+((packetToReceive.content[13]<<8)& 0xff00));
			 
			
				joint[boardNo-1].pidError=(unsigned char)packetToReceive.content[14]+((packetToReceive.content[15]<<8)& 0xff00) 
								  +((packetToReceive.content[16]<<16)& 0xff0000)+((packetToReceive.content[17]<<24)& 0xff000000);*/

		joint[boardNo-1].absPos1=(unsigned char)packetToReceive.content[30]+((packetToReceive.content[31]<<8)& 0xff00) 
				+((packetToReceive.content[32]<<16)& 0xff0000)+((packetToReceive.content[33]<<24)& 0xff000000); 
			
				
			}
			bytesReceived=0;
		}	






	/*PosOff[0]=joint[0].pos;
			gains[0]=0;
			gains[1]=0;
			gains[2]=0;
		sprintf_s(pidGains, sizeof(pidGains)/sizeof(char), "%d,%d,%d", gains[0],gains[1],gains[2]); 
		SetPidGains(2,1, pidGains);	*/
	
		ClearScreen();

		//declaration of Thread
		cThread* mainThread = new cThread();
		mainThread->set(updateLoop,CHAI_THREAD_PRIORITY_HAPTICS);

	while(curSimTime<SIMULATION_TIME && !stopSim)
	{
		
		if (sinewaveON==ON)
			{
//			rampON=OFF;
			desPos[0]=(int)(157000*(1+sin(1.5*sineTime*PI*curSimTime)))+314000+FunctionOffset;
			r=SetDesiredPosition(desPos);
			}
		
		if (rampON==ON)
		{
			//sinewaveON=OFF;
			if (rampDirection==FORWARD)
			{	
				
				rampPos+=rampSTEP;
				//				desPos[0]-=rampSTEP+FunctionOffset;
				desPos[0]=rampPos+FunctionOffset;

				//if((desPos[0]+(rampPos+rampSTEP)+FunctionOffset)<posMAX)
				if(desPos[0]<posMAX)
				{
				/*desPos[0]+=rampSTEP;
				r=SetDesiredPosition(desPos);
				*/
				r=SetDesiredPosition(desPos);
				}
				else
				rampDirection=REVERSE;
			}
			else 
			{		
				rampPos-=rampSTEP;
				//				desPos[0]-=rampSTEP+FunctionOffset;
				desPos[0]=rampPos+FunctionOffset;

				//if((desPos[0]-(rampPos-rampSTEP)+FunctionOffset)>posMIN)
				if(desPos[0]>posMIN)
				{					
				r=SetDesiredPosition(desPos);
				}
				else
				rampDirection=FORWARD;
			}
		}

		// get the time at thr start of the loop
		startLoopTime = timeObject.getSysTime();
		curSimTime=timeObject.getSysTime()-startSimTime;

		sT[loop]=curSimTime;

		if(_kbhit())
			ch = _getch();
		switch (ch)
		{
			case 'q':
				stopSim=1;
			break;

			case 'w':
				GetPidGains(0,1,gains);
				gotoxy(1,2); printf("p=%d, i=%d, d=%d,\n",gains[0],gains[1],gains[2]);
				GetPidGains(1,1,gains);
				gotoxy(1,3); printf("p=%d, i=%d, d=%d,\n",gains[0],gains[1],gains[2]);
			break;

			case 't'://sinewave reference
				//DisablePwmPad(0);//stopPositionControl();
				//DisablePwmPad(1);
				rampON=OFF;
				if (sinewaveON==OFF)
				sinewaveON=ON;
				else
				sinewaveON=OFF;
			break;

			case 'c'://sinewave reference
				//DisablePwmPad(0);//stopPositionControl();
				//DisablePwmPad(1);
				if (sinewaveON==ON)
				{
					if (sineTime<10)
						sineTime+=0.5;
					//else if (sineTime>0)
					//	sineTime-=0.5;
				}
				else if (rampON==ON)
				{
					if ( rampSTEP<2000)
						rampSTEP+=100;
					//else if (rampSTEP>100)
					//	rampSTEP-=100;
				}	
			break;

			case 'v'://sinewave reference
				//DisablePwmPad(0);//stopPositionControl();
				//DisablePwmPad(1);
				if (sinewaveON==ON)
				{
					if (sineTime>0)
					sineTime-=0.5;
				}
				else if (rampON==ON)
				{					
					if (rampSTEP>100)
					rampSTEP-=100;
				}	
			break;

			case 'r'://Ramp reference

				sinewaveON=OFF;
				if (rampON==OFF)
				rampON=ON;
				else
				rampON=OFF;
				//desPos[1]=joint[1].pos-15000;
				//desPos[0]=joint[0].pos-15000;
			break;

			case 'a':
				desPos[1]+=157000; 
				r=SetDesiredPosition(desPos);
				//desPos[0]+=1000;
				//if(desPos[1]>800000)desPos[1]=800000;
				
				//desPos[0]+=1000;
				//if(desPos[0]>800000)desPos[0]=800000;
				break;

			case 'z':
				desPos[1]-=157000;
				//desPos[0]-=1000;
				r=SetDesiredPosition(desPos);
				//if(desPos[1]<0)desPos[1]=0;
				//desPos[0g]-=15000;
			break;

			case 's':
				if(desPos[0]<1200000)
				{
					if (sinewaveON==ON)
					{
						if(FunctionOffset<(700000-5000))
							FunctionOffset+=5000;
					}
					else if (rampON==ON)
					{
						if(FunctionOffset<(700000-5000))
							FunctionOffset+=5000;
					}					
					else
					{
						desPos[0]+=10000;
						r=SetDesiredPosition(desPos);
					}
					
				}
				//if(desPos[1]>26000)desPos[1]=26000;
			break;

			case 'x':

				if(desPos[0]>=10000)
				{
					if (sinewaveON==ON)
					{
						if(FunctionOffset>(-157000+5000))
						FunctionOffset-=5000;
					}
					else if (rampON==ON)
					{
						if(FunctionOffset>(-157000+5000))
							FunctionOffset-=5000;
					}	
					else if (sinewaveON==OFF)
					{
					desPos[0]-=10000;
					r=SetDesiredPosition(desPos);
					}
				}
			break;
			case 'd':
				controlIsOn=1;
				startTorqueControl(1,1);
			break;

			case 'e':
				if (storeData==ON)
					storeData=OFF;
				else
					storeData=ON;
					deleteData=OFF;
			break;

			case 5://Press CTRL+'e'to delete stored values in data and FT_data
				storeData=OFF;
				deleteData=ON;
				//gotoxy(1,4);
			//	printf("\n ----------------------- Deleting data --------------------------\n\n\n");
				for(int d=0;d<dataCounter;d++)
				{
					for(int i=0;i<12;i++)
						data[d][i]=0;
					for(int i=0;i<6;i++)
						FT_data[d][i]=0;  //PositionMot1, PositionMot2, Force, torque
					
					sT[d]=0;
				}
				dataCounter=0;
				//gotoxy(1,4);
				//printf("\n --------------------------------------------------------------------\n\n\n");

			break;

			case 'f':
				controlIsOn=1;
				startTorqueControl(1,0);
				startPositionControl();	
			//	startTorqueControl(1,1);
				printf("\n \n  f-key is pressed \n ");
			break;

			case 'g':
				controlIsOn=1;
				startPositionControl();
//				startTorqueControl(1,1);
				printf("\n \n  g-key is pressed \n ");
			break;
			
			case 'h':
				controlIsOn=0;
				stopPositionControl();
				startTorqueControl(1,0);
			break;
		
			case 'o':
				// this button sets offsets for no contact force and position
				//desPos[0]=joint[0].pos;
				mytemp=desPos[0];
				desPos[0]=1;				
				r=SetDesiredPosition(desPos);
				printf("\n Reset position and force \n");
				desPos[0]=mytemp;
				//desPos[0]=0;
				//Destor[0]=joint[0].anInput1;
				//r=SetDesiredTorque(Destor);				
			break;
			case 'i':
				// this button resets the minimum and contact positions as part of a sequence button press:
				//first press 'o' wait then press 'i'
				//sets also offsets for no contact force and position
				mytemp=desPos[0];
				desPos[0]=2;				
				r=SetDesiredPosition(desPos);
				printf("\n Reset position and force \n");
				desPos[0]=mytemp;
				//printf("\n Desired position activated \n");
				//desPos[0]=0;
				break;

			//case 'p':// toggle printing on scree
			//	gains[1]=0;
			//	gains[2]=0;
			//	gains[0]=0;
			//	setControllerGains();
			//
			//	DesOffset[0] = 10000;
			//	SetPidOffset(DesOffset);
			//	
			//break;
			case 'p':// toggle printing on scree
					if (TogglePrint>0) TogglePrint=0;
					else TogglePrint=1;	
					ClearScreen();
			break;

			default:
			break;
		}
		
		ch=0;
		feq = 1/ts;
	
		//gotoxy(1,1);printf("t=%.3f  ts:%.12f Feq:%.4f\n",curSimTime,ts,feq);
						
	//	desPos[0]=150000.0 + 50000.0*sin(curSimTime*3.0);
	//	desPos[1]=20000.0 + 5000.0*sin(curSimTime*2.09);
	
		//t1 = timeObject.getSysTime();
	
		//for(int i=0;i<MAX_BOARDS;i++)
		//	desPos[i]=desPos[i]+PosOff[i];
		
	//r=SetDesiredPosition(desPos);
		//r=SetDesiredTorque(Destor);
		
		//t2 = timeObject.getSysTime();
		//timeudp = t2-t1;
		//printf("\n time for r=SetDesiredPosition = %.8f \n", timeudp);

		//t1 = timeObject.getSysTime();
		//getBroadCastData();
		
	int bytesReceived=0;
	int boardNo;
	int pktRead=0;

	for(int pkt=0;pkt<noActiveBoards+1;pkt++)
		if((bytesReceived=GetBCastData(packetToReceive))>0)
		{
			if(packetToReceive.content[2] == (char)0xbb)   // this is a boradcast packet
			{
				boardNo=packetToReceive.content[3];
				
				joint[boardNo-1].pos=(unsigned char)packetToReceive.content[4]+((packetToReceive.content[5]<<8)& 0xff00) 
								  +((packetToReceive.content[6]<<16)& 0xff0000)+((packetToReceive.content[7]<<24)& 0xff000000);
				//joint[boardNo-1].realCurrent=(unsigned char)packetToReceive.content[14]+((packetToReceive.content[15]<<8)& 0xff00) 
				//				  +((packetToReceive.content[16]<<16)& 0xff0000)+((packetToReceive.content[17]<<24)& 0xff000000);;
				//joint[boardNo-1].realCurrent=(unsigned char)packetToReceive.content[32]+((packetToReceive.content[33]<<8)& 0xff00);
			
				if (((packetToReceive.content[17])& 0x80)>0) 
					joint[boardNo-1].realCurrent=(unsigned char)packetToReceive.content[14]+((packetToReceive.content[15]<<8)& 0xff00) 
								  +((packetToReceive.content[16]<<16)& 0xff0000)+((packetToReceive.content[17]<<24)& 0xff000000)-0xffff;
			else
					joint[boardNo-1].realCurrent=(unsigned char)packetToReceive.content[14]+((packetToReceive.content[15]<<8)& 0xff00) 
								  +((packetToReceive.content[16]<<16)& 0xff0000)+((packetToReceive.content[17]<<24)& 0xff000000);
				
				if(((packetToReceive.content[9])& 0x80)>0) joint[boardNo-1].vel= ((unsigned char)packetToReceive.content[8]+((packetToReceive.content[9]<<8)& 0xff00))-0xffff;
			 else joint[boardNo-1].vel= ((unsigned char)packetToReceive.content[8]+((packetToReceive.content[9]<<8)& 0xff00));
			
			 joint[boardNo-1].tor= packetToReceive.content[10]+(packetToReceive.content[11]*256);


	/*		 if(((packetToReceive.content[13])& 0x80)>0) joint[boardNo-1].pidOutput= ((unsigned char)packetToReceive.content[12]+((packetToReceive.content[13]<<8)& 0xff00))-0xffff;
			 else joint[boardNo-1].pidOutput= ((unsigned char)packetToReceive.content[12]+((packetToReceive.content[13]<<8)& 0xff00));
			 
			joint[boardNo-1].pidError=(unsigned char)packetToReceive.content[14]+((packetToReceive.content[15]<<8)& 0xff00) 
								  +((packetToReceive.content[16]<<16)& 0xff0000)+((packetToReceive.content[17]<<24)& 0xff000000);*/

			 joint[boardNo-1].anInput1= ((unsigned char)packetToReceive.content[30]+((packetToReceive.content[31]<<8)& 0xff00));
			 //if(((packetToReceive.content[31])& 0x80)>0) joint[boardNo-1].anInput1= ((unsigned char)packetToReceive.content[30]+((packetToReceive.content[31]<<8)& 0xff00))-0xffff;
			 //else joint[boardNo-1].anInput1= ((unsigned char)packetToReceive.content[30]+((packetToReceive.content[31]<<8)& 0xff00));

			 joint[boardNo-1].anInput2= ((unsigned char)packetToReceive.content[32]+((packetToReceive.content[33]<<8)& 0xff00));
			//if(((packetToReceive.content[33])& 0x80)>0) joint[boardNo-1].anInput2= ((unsigned char)packetToReceive.content[32]+((packetToReceive.content[33]<<8)& 0xff00))-0xffff;
			//else joint[boardNo-1].anInput2= ((unsigned char)packetToReceive.content[32]+((packetToReceive.content[33]<<8)& 0xff00));


			// anInput3 and anInput4 seceive force and torque in decaN and mNm 
			 //for fitting  positive and negative forces. Force up to 327.67N with a resolution of 10mN. Torque up to 32.767Nm with a resolution of 1mNm.
			 if(((packetToReceive.content[35])& 0x80)>0) joint[boardNo-1].anInput3= ((unsigned char)packetToReceive.content[34]+((packetToReceive.content[35]<<8)& 0xff00))-0xffff;
			 else joint[boardNo-1].anInput3= ((unsigned char)packetToReceive.content[34]+((packetToReceive.content[35]<<8)& 0xff00));
			joint[boardNo-1].Force=joint[boardNo-1].anInput3/100.0;// convert to Newtons from decaNewtons

			// decaN and mNm for fitting  positive and negative forces up to 327.67N with a resolution of 10mN.
			 if(((packetToReceive.content[37])& 0x80)>0) joint[boardNo-1].anInput4= ((unsigned char)packetToReceive.content[36]+((packetToReceive.content[37]<<8)& 0xff00))-0xffff;
			 else joint[boardNo-1].anInput4= ((unsigned char)packetToReceive.content[36]+((packetToReceive.content[37]<<8)& 0xff00));
			 joint[boardNo-1].Torque=joint[boardNo-1].anInput4/1000.0;// convert to Nm from mNm


			 // if(((packetToReceive.content[31])& 0x80)>0) joint[boardNo-1].anInput1= ((unsigned char)packetToReceive.content[30]+((packetToReceive.content[31]<<8)& 0xff00))-0xffff;
			 // else joint[boardNo-1].anInput1= ((unsigned char)packetToReceive.content[30]+((packetToReceive.content[31]<<8)& 0xff00));

			 // if(((packetToReceive.content[33])& 0x80)>0) joint[boardNo-1].anInput2= ((unsigned char)packetToReceive.content[32]+((packetToReceive.content[33]<<8)& 0xff00))-0xffff;
			 // else joint[boardNo-1].anInput2= ((unsigned char)packetToReceive.content[32]+((packetToReceive.content[33]<<8)& 0xff00));

			/*joint[boardNo-1].absPos1=(unsigned char)packetToReceive.content[30]+((packetToReceive.content[31]<<8)& 0xff00) 
				+((packetToReceive.content[32]<<16)& 0xff0000)+((packetToReceive.content[33]<<24)& 0xff000000); */
			 if(((packetToReceive.content[49])& 0x80)>0) joint[boardNo-1].Xaccleration = ((unsigned char)packetToReceive.content[48]+((packetToReceive.content[49]<<8)& 0xff00))-0xffff;
			 else joint[boardNo-1].Xaccleration = ((unsigned char)packetToReceive.content[48]+((packetToReceive.content[49]<<8)& 0xff00));

			 if(((packetToReceive.content[51])& 0x80)>0) joint[boardNo-1].Yaccleration = ((unsigned char)packetToReceive.content[50]+((packetToReceive.content[51]<<8)& 0xff00))-0xffff;
			 else joint[boardNo-1].Yaccleration = ((unsigned char)packetToReceive.content[50]+((packetToReceive.content[51]<<8)& 0xff00));

			  if(((packetToReceive.content[53])& 0x80)>0) joint[boardNo-1].Zaccleration = ((unsigned char)packetToReceive.content[52]+((packetToReceive.content[53]<<8)& 0xff00))-0xffff;
			 else joint[boardNo-1].Zaccleration = ((unsigned char)packetToReceive.content[52]+((packetToReceive.content[53]<<8)& 0xff00));
				
			 joint[boardNo-1].TwinActPos= (unsigned char)packetToReceive.content[38]+((packetToReceive.content[39]<<8)& 0xff00) 
								  +((packetToReceive.content[40]<<16)& 0xff0000)+((packetToReceive.content[41]<<24)& 0xff000000);
			 joint[boardNo-1].TwinTargPos= (unsigned char)packetToReceive.content[42]+((packetToReceive.content[43]<<8)& 0xff00) 
								  +((packetToReceive.content[44]<<16)& 0xff0000)+((packetToReceive.content[45]<<24)& 0xff000000);
			 joint[boardNo-1].TwinVel= (unsigned char)packetToReceive.content[46]+((packetToReceive.content[47]<<8)& 0xff00);

			}
			bytesReceived=0;
		}	
		if((desTor& 0x8000)>0) desTor=desTor -0xffff;
		
		
		
		//---------------Calculation of force from the Calibration matrix------------------------------------------------------
		
		//The next two lines use the calibration matris to generate the forcein N and the torque in Nm from the raw data in counts
		//works only when the incoming values are in counts.  the firmware is now changed for anInput3 and anInput4 to send
		// decaN and mNm for fitting  positive and negative forces up to 327.67N with a resolution of 10mN.
		Force= joint[0].anInput3*Cmatrix[0][0]+joint[0].anInput4*Cmatrix[1][0];
		Torque= joint[0].anInput3*Cmatrix[0][1]+joint[0].anInput4*Cmatrix[1][1];
		
		//---------------END of Calculation of force from the Calibration matrix-------------------------------------------------



		//data[loop][5] = 0.015*(float)(desPos[1]-10000.0)/16000.0;
		////data[loop][6] = 0.015*(float)(joint[1].pos-10000.0)/16000.0;
		//data[loop][7] = (float)joint[10].realCurrent;
		//data[loop][8] = (float)joint[1].tor;
		//data[loop][9] = (float)joint[1].absPos1;
		//data[loop][10] = ts;
		//
		////posi =((float)joint[0].absPos1/54800)*2*PI;
		//
		//data[loop][5]= 223.8*sin(((float)joint[1].pos/54800)*2*PI);//Height mm

		//if (data[loop][5]-data[loop][6]<0)
		//	data[loop][6]=data[loop][5];
		
// Commented for voltage control
// position control was successful with following commented code
	/*gains[1]=0;
	gains[2]=0;
		if (desPos[1]-joint[1].pos<0)
			gains[0]=(joint[1].pos-desPos[1])*600;
		else
			gains[0]=0;*/
	
	//if (controlIsOn==1)
		//setControllerGains();
		

//*******************************************************
		/*		gains[1]=0;
				gains[2]=0;
				gains[0]=0;
				setControllerGains();
			
				DesOffset[0] = 10000;
				SetPidOffset(DesOffset);*/
//*******************************************************

/////////////////////VOLTAGE CONTROL///////////////	
		
	//if (desPos[1]-joint[1].pos<0)
	//	{
	//		DesOffset[1]= 2*(desPos[1]-joint[1].pos);
	//		//SetPidOffset(DesOffset);
	//		/*
	//		if (DesOffset[1]<24001)
	//			SetPidOffset(DesOffset);
	//		else{
	//			DesOffset[1]= 0;
	//			SetPidOffset(0);	
	//		}	*/
	//	}
	//	else{
	//	DesOffset[1]= 0;
	//	//SetPidOffset(DesOffset);
	//	//SetPidOffset(DesOffset);
	//	}
		//       
//	if (DesOffset[1]>0) DesOffset[1]= 0;

//SetPidOffset(DesOffset);


//	gotoxy(1,21);printf("Voltage=%d\n",DesOffset[1]);

/////////////////////////////////////////////////////

		/*GetPidGains(2,1,gains);
		gotoxy(1,21); printf("p=%d, i=%d, d=%d \n",gains[0],gains[1],gains[2]);*/
		//data[loop][4] = pid_off[0];//desTor;
		//data[loop][5] = joint[0].pidError;
		//data[loop][6] = joint[0].pidOutput;
		//data[loop][7] = posPID[0];
		//data[loop][8] = posPID[1];
		//data[loop][9] = posPID[2];
		//data[loop][10] = torPID[0];
		//data[loop][11] = torPID[1];
		//data[loop][12] = torPID[2];

///////////////////////////////////////////////////////

		//desTor =  GetDesiredTorque(1);
	//	pidError = GetPidError(1);
	//	pidOutput = GetPidOutput(1);
		//	r=SetDesiredVelocity(desVel);
		//	r=SetDesiredPosition(desPos);
		
		// calculate the samplig interval 
	
		/*if (count++>50)
		{
		gotoxy(1,1);printf("t=%.3f  ts:%.12f Feq:%.4f\n",curSimTime,ts,feq);
		printf("\n\n Sensor data Fx=%10f Fy=%10f Fz=%10f \n", FTData[0], FTData[1], FTData[2]);
count=0;
		}*/	
		//t1 = timeObject.getSysTime();
		myUDP.ReadFTsensorData();
		myUDP.GetFTData(FTData);
		//t2 = timeObject.getSysTime();
		//timeudp = t2-t1;
		//printf("\n\n time for udp comm = %.8f \n", timeudp);




		//Force = (float)(offsetF-joint[0].anInput1)*gainC;
		//Force = (float)(joint[0].anInput3)*gainC;
		//Torque = gainB*(float)(offsetT-joint[0].anInput2)-(gainB*gainA*Force);
		//Torque =(float)(joint[0].anInput2);



		
////////************************************************  Store Data Yogesh**********************************************************************
//////
//////		data[loop][2] = joint[1].pos;
//////		data[loop][4] = (float)joint[1].tor;
//////		data[loop][2] = joint[0].pos;
//////		data[loop][3] = joint[0].TwinActPos;
////////***********Added on 13-2-13 for acceleration***********
//////		accn[loop][0] =  joint[0].Xaccleration;
//////		accn[loop][1] =  joint[0].Yaccleration;
//////		accn[loop][2] =  joint[0].Zaccleration;
//////		//data[loop][5] = (double)joint[0].Xaccleration;
//////		//data[loop][6] = (double)joint[0].Yaccleration;
//////		//data[loop][7] = (double)joint[0].Zaccleration;
////////*******************************************************	
		//data[loop][9] = FTData[0];
		//data[loop][10] = FTData[1];
//////		data[loop][9] = joint[0].anInput1;
//////		data[loop][10] = Force;
//////		data[loop][11] = Torque;
////////************************************************  Store Data **************************************************************************


		////////************************************************  Store Data ioannis 10/3/2013 **********************************************************************
//////
//////		data[loop][2] = joint[1].pos;
//////		data[loop][4] = (float)joint[1].tor;
//////		data[loop][2] = joint[0].pos;
//////		data[loop][3] = joint[0].TwinActPos;
////////***********Added on 13-2-13 for acceleration***********
//////		accn[loop][0] =  joint[0].Xaccleration;
//////		accn[loop][1] =  joint[0].Yaccleration;
//////		accn[loop][2] =  joint[0].Zaccleration;
//////		//data[loop][5] = (double)joint[0].Xaccleration;
//////		//data[loop][6] = (double)joint[0].Yaccleration;
//////		//data[loop][7] = (double)joint[0].Zaccleration;
////////*******************************************************	
		//data[loop][9] = FTData[0];
		//data[loop][10] = FTData[1];
//////		data[loop][9] = joint[0].anInput1;
//////		data[loop][10] = Force;
//////		data[loop][11] = Torque;
////////************************************************  Store Data **************************************************************************


		////////************************************************  Store Data Calibration ioannis 10/3/2013 **********************************************************************
//////
//////		data[loop][2] = joint[1].pos;
//////		data[loop][4] = (float)joint[1].tor;
//////		data[loop][2] = joint[0].pos;
//////		data[loop][3] = joint[0].TwinActPos;
////////***********Added on 13-2-13 for acceleration***********
//////		accn[loop][0] =  joint[0].Xaccleration;
//////		accn[loop][1] =  joint[0].Yaccleration;
//////		accn[loop][2] =  joint[0].Zaccleration;
//////		//data[loop][5] = (double)joint[0].Xaccleration;
//////		//data[loop][6] = (double)joint[0].Yaccleration;
//////		//data[loop][7] = (double)joint[0].Zaccleration;
////////*******************************************************	

		////data[loop][0] =joint[0].pos;
		////data[loop][1] =joint[1].pos;
		////FT_data[loop][0] =FTData[0];
		////FT_data[loop][1] =FTData[1];
		////FT_data[loop][2] =FTData[2];
		////data[loop][2] =joint[0].anInput3;
		////FT_data[loop][3] =FTData[3];
		////FT_data[loop][4] =FTData[4];
		////FT_data[loop][5] =FTData[5];
		////data[loop][3] =joint[0].anInput4;
if (storeData==ON)
{	

		data[dataCounter][0] =joint[0].pos;
		data[dataCounter][1] =joint[1].pos;
		FT_data[dataCounter][0] =joint[0].Force;
		FT_data[dataCounter][1] =joint[0].Torque;
		
		dataCounter++;
}


		//data[loop][0] = 11;//joint[0].pos;
		//data[loop][1] = 1;//joint[1].pos;
		//FT_data[loop][0] = 0.2;//FTData[0];
		//FT_data[loop][1] = 0.3;//FTData[1];
		//FT_data[loop][2] = 0.4;//FTData[2];
		//data[loop][2] = 8;//joint[0].anInput3;
		//FT_data[loop][3] = 0.5;//FTData[3];
		//FT_data[loop][4] = 0.6;//FTData[4];
		//FT_data[loop][5] = 0.7;//FTData[5];
		//data[loop][3] = 9;//joint[0].anInput4;


//////		data[loop][9] = joint[0].anInput1;
//////		data[loop][10] = Force;
//////		data[loop][11] = Torque;
////////************************************************  Store Data **************************************************************************


	//int mtype1,mtype2;
	//mtype2 = GetMotorType(2);
	//mtype1 = GetMotorType(1);

	if(TogglePrint)
	{
		gotoxy(1,1);

		if (storeData==ON)
		printf("\n ----------------------Storing data.        Data counter:%*d --------------\n\n",7,dataCounter);
		else if(deleteData==OFF)
		printf("\n ----------------------NOT Storing data.    Data counter:%*d --------------\n\n",7,dataCounter);
		else
		printf("\n ----------------------Deleted Stored data. Data counter:%*d --------------\n\n",7,dataCounter);


		//printf("MotorType2 = %d \t MotorType1 = %d \n ", mtype2, mtype1);
		//printf("t=%.3f\t ts:%.12f \n Feq:%.4f \n targetP=%d \t p=%d \t w=%.6f \t height=%.6f \n heightMin=%.6f Cur=%f Fz=%.6f AnalogIn1=%d AnalogIn2=%d AnalogIn3=%d AnalogIn4=%d\n ",curSimTime,ts,feq, desPos[1],joint[1].pos, data[dataCounter][3], data[dataCounter][5], data[dataCounter][6],data[dataCounter][7],data[dataCounter][11], joint[0].anInput1, joint[0].anInput2 , joint[0].anInput3, joint[0].anInput4);
		//printf("t=%.3f\t ts:%.12f \n Feq:%.4f \n targetP=%d \t p=%d \t w=%.6f \t height=%.6f \n heightMin=%.6f Cur=%f Fz=%.6f \n",curSimTime,ts,feq, desPos[0],joint[0].absPos1, data[dataCounter][3], data[dataCounter][5], data[dataCounter][6],data[dataCounter][7],data[dataCounter][11]);
		//printf("\n \n Position = %*d \n Force(AnalogIn1)=%*f \n Force(AnalogIn2)=%*f \n Force(AnalogIn3)=%*f \n Torque(AnalogIn2)=%*f \n ", 7,joint[0].pos, 7,joint[0].anInput3*gainC, 7,joint[0].anInput4*gainC, 7,Force ,7,Torque);
		//printf("\n \n Position = %*d \n Force(AnalogIn3)=%*f \n Torque(AnalogIn4)=%*f \n ", 7,joint[0].pos, 7,Force ,7,Torque);
		printf("\n \n Position = %*d \n Force(AnalogIn3)=%*f \n Torque(AnalogIn4)=%*f \n ", 7,joint[0].pos, 7,joint[0].Force ,7,joint[0].Torque );


		printf("\n\n Sensor data Fx=%10f Fy=%10f Fz=%10f \n", FTData[0], FTData[1], FTData[2]);
		printf("Sensor data Tx=%10f Ty=%10f Tz=%10f \n", FTData[3], FTData[4], FTData[5]);

		printf("\n \n AnalogIn1=%*d \n AnalogIn2=%*d\n AnalogIn3=%*d \n AnalogIn4=%*d \n", 7,joint[0].anInput1, 7,joint[0].anInput2, 7,joint[0].anInput3, 7,joint[0].anInput4);

		//printf("Sensor data Fx=%10f Fy=%10f Fz=%10f Tx=%10f Ty=%10f Tz=%10f \n", FTData[0], FTData[1], FTData[2], FTData[3], FTData[4], FTData[5]);
		printf("\n MainMPos=%*d  ThetaW=%*d MainVel=%*d  \n \n Xacceleration=%*d Yacceleration=%*d Zacceleration=%*d \n", 7,joint[0].TwinActPos,7,joint[0].TwinTargPos, 7,joint[0].TwinVel, 5,joint[0].Xaccleration, 5,joint[0].Yaccleration, 5,joint[0].Zaccleration);
		printf("\n MainPos=%*d  MainVel=%*d MainPosTar=%*d\n", 7,joint[1].pos,7,joint[1].vel,7,desPos[1]);
		printf("\n DampPos=%*d  DampVel=%*d  DampPosTar=%*d  FunctionOffset=%*d \n", 7,joint[0].pos,7,joint[0].vel, 7,desPos[0],7,FunctionOffset);
		printf("\n rampPos=%*d  rampSTEP=%*d   \n", 7,rampPos,7,rampSTEP);
		
		//printf("%.4f \t %d \t %d \t %d \t %d \t %.4f \t %.4f \t %.4f \t %.4f \t %.4f \t %.4f\n",sT[dataCounter], data[dataCounter][0],data[dataCounter][1],data[dataCounter][2],data[dataCounter][3],FT_data[dataCounter][0],FT_data[dataCounter][1],FT_data[dataCounter][2],FT_data[dataCounter][3],FT_data[dataCounter][4],FT_data[dataCounter][5]); //time,damper motor position,main motor position, force(counts),force(N), accns //added by Yogesh on 13-2-13	
		

		
		//printf("\n\n NOT 1 1=%d \t 2 1=%d \t 3 1=%d \n", (1&~1), (2&~1), (3&~1));

		//printf("\n Pos=%d  targetP=%d v=%d  Xacceleration=%d Yacceleration=%d Zacceleration=%d \n", joint[1].pos,joint[1].vel, joint[1].TwinVel, joint[1].Xaccleration, joint[1].Yaccleration, joint[1].Zaccleration);
		//printf("\n Pos=%d  targetP=%d v=%d  Xacceleration=%d Yacceleration=%d Zacceleration=%d \n", joint[1].pos,joint[1].vel, joint[1].TwinVel, joint[1].Xaccleration, joint[1].Yaccleration, joint[1].Zaccleration);
		//gotoxy(1,15);printf("t=%.3f  ts:%.12f Feq:%.4f ",curSimTime,ts,feq);
	}

	ts = timeObject.getSysTime() - startLoopTime;
	feq=1/ts;
		
	loop++;
	
	}
	close();
	

	//stopPositionControl();

	for(int b=FIRST_LBODY_BOARD;b<LAST_LBODY_BOARD;b++)
		stopBroadCast(b);     
	Sleep(500);
	// clsoe the network sockets
	CloseSockets();
	Sleep(3000);
	// wait for few seconds
	sampleloop = loop;
	// Save data to file
	saveData();

	

	return 0;
}
void updateLoop(void)
{
	double time1LoopHap=0.0, PrintLoop=0.0;

	PrintClock->reset();
	PrintClock->start();

	mainClock->reset();
	mainClock->start();

	while(simulationRunning)
	{
		//int g=0;
		//contHap+=1;
		//THapSec->reset();
		//THapSec->start();


		//time1LoopHap=THapSec->getCurrentTimeSeconds()+time1LoopHap;

		////printf("Loop: %0.8f \t Time:%0.8f\n",time1LoopHap,THapSec->getCurrentTimeSeconds());

		//if (time1LoopHap >= 5)
		//{
		//	printf("Loop: %0.4f\n",time1LoopHap);		
		//	time1LoopHap = 0.0;
		//	THapSec->reset();
		//}

		PrintLoop=PrintClock->getCurrentTimeSeconds();

		//printf("Loop: %0.8f \t Time:%0.8f\n",time1LoopHap,THapSec->getCurrentTimeSeconds());

		if (PrintClock->getCurrentTimeSeconds() >= 0.02)
		{
			printf("Loop: %0.4f\n",PrintLoop);		
			//time1LoopHap = 0.0;
			PrintClock->reset();
		}

	}


}

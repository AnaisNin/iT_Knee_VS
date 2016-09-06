/*  This program was modified from Haptic Actuator Intercom_WStartUp.cpp on 1st Feb 2013 for braking application asked by Nikos */

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


#define TRAJ_SAMPLES 50000
#define SAMPLES 1000000
#define PI 3.141592653589793
# define SIMULATION_TIME 3600
#define TSAMPLE 0.001//0.0005

#define BCAST_POLICY 1791
#define EXTRA_BCAST_POLICY 3584
#define BCAST_RATE 1

#define Fgain 0.0073 
#define Tgain 1
#define offsetF 30358
#define offsetT 30675
#define gainA 35.53
#define gainB 1.0876e-004
#define gainC 0.007


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

int desTor;
int pidOutput;
long pidError;
int pid_off[15];
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

//second prototype 
jointData joint[1]={

1,0,0,0,0,0,200,0,0,10000,10000,0,0,60,-60,0,0,0,0,0,0,0,3333,0,1600,16,1000,32000,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,PID_CONTROL,
};

int oldTarget=0;
int r;
int loop=0;
int sampleloop=0;
int ch=0;
int stopSim=0;

double data[SAMPLES][12];
float sT[SAMPLES];

///////////////////////////////////////////////////////////////////////////////////////////////////////

long temp;
long posPID[3];
long torPID[3];
char pidGains[20];

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

torPID[0]=0;
torPID[1]=0;
torPID[2]=0;

// sprintf_s is changing gains (long datatypes) to char types for SetPidGains function variable char * gains. 
sprintf_s(pidGains, sizeof(pidGains)/sizeof(char), "%d,%d,%d", gains[0],gains[1],gains[2]); 
		temp=SetPidGains(2,1,pidGains);
}

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
void saveData()
/******************************************/
{
	FILE *fp;
	char filename[256];
	cout<<"enter file name with extension (.txt) \n";
	if (_getch()==13)
		fp=fopen("adata.txt","w");
	else
	{	
		cin>>filename;	
		fp=fopen(filename,"w");
	}
	
	for(int d=0;d<sampleloop;d++)
fprintf(fp,"%.4f \t %.4f \t %.4f \t %.4f \t %.1f \t \n",sT[d], data[d][2],data[d][3],data[d][9],data[d][10]); //time,damper motor position,main motor position, force(counts),force(N) //added by Yogesh	
	fclose(fp);
}

int _tmain(int argc, _TCHAR* argv[])
{
	int res;
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

		setBroadCastRate(1,BCAST_RATE);    //Main Motor Board
		setBroadCastPolicy(1,BCAST_POLICY); 

		//setExtraBroadCastPolicy(1,EXTRA_BCAST_POLICY);

		setBroadCastRate(0,BCAST_RATE);    //Damper Motor Board
		setBroadCastPolicy(0,BCAST_POLICY); 

	// wait for few secs
	
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

		devMeasurementFns myUDP;
		UDP_Communication myUDP_1;
		myUDP_1.InitializeWinSock();

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

			 if(((packetToReceive.content[9])& 0x80)>0) joint[boardNo-1].vel= ((unsigned char)packetToReceive.content[8]+((packetToReceive.content[9]<<8)& 0xff00))-0xffff;
			 else joint[boardNo-1].vel= ((unsigned char)packetToReceive.content[8]+((packetToReceive.content[9]<<8)& 0xff00));


		joint[boardNo-1].absPos1=(unsigned char)packetToReceive.content[30]+((packetToReceive.content[31]<<8)& 0xff00) 
				+((packetToReceive.content[32]<<16)& 0xff0000)+((packetToReceive.content[33]<<24)& 0xff000000); 
			}
			bytesReceived=0;
		}	

	while(curSimTime<SIMULATION_TIME && !stopSim)
	{
		
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
				GetPidGains(1,1,gains);
			//	gotoxy(1,20); printf("p=%d, i=%d, d=%d,\n",gains[0],gains[1],gains[2]);
				GetPidGains(2,1,gains);
			//	gotoxy(1,21); printf("p=%d, i=%d, d=%d,\n",gains[0],gains[1],gains[2]);
			break;

			case 't':
				DisablePwmPad(0);//stopPositionControl();
				DisablePwmPad(1);
			break;

			case 'a':
				//desPos[1]+=15700; 
				desPos[1]+=13000; // for braking 
					r=SetDesiredPosition(desPos);
				
				break;

			case 'z':
				desPos[1]-=5000;
				desPos[0]-=1000;

			break;

			case 's':
				desPos[1]+=3000;
				if(desPos[1]>26000)desPos[1]=26000;
			break;

			case 'x':
				desPos[1]-=3000;
				if(desPos[1]<10000)desPos[1]=10000;
			break;

			case 'g':
				controlIsOn=1;
				startPositionControl();
				startTorqueControl(1,1);
				
			break;
			
			case 'd':
				controlIsOn=1;
				startTorqueControl(1,1);
				
			break;
			
			case 'r':
				desPos[1]=joint[1].pos-15000;
				//desPos[0]=joint[0].pos-15000;
							
			break;

			case 'h':
				controlIsOn=0;
				stopPositionControl();
				//startTorqueControl(1,0);
			break;

			case 'm':
				//	controlIsOn=0;	
					desPos[0]-=1000;
				//startTorqueControl(1,0);
			break;

			case 'o':
				// this button sets offsets for no contact force and position
				//desPos[0]=joint[0].pos;
				
				desPos[0]=1;
				r=SetDesiredPosition(desPos);
				printf("\n Desired position activated \n");
				desPos[0]=0;
				
				Destor[0]=joint[0].anInput1;
				//r=SetDesiredTorque(Destor);				
			break;

			case 'p':// this button sets offsets automatically for no contact force and position
				gains[1]=0;
				gains[2]=0;
				gains[0]=0;
				setControllerGains();
			
				DesOffset[0] = 10000;
				SetPidOffset(DesOffset);
				
			break;

			default:
			break;
		}
		
		ch=0;
		feq = 1/ts;
	
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
			
				if (((packetToReceive.content[17])& 0x80)>0) 
					joint[boardNo-1].realCurrent=(unsigned char)packetToReceive.content[14]+((packetToReceive.content[15]<<8)& 0xff00) 
								  +((packetToReceive.content[16]<<16)& 0xff0000)+((packetToReceive.content[17]<<24)& 0xff000000)-0xffff;
			else
					joint[boardNo-1].realCurrent=(unsigned char)packetToReceive.content[14]+((packetToReceive.content[15]<<8)& 0xff00) 
								  +((packetToReceive.content[16]<<16)& 0xff0000)+((packetToReceive.content[17]<<24)& 0xff000000);
				
				if(((packetToReceive.content[9])& 0x80)>0) joint[boardNo-1].vel= ((unsigned char)packetToReceive.content[8]+((packetToReceive.content[9]<<8)& 0xff00))-0xffff;
			 else joint[boardNo-1].vel= ((unsigned char)packetToReceive.content[8]+((packetToReceive.content[9]<<8)& 0xff00));
			
			 joint[boardNo-1].tor= packetToReceive.content[10]+(packetToReceive.content[11]*256);

			  joint[boardNo-1].anInput1= ((unsigned char)packetToReceive.content[30]+((packetToReceive.content[31]<<8)& 0xff00));

			 joint[boardNo-1].anInput2= ((unsigned char)packetToReceive.content[32]+((packetToReceive.content[33]<<8)& 0xff00));
			 			 
			 if(((packetToReceive.content[35])& 0x80)>0) joint[boardNo-1].anInput3= ((unsigned char)packetToReceive.content[34]+((packetToReceive.content[35]<<8)& 0xff00))-0xffff;
			 else joint[boardNo-1].anInput3= ((unsigned char)packetToReceive.content[34]+((packetToReceive.content[35]<<8)& 0xff00));

			 if(((packetToReceive.content[37])& 0x80)>0) joint[boardNo-1].anInput3= ((unsigned char)packetToReceive.content[36]+((packetToReceive.content[37]<<8)& 0xff00))-0xffff;
			 else joint[boardNo-1].anInput3= ((unsigned char)packetToReceive.content[36]+((packetToReceive.content[37]<<8)& 0xff00));

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

		data[loop][2] = joint[1].pos;
	
		data[loop][4] = (float)joint[1].tor;

		float Force;
		float Torque;

		data[loop][2] = joint[0].pos;
		data[loop][3] = joint[0].TwinActPos;
		data[loop][9] = joint[0].anInput1;
		Force = (float)(offsetF-joint[0].anInput1)*gainC;
		data[loop][10] = Force;

		Torque = gainB*(float)(offsetT-joint[0].anInput2)-(gainB*gainA*Force);
		data[loop][11] = Torque;

	gotoxy(1,11);
	
	printf("Position = %d \n Force-AnalogIn1=%f Torque-AnalogIn2=%f \n ", joint[0].pos, Force ,Torque);
	
	printf("\n MainPos=%d  MainVel=%d MainPosTar=%d\n", joint[1].pos,joint[1].vel,desPos[1]);
	printf("\n DampPos=%d  DampVel=%d DampPosTar=%d\n", joint[0].pos,joint[0].vel, desPos[0]);
	
	ts = timeObject.getSysTime() - startLoopTime;
	feq=1/ts;
	loop++;
	}
	
	Sleep(5000);
	//stopPositionControl();

	for(int b=FIRST_LBODY_BOARD;b<LAST_LBODY_BOARD;b++)
		stopBroadCast(b);     
	
	// clsoe the network sockets
	CloseSockets();
	// wait for few seconds
	sampleloop = loop;
	// Save data to file
	saveData();

	return 0;
}
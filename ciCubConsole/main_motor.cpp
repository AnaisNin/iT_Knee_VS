/ ciCubConsole.cpp : Defines the entry point for the console application.
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
#define BCAST_POLICY 51455
//#define BCAST_POLICY 1
#define BCAST_RATE 0.0010 //0.0005

//#define MAX_BOARDS 15

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

//jointData joint[MAX_BOARDS];

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
void setBroadCastRate(int boardNumber, int BCastRate)
/******************************************/
{
	r=SetBCastRate(activeBoards[boardNumber], 2*BCastRate);  // 1=0.5msec 
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
	

	fp=fopen("adata.txt","w");
//	fprintf(fp, "time, M1Despos, M1RealPos, M1vel, M1tor, M2Despos, M2RealPos, M2vel, M2tor\n");
	for(int d=0;d<sampleloop;d++)
		//fprintf(fp, "%.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f\n",sT[d], data[d][1], data[d][2], data[d][3], data[d][4], data[d][5], 
		//data[d][6], data[d][7], data[d][8]);
	fprintf(fp,"%.4f \t %.4f \t %.4f\t %.4f\t %10.4f\t %10.4f\t  %10.4f\t %10.4f\t \n",sT[d], data[d][2],data[d][3],data[d][4],data[d][7], data[d][9], data[d][10], data[d][11]); //added by Yogesh	
	

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


	for(int b=FIRST_LBODY_BOARD;b<LAST_LBODY_BOARD;b++){
		setBroadCastRate(b,BCAST_RATE*1000);    //1 = 0.0005sec
		setBroadCastPolicy(b,BCAST_POLICY);    
	} 

	
	Sleep(1000);

//	system("cls");

	desVel[0]=2000;
	desVel[1]=5000;
	r=SetDesiredVelocity(desVel);
	//stopPositionControl();
	
	// get simulation run time
	startSimTime = timeObject.getSysTime();
	// run until the defined sim time
	int count=0;

		
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

		//joint[boardNo-1].absPos1=(unsigned char)packetToReceive.content[30]+((packetToReceive.content[31]<<8)& 0xff00); 
				
			
				
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
				desPos[1]+=15000;
				if(desPos[1]>800000)desPos[1]=800000;
				
				break;

			case 'z':
				desPos[1]-=15000;
				//if(desPos[1]<0)desPos[1]=0;
			
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
				
			break;
			
			case 'r':
				desPos[1]=joint[1].pos-15000;
							
			break;

			case 'h':
				controlIsOn=0;
				
				stopPositionControl();
				
			break;

			default:
			break;
		}
		
		ch=0;
		feq = 1/ts;
	
		r=SetDesiredPosition(desPos);
		
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
			
		/*	 joint[boardNo-1].tor= packetToReceive.content[10]+(packetToReceive.content[11]*256);


			 if(((packetToReceive.content[13])& 0x80)>0) joint[boardNo-1].pidOutput= ((unsigned char)packetToReceive.content[12]+((packetToReceive.content[13]<<8)& 0xff00))-0xffff;
			 else joint[boardNo-1].pidOutput= ((unsigned char)packetToReceive.content[12]+((packetToReceive.content[13]<<8)& 0xff00));
			 
			
				joint[boardNo-1].pidError=(unsigned char)packetToReceive.content[14]+((packetToReceive.content[15]<<8)& 0xff00) 
								  +((packetToReceive.content[16]<<16)& 0xff0000)+((packetToReceive.content[17]<<24)& 0xff000000);*/

			// joint[boardNo-1].absPos1=(unsigned char)packetToReceive.content[30]+((packetToReceive.content[31]<<8)& 0xff00); 
		
				
			}
			bytesReceived=0;
		}	
		if((desTor& 0x8000)>0) desTor=desTor -0xffff;
				
	gotoxy(1,11);printf("t=%.3f\t ts:%.12f \n Feq:%.4f \n targetP=%d \t p=%d \t w=%.6f \t height=%.6f \n heightMin=%.6f Cur=%f Fz=%.6f \n",curSimTime,ts,feq, desPos[0],joint[0].absPos1, data[loop][3], data[loop][5], data[loop][6],data[loop][7],data[loop][11]);
	
		ts = timeObject.getSysTime() - startLoopTime;
		feq=1/ts;
	
	loop++;
	
	}
	

	//int newtime;
	//int oldtime=0;
	//devMeasurementFns myUDP;
	//UDP_Communication myUDP_1;

	//myUDP_1.InitializeWinSock();
	////myUDP.ReadFTsensorData();
	//while(1){
	//newtime=clock();
	////myUDP.RequestFTsensorData();
	//myUDP.ReadFTsensorData();
	//myUDP.GetFTData(FTData);
	////myUDP.GetRawFTData(FTData);
	////printf("sensor x= %f \n", FTData[1]);	
	////printf("Sensor data Fx=%10f Fy=%10f Fz=%10f Tx=%10f Ty=%10f Tz=%10f \n", FTData[0], FTData[1], FTData[2], FTData[3], FTData[4], FTData[5]);
	//
	//if (newtime-oldtime>=20)
	//{
	//	oldtime=newtime;
	//	//printf("Clock = %10d \n", oldtime);
	//    //printf("Sensor data Fx=%10f Fy=%10f Fz=%10f \n", FTData[0], FTData[1], FTData[2]);	
	//	//printf("Sensor data Fx=%10f Fy=%10f Fz=%10f Tx=%10f Ty=%10f Tz=%10f \n", FTData[0], FTData[1], FTData[2], FTData[3], FTData[4], FTData[5]);
	//	//printf("Sensor data Fx=%d Fy=%d Fz=%d Tx=%d Ty=%d Tz=%d \n", FTData[0], FTData[1], FTData[2], FTData[3], FTData[4], FTData[5]);
	//	FILE *xp;
	//	xp = fopen(filename,"a");
	//	fprintf(xp,"%10f \t %10f \t %10f \n",FTData[0], FTData[1], FTData[2]); //added by Yogesh
	//	fclose(xp);
	//}
	//		
	//}

	Sleep(5000);
	stopPositionControl();

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

//This is a UDP server class Operating with a Non-Blocking recvfrom function 
//in order to stream and also accept commands.
#include "StdAfx.h"
#include "UDP_server.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <time.h>
#include <winsock.h>
#include <conio.h>
#include <math.h>
#include <sstream>

#include "shTime.h"
#include <Windows.h>
using namespace std;
#define SIZE 128
#define BUFFER_SIZE 512
//#define PI	3.14159

CUDP_server::CUDP_server(void)
{
}

CUDP_server::~CUDP_server(void)
{
}

int argc=3;
char *argv[4] = {"foo", "127.0.0.1", "5000"};


WSADATA w;							/* Used to open windows connection */
unsigned short port_number;			/* Port number to use */
int a1, a2, a3, a4;					/* Components of address in xxx.xxx.xxx.xxx form */
int client_length;					/* Length of client struct */
int bytes_received;					/* Bytes received from client */
SOCKET sd;							/* Socket descriptor of server */
struct sockaddr_in server;			/* Information about the server */
struct sockaddr_in client;			/* Information about the client */
char buffer[BUFFER_SIZE];			/* Where to store received data */
struct hostent *hp;					/* Information about this computer */
char host_name[256];				/* Name of the server */
time_t current_time;				/* Current time */

char sendValue[128];
string myString(buffer);

int CUDP_server::connectUDPserver(void)
{

	///* Interpret command line */
	if (argc == 2)
	{
		/* Use local address */
		if (sscanf(argv[1], "%u", &port_number) != 1)
		{
			CUDP_server::usage();
		}
	}
	else if (argc == 3)
	{
		/* Copy address */
		if (sscanf(argv[1], "%d.%d.%d.%d", &a1, &a2, &a3, &a4) != 4)
		{
			CUDP_server::usage();
		}
		if (sscanf(argv[2], "%u", &port_number) != 1)
		{
			CUDP_server::usage();
		}
	}
	else
	{
		CUDP_server::usage();
	}

	/* Open windows connection */
	if (WSAStartup(0x0101, &w) != 0)
	{
		fprintf(stderr, "Could not open Windows connection.\n");
		exit(0);
	}

	/* Open a datagram socket */
	sd = socket(AF_INET, SOCK_DGRAM, 0);
	if (sd == INVALID_SOCKET)
	{
		fprintf(stderr, "Could not create socket.\n");
		WSACleanup();
		exit(0);
	}

	/* Clear out server struct */
	memset((void *)&server, '\0', sizeof(struct sockaddr_in));

	/* Set family and port */
	server.sin_family = AF_INET;
	server.sin_port = htons(port_number);

	/* Set address automatically if desired */
	if (argc == 2)
	{
		/* Get host name of this computer */
		gethostname(host_name, sizeof(host_name));
		hp = gethostbyname(host_name);

		/* Check for NULL pointer */
		if (hp == NULL)
		{
			fprintf(stderr, "Could not get host name.\n");
			closesocket(sd);
			WSACleanup();
			exit(0);
		}

		/* Assign the address */
		server.sin_addr.S_un.S_un_b.s_b1 = hp->h_addr_list[0][0];
		server.sin_addr.S_un.S_un_b.s_b2 = hp->h_addr_list[0][1];
		server.sin_addr.S_un.S_un_b.s_b3 = hp->h_addr_list[0][2];
		server.sin_addr.S_un.S_un_b.s_b4 = hp->h_addr_list[0][3];
	}
	/* Otherwise assign it manually */
	else
	{
		server.sin_addr.S_un.S_un_b.s_b1 = (unsigned char)a1;
		server.sin_addr.S_un.S_un_b.s_b2 = (unsigned char)a2;
		server.sin_addr.S_un.S_un_b.s_b3 = (unsigned char)a3;
		server.sin_addr.S_un.S_un_b.s_b4 = (unsigned char)a4;
	}

	/* Bind address to socket */
	if (bind(sd, (struct sockaddr *)&server, sizeof(struct sockaddr_in)) == -1)
	{
		fprintf(stderr, "Could not bind name to socket.\n");
		closesocket(sd);
		WSACleanup();
		exit(0);
	}

	/* Print out server information */
	printf("Server running on %u.%u.%u.%u\n", (unsigned char)server.sin_addr.S_un.S_un_b.s_b1,
		(unsigned char)server.sin_addr.S_un.S_un_b.s_b2,
		(unsigned char)server.sin_addr.S_un.S_un_b.s_b3,
		(unsigned char)server.sin_addr.S_un.S_un_b.s_b4);
	printf("Press CTRL + C to quit\n");

	/* Loop and get data from clients */

	DWORD nonBlocking = 1;
	if ( ioctlsocket( sd, FIONBIO, &nonBlocking ) != 0 )
	{
		printf( "failed to set non-blocking socket\n" );
		return false;
	}
	client_length = (int)sizeof(struct sockaddr_in);

	/* Uncomment from here for the streaming version*/
	/* Receive bytes from client */
	////bytes_received = recvfrom(sd, buffer, BUFFER_SIZE, 0, (struct sockaddr *)&client, &client_length);
	////if (bytes_received < 0)
	////{
	////	fprintf(stderr, "Could not receive datagram.\n");
	////	closesocket(sd);
	////	WSACleanup();
	////	exit(0);
	////}
	////else
	////{
	////fprintf(stderr, "UDP server started\n");
	////}
	/* Uncomment until here for the streaming version*/

}

void CUDP_server::sendUDPdata(char* dataToSend)
{
char *sendValue1;
double ts1;
sendValue1=dataToSend;

		/* Comment from here for the streaming version*/
		/* Receive bytes from client */
		bytes_received = recvfrom(sd, buffer, BUFFER_SIZE, 0, (struct sockaddr *)&client, &client_length);
		if (bytes_received < 0)
		{
			//fprintf(stderr, "Could not receive datagram.\n");
			//closesocket(sd);
			//WSACleanup();
			//exit(0);
		}
		/* Comment until here for the streaming version*/
		//fprintf(stderr, "bytes_received = %d\n", bytes_received );

		//if (strcmp(buffer, "GET TIME\r\n") == 0)
		//{
		//printf("dataIn: %s", buffer);
		//printf("r=: %d \n", r);
		//* Get current time */
		//current_time = time(NULL);

		//* Send data back */
		////if (sendto(sd, (char *)&current_time, (int)sizeof(current_time), 0, (struct sockaddr *)&client, client_length) != (int)sizeof(current_time))
		////{
		////	fprintf(stderr, "Error sending datagram.\n");
		////	closesocket(sd);
		////	WSACleanup();
		////	exit(0);
		////}

		//int value = atoi(myString.c_str());
		//printf("buffer=: %s \t Value:%d \n", myString.c_str(),r);

		//tl=0;
		//float sinewv, sinewv2;
		//sinewv=(float)sin(2.0*PI*freq*ts);
		//sinewv2=(float)sin(2.0*PI*freq*ts+3.0*PI/2.0);
		//sprintf(sendValue,"%0.3f %0.3f %0.3f %0.3f\n", sinewv, (float)ts, (float)tl, sinewv2);  //				

		//printf(" sendValue= %s \n", sendValue1);  //
		if (bytes_received > 0)
		{
			if (sendto(sd, sendValue1, (int)sizeof(sendValue), 0, (struct sockaddr *)&client, client_length) != (int)sizeof(sendValue))
			//if (sendto(sd, (char *) &sendValue, (int)sizeof(sendValue), 0, (struct sockaddr *)&client, client_length) != (int)sizeof(sendValue))
			{
				fprintf(stderr, "Error sending datagram.\n");
				//closesocket(sd);
				//WSACleanup();
				//exit(0);
			}
		}
		//}
		//tl=timeObject.getSysTime() - startLoopTime;
		//printf(" LootTime= %f \n", tl);  //	
}
void CUDP_server::closeUDPserver(void)
{
	closesocket(sd);
	WSACleanup();
}
void CUDP_server::usage(void)
{
	// TODO: Add your control notification handler code here
	printf("timeserv [server_address] port\n");
	exit(0);
}
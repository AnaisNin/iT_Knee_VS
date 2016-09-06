/*	This class implements functions that allow to communicate	*/
/*	over the network with the FT sensor.						*/


#include "stdafx.h"
#include "UDP_Communication.h"


//	Initialization of NETWORK variables
WSADATA wsaData;

SOCKET SendRecvSocket;

sockaddr_in Address;

int Port = 49152;

UDP_Communication::UDP_Communication(void)
{
}

UDP_Communication::~UDP_Communication(void)
{
}

void UDP_Communication::InitializeWinSock()
{
	//	This function initializes the Socket for UDP communication.	//

	// Initialize Winsock
	WSAStartup(MAKEWORD(2,2), &wsaData);

	// Create a socket for sending data
	SendRecvSocket = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
	
	// Create a receiver socket to receive datagrams
	Address.sin_family = AF_INET;
	//Insert the IP address of the FT Sensor

	//Address.sin_addr.s_addr = inet_addr("10.255.32.18");	//ATI 45		NetBox MAC:00 16 BD 00 03 0F (Jody's IP)
	//Address.sin_addr.s_addr = inet_addr("10.255.43.21");	//ATI 45		NetBox MAC:00 16 BD 00 03 0F
	Address.sin_addr.s_addr = inet_addr("169.254.89.60");	//ATI 45		NetBox MAC:00 16 BD 00 03 0F
	//Address.sin_addr.s_addr = inet_addr("169.254.89.61");	//ATI 45		NetBox MAC:00 16 BD 00 03 0F
	//Address.sin_addr.s_addr = inet_addr("10.255.32.133");	//ATI nano25	NetBox MAC:00 16 BD 00 03 AF (Iqbal's IP)
	Address.sin_port = htons(Port);

	int ATIcon = connect(SendRecvSocket,				// Socket
			(LPSOCKADDR)&Address,						// Server address
			sizeof(struct sockaddr));
}
void UDP_Communication::UDPSendMessage(char MessageBuf[BufLenS])
{
	//	This function sends a message over the network	//

	send(SendRecvSocket,				// Connected socket
				MessageBuf,			// Data buffer
				BufLenS,			// Length of data
				0);
}
void UDP_Communication::UDPReceieveMessage(char RecvBuf[BufLenR])
{
	//	This function receives a message from the network	//
	recv(SendRecvSocket, RecvBuf, BufLenR, 0);
}
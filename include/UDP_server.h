#pragma once

class CUDP_server
{
public:
	CUDP_server(void);
	~CUDP_server(void);

	void mainCall(void);
	int connectUDPserver(void);
	void sendUDPdata(char*);
	void closeUDPserver(void);
	void usage(void);
	char UDPdata[128];
	float fUDPdata[3];
};

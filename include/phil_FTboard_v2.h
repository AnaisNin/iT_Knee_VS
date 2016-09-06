#ifndef PHIL_BOARD_H
#define PHIL_BOARD_H

#include "utils.h"

#ifdef __cplusplus
extern "C" {
#endif

#define COMMAND_MAX_SIZE   2
#define PACKET_MAX_SIZE   128

#define  HEADER_TCP_COMMAND   0xFF
#define  HEADER_TCP_REPLY     0xFE
#define  HEADER_UDP_COMMAND   0xFF
#define  HEADER_UDP_REPLY     0xFF

#define deg2count	6900/180;
#define count2deg	180/6900;

extern const unsigned int fLength_header;
extern const unsigned int fLength_payloadSize;
extern const unsigned int fLength_checksum;

typedef struct packet_descriptor {
    char  header;
    char  commandSize;
    char  command[COMMAND_MAX_SIZE];
    unsigned char  payloadSize;
    unsigned char  totSize;
} PckDescr;


typedef struct char_buff {
    char content[PACKET_MAX_SIZE];
    unsigned char size;
} CharBuff;


typedef enum tcp_commands {
GET_BOARD_TYPE = 0,
GET_FIRMWARE_VERSION,
SET_FIRMWARE_VERSION,

CLEAR_BOARD_FAULT,
GET_BOARD_FAULT,

SET_BCAST_RATE,
GET_BCAST_RATE,
SET_BCAST_POLICY,
GET_BCAST_POLICY,

CMD_UPGRADE,
SAVE_PARAMS_TO_FLASH,
LOAD_PARAMS_FROM_FLASH,
LOAD_DEFAULT_PARAMS,

CALIBRATE_OFFSETS,
SET_BOARD_NUMBER,
GET_CALIBRATION_OFFSETS,
SET_RESOLUTION,
GET_RESOLUTION,
SET_TEMP_FACTORS,
GET_TEMP_FACTORS,
GET_CAL_TEMP,

SET_CONVERSION_FACTORS,
GET_CONVERSION_FACTORS,

SET_AVARAGE_SAMPLES,
GET_AVARAGE_SAMPLES,

SET_MATRIX_ROW,
GET_MATRIX_ROW,
///////////////////////////////////////////////////////////////////////////////////
//New commands for user setting the IP of the sensor.
//The external switch can force the sensor to the following settings
//IP address 	192.168.1.1
//Net mask	255.255.0.0
//Ethernet Port	23
//The new commands can be used to change the IP address, net mask and Ethernet port. Changes are only effective after the values are saved to the flash and the sensor is restarted.
//The default values for the sensor are
//IP address 	169.254.89.21
//Net mask	255.255.0.0
//Ethernet Port	23
SET_ETHERNET_PORT,
SET_IP_ADDR,
SET_NET_MASK,
GET_ETHERNET_PORT,
GET_IP_ADDR,
GET_NET_MASK,
///////////////////////////////////////////////////////////////////////////////////
/* LEAVE THIS SYMBOL AT THE END! */
TCP_COMMANDS_COUNT


} TCP_command;


typedef enum tcp_replies {
REPLY_BOARD_TYPE,
REPLY_FIRMWARE_VERSION,

REPLY_BOARD_FAULT,

REPLY_BCAST_RATE,
REPLY_BCAST_POLICY,

REPLY_CALIBRATION_OFFSETS,
REPLY_RESOLUTION,

REPLY_TEMP_FACTORS,
REPLY_CAL_TEMP,

REPLY_CONVERSION_FACTORS,

REPLY_AVARAGE_SAMPLES,

REPLY_MATRIX_ROW,

REPLY_ETHERNET_PORT,
REPLY_IP_ADDR,
REPLY_NET_MASK,


/* LEAVE THIS SYMBOL AT THE END! */
TCP_REPLIES_COUNT

} TCP_reply;



typedef enum UDP_commands {
	GET_ACTIVE_BOARDS,
	
	/* LEAVE THIS SYMBOL AT THE END! */
	UDP_COMMANDS_COUNT

} UDP_command;


typedef enum UDP_replies {
	REPLY_ACTIVE_BOARDS,
	
	/* LEAVE THIS SYMBOL AT THE END! */
	UDP_REPLIES_COUNT
} UDP_reply;


typedef enum UDP_broadcasts {
	BCAST_DATA_PACKET_MT,
	
	/* LEAVE THIS SYMBOL AT THE END! */
	UDP_BROADCASTS_COUNT
} UDP_broadcast;


//extern const PckDescr tcpCommands[TCP_COMMANDS_COUNT];

void init(unsigned int numOfBoards);

int getPayloadOffset(const PckDescr* descr);
const PckDescr* getDescriptor_TCPCmd(TCP_command which);
const PckDescr* getDescriptor_UDPCmd(UDP_command which);
const PckDescr* getDescriptor_TCPRep(TCP_reply which);

int buildTCPCommand(CharBuff* toBeFilled, TCP_command which, char* payload);
int buildUDPCommand(CharBuff* toBeFilled, UDP_command which, char* payload);
int buildTCPReply(CharBuff* toBeFilled,  TCP_reply which, char* payload);
int buildUDPReply(CharBuff* toBeFilled,  UDP_reply which, char* payload);
int buildUDPBroadcast(CharBuff* toBeFilled,  UDP_broadcast which, char* payload);

//int  getTCPReplyPayload(char* toBeFilled, Packet* replyPacket, TCP_reply which);


#ifdef __cplusplus
}
#endif


#endif

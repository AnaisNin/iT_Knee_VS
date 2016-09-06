//---------------POLLING MODE /STREAMING MODE--------------
//Uncomment the following to indicate POLLING MODE , commnet it to indicate STREAMING MODE

//#define IITFT_POLLING 1


//---------------MULTI SENSOR MODE / SINGLE SENSOR MODE------------
//Please follow the sensor instructions manual procedure on how set the sensor to DEFAULT_IP mode
//Uncomment the following line to indicate operation in MULTI SENSOR MODE, comment it to indicate SINGLE SENSOR MODE

//#define IITFT_MultiSensorMode 1

//--------------- DEFAULT_IP MODE / USER_IP MODE --------------
#ifndef IITFT_MultiSensorMode
//Please follow the sensor instructions manual procedure on how set the sensor to DEFAULT_IP mode
//Uncomment the following line to indicate that the connected sensor is in the DEFAULT_IP MODE, comment it to indicate USER_IP MODE

//#define IITFT_DefaultIP 1

#endif



//------------------------------	USER_IP MODE SETTINGS ---------------------------------
//When the sensor is in SINGLE_SENSOR Mode as described in the manual set the parameters of
//USER_IP, USER_IP_MASK and USER_PORT in this file to reflect the sensor settings
#define USER_IP1		169//
#define USER_IP2		254//254
#define USER_IP3		89//89
#define USER_IP4		21

#define USER_IP_MASK1	255
#define USER_IP_MASK2	255
#define USER_IP_MASK3	255
#define USER_IP_MASK4	0
#define USER_PORT		23

//------------------------------	MULTI SENSOR MODE SETTINGS ---------------------------------
//When the sensor is in MULTI_SENSOR Mode as described in the manual set the parameters of
//BASE_IP, BASE_IP_MASK and BASE_PORT in this file to reflect the sensor settings
#define BASE_IP1		169
#define BASE_IP2		254
#define BASE_IP3		89
#define BASE_IP4		20

#define BASE_IP_MASK1	255
#define BASE_IP_MASK2	255
#define BASE_IP_MASK3	255
#define BASE_IP_MASK4	0
#define BASE_PORT		23


//------------------------------	DEFAULT_IP MODE SETTINGS ------------------------------------
/////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////		DO NOT CHANGE		/////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////
//Do not change the DEFAUL_IP parameters
#define DEFAULT_IP1		192
#define DEFAULT_IP2		168
#define DEFAULT_IP3		1
#define DEFAULT_IP4		1
//Do not change the DEFAUL_IP_MASK parameters
#define DEFAULT_IP_MASK1	255
#define DEFAULT_IP_MASK2	255
#define DEFAULT_IP_MASK3	0
#define DEFAULT_IP_MASK4	0
//Do not change the DEFAUL_PORT parameters
#define DEFAULT_PORT		23
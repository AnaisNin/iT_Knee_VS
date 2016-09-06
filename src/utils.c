#include "../include/utils.h"


static const int INT_BYTES  = sizeof(int);
static const int LONG_BYTES = sizeof(long);

void bytesToLong(const char* buff, int count, long* value) {
	// The cast to unsigned char ensures that the corresponding long
	// value will be 0-extended, and not 1-extended (when the msb of
	// the char is high). A cast to unsigned long would not prevent
	// the 1-extension. I also put the cast to long to be sure (??) 
	// that enough bytes are "allocated" before doing the shift.
	int i;
	*value &= 0; //zeroes out the value
	for(i=0; i<count && i<LONG_BYTES; i++) {
		(*value) += ((long)((unsigned char)(buff[i]))) << (i*8);
	}
}

void longToBytes(long value, char* buff, int count) {
	int i;
	for(i=0; i<count && i<LONG_BYTES; i++) {
		*(buff+i) = (char)((value >> (i*8)) % 256);//maybe the modulus is not necessary. The cast to char is enough
	}
}

void bytesToInt(const char* buff, int count, int* value) {
	int i;
	*value &= 0; //zeroes out the value
	for(i=0; i<count && i<INT_BYTES; i++) {
		*value += ((int)((unsigned char)(buff[i]))) << (i*8);
	}
}

void intToBytes(int value, char* buff, int count) {
	int i;
	for(i=0; i<count && i<INT_BYTES; i++) {
		*(buff+i) = (char)((value >> (i*8)) % 256);//maybe the modulus is not necessary. The cast to char is enough
	}
}
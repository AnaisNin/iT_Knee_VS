/**
 \author Marco Frigerio
 \date March 2010
*/

#ifndef UTILS_H
#define UTILS_H

/*
#if (__STDC_VERSION__ >= 199901L)//this should check for a c99 compliant compiler
#define INLINE inline
#else
#define INLINE static
#endif
*/

#ifdef __cplusplus
extern "C" {
#endif



//TODO: when converting a number into a buff of chars, maybe we should not stop after
// {INT|LONG}_BYTES but just keep going putting zeroes until the limit argument is reached

void bytesToLong(const char* buff, int count, long* value);

void longToBytes(long value, char* buff, int count);

void bytesToInt(const char* buff, int count, int* value);

void intToBytes(int value, char* buff, int count);


#ifdef __cplusplus
}
#endif

#endif

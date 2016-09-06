#include <stdio.h>
#include <math.h>

#define M_PI 3.14159265358979
#define TRUE 255
#define FALSE 0
//#define Zc 0.48823//0.47573
#define Grav 9.80621
#define Rad2Deg 180/M_PI
#define Deg2Rad M_PI/180


float Polinom5(const float RealTime, const float tstart, const float twidth, const float z01, const float v01, const float a01, const float z02, const float v02, const float a02);
float FootPolinom6(const float RealTime, const float tstart, const float twidth, const float z0, const float za, const float z1);
unsigned int Interval(float RealTime, float ta, float tb);
unsigned int EqInterval(float RealTime, float ta, float tb);
unsigned int Equal(float RealTime, float ta);
unsigned int GrtOrEq(float RealTime, float ta);
unsigned int Greater(float RealTime, float ta);
unsigned int LessOrEq(float RealTime, float ta);
unsigned int Less(float RealTime, float ta);
//float Ytraj(float RealTime, float t0, float te, float y0, float ym, int ID);
float sech(float a);
float coth(float a);

float FuncYcom(float RealTime, float tstart, float twidth, float z, float y0, float yd);
float FuncXcom(float RealTime, float tstart, float twidth, float z, float x0, float Vmean);
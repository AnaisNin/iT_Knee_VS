#include "mUtils.h"

double g_DEG2RAD(const double & a_deg) 
{ 
	return a_deg*M_PI / 180; 
}

double g_RAD2DEG(const double & a_rad)
{ 
	return a_rad * 180 / M_PI; 
}
#ifndef M_UTILS_H
#define M_UTILS_H
//Class-less header file - fnt/var shared accross multiples .cpp -

#define _USE_MATH_DEFINES //M_PI
#include <math.h>

	//const 
	const double g_GRAVITYacc = 9.80665;
	double g_DEG2RAD(const double & a_deg); 
	double g_RAD2DEG(const double & a_rad);

	/*inline double m_DEG2RAD(const double & a_deg) { return a_deg*M_PI / 180; }
	inline double g_RAD2DEG(const double & a_rad) { return a_rad * 180 / M_PI; }*/


#endif
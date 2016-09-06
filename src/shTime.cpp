
//
// shTime.cpp 


#include "stdafx.h"
#include "shTime.h"



//-------------------------------------------------------------
// Returns the current time in seconds using best resolution.
//-------------------------------------------------------------
double shTime::getSysTime()
{
    static LARGE_INTEGER  s_frequency = {0};

    static double  s_wavelength        = 0.0;
    static double  s_wavelength_x_high = 0.0;

    static bool  s_isFirstTime = true;

    LARGE_INTEGER  l_ticks = {0};

    double  l_highResult = 0;
    double  l_lowResult  = 0;
    double  l_result     = 0;


    if( s_isFirstTime )
    {
        ::QueryPerformanceFrequency( &s_frequency );
        s_wavelength = 1.0 / (double)s_frequency.LowPart;
        s_wavelength_x_high = s_wavelength * (double)MAXDWORD;
        s_isFirstTime = false;
    }

    ::QueryPerformanceCounter( &l_ticks );

    l_highResult = s_wavelength_x_high * (double)l_ticks.HighPart;
    l_lowResult  = s_wavelength        * (double)l_ticks.LowPart ;
    l_result = l_highResult + l_lowResult;

    return l_result;
}







// End File: time.cpp

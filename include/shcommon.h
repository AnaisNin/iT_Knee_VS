
//
// common.h -- contains the common includes and defines for OS support.
//


#ifndef __shCOMMON_H__
#define __shCOMMON_H__

// We require Windows NT 4.0 or greater
#define _WIN32_WINNT 0x0400

// We don't pull in everything from windows.h, to reduce compile time
#define WIN32_LEAN_AND_MEAN

// The rope template class implementation we use (supplied with GHOST)
// correctly generates a warning from the Visual C++ due to using the
// this pointer in the member initialization list.  The correct
// solution is to avoid using rope, but for now we disable that
// warning.
//
// 'this' : used in base member initializer list
#pragma warning(disable:4355)

// Various standard header files require the windows.h header file.
// We include it here to avoid polluting our source code with OS
// specific includes.
#include <stdio.h>
#include <stdlib.h>
#include <malloc.h>
#include <tchar.h>
#include <windows.h>

// We prefer to treat assert as a "standard operation".  We include
// assert.h here for convenience, since we use assert so extensively.
#include <assert.h>

// A convenience include for math functions & constants.
#include <math.h>

#endif

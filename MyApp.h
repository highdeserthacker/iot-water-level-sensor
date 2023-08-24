///////////////////////////////////////////////////////////////////////////////
/*  MyApp.h - Application specific settings & defines, including debugging.  */
///////////////////////////////////////////////////////////////////////////////
#ifndef MyApp_h
#define MyApp_h

#include "AllApps.h"       // general settings for all my devices

/* Platform specific settings. */
#include <stdint.h>

/**************************************************************************************/
// Debug & Build Settings
/**************************************************************************************/
// Comment this out for production deployment.
#define _DEBUG_BUILD                                  

/* Debug build settings. Enables trace.                 */
#ifdef _DEBUG_BUILD
#define _DEBUG
#endif

// New Feature Testing


/* (optional) Trace switches */
enum
{
   /* Switches:   
      0: Reserved - TS_MAIN
      1..3: available for app specific uses
      4..7 Reserved and defined in QTrace.h     */
};


#endif

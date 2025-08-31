/* tpx3HistogramMain.cpp */
/* Author:  Marty Kraimer Date:    17MAR2000 */
/* Modified for Timepix3 Histogram IOC */

#include <stddef.h>
#include <stdlib.h>
#include <stddef.h>
#include <string.h>
#include <stdio.h>

#include "epicsExit.h"
#include "epicsThread.h"
#include "iocsh.h"
#include "epicsExport.h"

// Include our driver support
extern "C" {
    extern void register_func_tpx3HistogramConfigure(void);
}

int main(int argc,char *argv[])
{
    if(argc>=2) {
        iocsh(argv[1]);
        epicsThreadSleep(.2);
    }
    
    // Register our driver support
    register_func_tpx3HistogramConfigure();
    
    iocsh(NULL);
    epicsExit(0);
    return(0);
}

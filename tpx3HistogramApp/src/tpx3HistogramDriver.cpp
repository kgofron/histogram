#include "tpx3HistogramDriver.h"
#include <epicsExport.h>
#include <epicsThread.h>
#include <epicsTime.h>
#include <epicsString.h>
#include <epicsStdio.h>
#include <iocsh.h>
#include <cstring>

// Parameter definitions
#define NUM_PARAMS 22

// Global driver instance
static tpx3HistogramDriver* g_driver = NULL;

// Constructor
tpx3HistogramDriver::tpx3HistogramDriver(const char *portName, int maxAddr)
    : asynPortDriver(portName, maxAddr, NUM_PARAMS,
                     asynInt32Mask | asynOctetMask | asynDrvUserMask,
                     asynInt32Mask | asynOctetMask | asynDrvUserMask,
                     ASYN_CANBLOCK, 1, 0, 0),
      host_(DEFAULT_HOST),
      port_(DEFAULT_PORT),
      connected_(false),
      running_(false),
      frame_count_(0),
      total_counts_(0),
      bin_size_(MAX_BINS),
      error_count_(0),
      acquisition_rate_(0.0),
      processing_time_(0.0),
      memory_usage_(0.0)
{
    // Initialize bin arrays
    bin_values_.resize(MAX_BINS, 0);
    bin_edges_.resize(MAX_BINS + 1, 0.0);
    
    // Initialize bin edges (picoseconds)
    for (size_t i = 0; i <= MAX_BINS; ++i) {
        bin_edges_[i] = i * 0.260; // 0.260 ps per bin
    }
    
    // Create mutex and event
    mutex_ = epicsMutexCreate();
    dataReady_ = epicsEventCreate(epicsEventEmpty);
    
    // Initialize status
    status_ = "Initialized";
    
    // Create parameter indices
    connectIndex_ = 0;
    disconnectIndex_ = 1;
    resetIndex_ = 2;
    startIndex_ = 3;
    stopIndex_ = 4;
    saveDataIndex_ = 5;
    hostIndex_ = 6;
    portIndex_ = 7;
    frameCountIndex_ = 8;
    totalCountsIndex_ = 9;
    connectedIndex_ = 10;
    statusIndex_ = 11;
    errorCountIndex_ = 12;
    acquisitionRateIndex_ = 13;
    processingTimeIndex_ = 14;
    memoryUsageIndex_ = 15;
    binWidthIndex_ = 16;
    totalTimeIndex_ = 17;
    filenameIndex_ = 18;
    histogramDataIndex_ = 19;
    
    // Create parameters
    createParam("CONNECT", asynParamInt32, &connectIndex_);
    createParam("DISCONNECT", asynParamInt32, &disconnectIndex_);
    createParam("RESET", asynParamInt32, &resetIndex_);
    createParam("START", asynParamInt32, &startIndex_);
    createParam("STOP", asynParamInt32, &stopIndex_);
    createParam("SAVE_DATA", asynParamInt32, &saveDataIndex_);
    createParam("HOST", asynParamOctet, &hostIndex_);
    createParam("PORT", asynParamInt32, &portIndex_);
    createParam("FRAME_COUNT", asynParamInt32, &frameCountIndex_);
    createParam("TOTAL_COUNTS", asynParamInt32, &totalCountsIndex_);
    createParam("CONNECTED", asynParamInt32, &connectedIndex_);
    createParam("STATUS", asynParamOctet, &statusIndex_);
    createParam("ERROR_COUNT", asynParamInt32, &errorCountIndex_);
    createParam("ACQUISITION_RATE", asynParamFloat64, &acquisitionRateIndex_);
    createParam("PROCESSING_TIME", asynParamFloat64, &processingTimeIndex_);
    createParam("MEMORY_USAGE", asynParamFloat64, &memoryUsageIndex_);
    createParam("BIN_WIDTH", asynParamFloat64, &binWidthIndex_);
    createParam("TOTAL_TIME", asynParamFloat64, &totalTimeIndex_);
    createParam("FILENAME", asynParamOctet, &filenameIndex_);
    createParam("HISTOGRAM_DATA", asynParamInt32Array, &histogramDataIndex_);
    
    // Set initial values
    setIntegerParam(connectedIndex_, 0);
    setIntegerParam(frameCountIndex_, 0);
    setIntegerParam(totalCountsIndex_, 0);
    setIntegerParam(errorCountIndex_, 0);
    setDoubleParam(acquisitionRateIndex_, 0.0);
    setDoubleParam(processingTimeIndex_, 0.0);
    setDoubleParam(memoryUsageIndex_, 0.0);
    setDoubleParam(binWidthIndex_, 0.260);
    setDoubleParam(totalTimeIndex_, 260.0);
    setStringParam(statusIndex_, "Initialized");
    
    // Start monitor thread
    monitorThreadId_ = epicsThreadCreate("tpx3HistogramMonitor",
                                        epicsThreadPriorityMedium,
                                        epicsThreadGetStackSize(epicsThreadStackMedium),
                                        monitorThreadC,
                                        this);
    
    printf("Timepix3 Histogram Driver initialized on port %s\n", portName);
}

// Destructor
tpx3HistogramDriver::~tpx3HistogramDriver()
{
    if (running_) {
        stop();
    }
    
    if (connected_) {
        disconnect();
    }
    
    if (mutex_) {
        epicsMutexDestroy(mutex_);
    }
    
    if (dataReady_) {
        epicsEventDestroy(dataReady_);
    }
}

// Write methods
asynStatus tpx3HistogramDriver::writeInt32(asynUser *pasynUser, epicsInt32 value)
{
    int function = pasynUser->reason;
    asynStatus status = asynSuccess;
    
    if (function == connectIndex_) {
        if (value == 1) {
            connect();
        }
    } else if (function == disconnectIndex_) {
        if (value == 1) {
            disconnect();
        }
    } else if (function == resetIndex_) {
        if (value == 1) {
            reset();
        }
    } else if (function == startIndex_) {
        if (value == 1) {
            start();
        }
    } else if (function == stopIndex_) {
        if (value == 1) {
            stop();
        }
    } else if (function == saveDataIndex_) {
        if (value == 1) {
            // Get filename from parameter
            char filename[256];
            getStringParam(filenameIndex_, sizeof(filename), filename);
            saveData(filename);
        }
    } else if (function == portIndex_) {
        port_ = value;
        setIntegerParam(portIndex_, value);
    } else {
        status = asynPortDriver::writeInt32(pasynUser, value);
    }
    
    callParamCallbacks();
    return status;
}

asynStatus tpx3HistogramDriver::writeOctet(asynUser *pasynUser, const char *value, size_t maxChars, size_t *nActual)
{
    int function = pasynUser->reason;
    asynStatus status = asynSuccess;
    
    if (function == hostIndex_) {
        host_ = std::string(value, maxChars);
        setStringParam(hostIndex_, value);
    } else if (function == filenameIndex_) {
        setStringParam(filenameIndex_, value);
    } else {
        status = asynPortDriver::writeOctet(pasynUser, value, maxChars, nActual);
    }
    
    callParamCallbacks();
    return status;
}

// Read methods
asynStatus tpx3HistogramDriver::readInt32(asynUser *pasynUser, epicsInt32 *value)
{
    int function = pasynUser->reason;
    asynStatus status = asynSuccess;
    
    if (function == connectedIndex_) {
        *value = connected_ ? 1 : 0;
    } else if (function == frameCountIndex_) {
        *value = static_cast<epicsInt32>(frame_count_);
    } else if (function == totalCountsIndex_) {
        *value = static_cast<epicsInt32>(total_counts_);
    } else if (function == errorCountIndex_) {
        *value = error_count_;
    } else {
        status = asynPortDriver::readInt32(pasynUser, value);
    }
    
    return status;
}

asynStatus tpx3HistogramDriver::readOctet(asynUser *pasynUser, char *value, size_t maxChars, size_t *nActual, int *eomReason)
{
    int function = pasynUser->reason;
    asynStatus status = asynSuccess;
    
    if (function == statusIndex_) {
        strncpy(value, status_.c_str(), maxChars);
        *nActual = std::min(status_.length(), maxChars);
        *eomReason = ASYN_EOM_END;
    } else {
        status = asynPortDriver::readOctet(pasynUser, value, maxChars, nActual, eomReason);
    }
    
    return status;
}

// Public methods
void tpx3HistogramDriver::connect()
{
    epicsMutexLock(mutex_);
    
    if (!connected_) {
        // Simulate connection (in real implementation, this would connect to Timepix3 server)
        connected_ = true;
        status_ = "Connected";
        setIntegerParam(connectedIndex_, 1);
        setStringParam(statusIndex_, status_.c_str());
        printf("Connected to Timepix3 server at %s:%d\n", host_.c_str(), port_);
    }
    
    epicsMutexUnlock(mutex_);
    callParamCallbacks();
}

void tpx3HistogramDriver::disconnect()
{
    epicsMutexLock(mutex_);
    
    if (connected_) {
        if (running_) {
            stop();
        }
        
        connected_ = false;
        status_ = "Disconnected";
        setIntegerParam(connectedIndex_, 0);
        setStringParam(statusIndex_, status_.c_str());
        printf("Disconnected from Timepix3 server\n");
    }
    
    epicsMutexUnlock(mutex_);
    callParamCallbacks();
}

void tpx3HistogramDriver::reset()
{
    epicsMutexLock(mutex_);
    
    frame_count_ = 0;
    total_counts_ = 0;
    std::fill(bin_values_.begin(), bin_values_.end(), 0);
    
    setIntegerParam(frameCountIndex_, 0);
    setIntegerParam(totalCountsIndex_, 0);
    status_ = "Reset";
    setStringParam(statusIndex_, status_.c_str());
    
    epicsMutexUnlock(mutex_);
    callParamCallbacks();
    printf("Histogram data reset\n");
}

void tpx3HistogramDriver::start()
{
    epicsMutexLock(mutex_);
    
    if (connected_ && !running_) {
        running_ = true;
        status_ = "Running";
        setStringParam(statusIndex_, status_.c_str());
        
        // Start worker thread
        workerThreadId_ = epicsThreadCreate("tpx3HistogramWorker",
                                          epicsThreadPriorityMedium,
                                          epicsThreadGetStackSize(epicsThreadStackMedium),
                                          workerThreadC,
                                          this);
        
        printf("Started histogram acquisition\n");
    }
    
    epicsMutexUnlock(mutex_);
    callParamCallbacks();
}

void tpx3HistogramDriver::stop()
{
    epicsMutexLock(mutex_);
    
    if (running_) {
        running_ = false;
        status_ = "Stopped";
        setStringParam(statusIndex_, status_.c_str());
        
        // Signal worker thread to stop
        epicsEventSignal(dataReady_);
        
        printf("Stopped histogram acquisition\n");
    }
    
    epicsMutexUnlock(mutex_);
    callParamCallbacks();
}

void tpx3HistogramDriver::saveData(const std::string& filename)
{
    epicsMutexLock(mutex_);
    
    // In real implementation, this would save histogram data to file
    status_ = "Data saved to " + filename;
    setStringParam(statusIndex_, status_.c_str());
    
    epicsMutexUnlock(mutex_);
    callParamCallbacks();
    printf("Histogram data saved to %s\n", filename.c_str());
}

// Thread methods
void tpx3HistogramDriver::workerThread()
{
    while (running_) {
        if (connected_) {
            // Simulate histogram data processing
            epicsMutexLock(mutex_);
            
            // Simulate frame processing
            frame_count_++;
            total_counts_ += 1000; // Simulate counts per frame
            
            // Simulate bin updates
            for (size_t i = 0; i < bin_size_; ++i) {
                bin_values_[i] += (i % 10) + 1; // Simulate bin values
            }
            
            // Update parameters
            setIntegerParam(frameCountIndex_, static_cast<epicsInt32>(frame_count_));
            setIntegerParam(totalCountsIndex_, static_cast<epicsInt32>(total_counts_));
            
            epicsMutexUnlock(mutex_);
            
            // Signal data ready
            epicsEventSignal(dataReady_);
            
            // Update callbacks
            callParamCallbacks();
        }
        
        epicsThreadSleep(1.0); // 1 second interval
    }
}

void tpx3HistogramDriver::monitorThread()
{
    while (true) {
        // Update status and performance metrics
        epicsMutexLock(mutex_);
        
        // Simulate performance metrics
        if (running_ && connected_) {
            acquisition_rate_ = 1.0; // 1 frame per second
            processing_time_ = 0.1;  // 100ms
            memory_usage_ = 50.0;    // 50MB
        } else {
            acquisition_rate_ = 0.0;
            processing_time_ = 0.0;
            memory_usage_ = 10.0;
        }
        
        setDoubleParam(acquisitionRateIndex_, acquisition_rate_);
        setDoubleParam(processingTimeIndex_, processing_time_);
        setDoubleParam(memoryUsageIndex_, memory_usage_);
        
        epicsMutexUnlock(mutex_);
        
        callParamCallbacks();
        epicsThreadSleep(5.0); // Update every 5 seconds
    }
}

// Static thread functions
void tpx3HistogramDriver::workerThreadC(void *pPvt)
{
    tpx3HistogramDriver* pDriver = (tpx3HistogramDriver*)pPvt;
    pDriver->workerThread();
}

void tpx3HistogramDriver::monitorThreadC(void *pPvt)
{
    tpx3HistogramDriver* pDriver = (tpx3HistogramDriver*)pPvt;
    pDriver->monitorThread();
}

// Global functions
extern "C" tpx3HistogramDriver* getTpx3HistogramDriver()
{
    return g_driver;
}

extern "C" void performTpx3HistogramCleanup()
{
    if (g_driver) {
        delete g_driver;
        g_driver = NULL;
    }
}

// Configuration function
extern "C" int tpx3HistogramConfigure(const char* portName, int maxAddr)
{
    if (g_driver) {
        printf("Driver already configured\n");
        return -1;
    }
    
    g_driver = new tpx3HistogramDriver(portName, maxAddr);
    if (!g_driver) {
        printf("Timepix3 Histogram Driver configured on port %s\n", portName);
        return 0;
    }
    
    printf("Timepix3 Histogram Driver configured on port %s\n", portName);
    return 0;
}

// iocsh registration
static const iocshArg tpx3HistogramConfigureArg0 = {"portName", iocshArgString};
static const iocshArg tpx3HistogramConfigureArg1 = {"maxAddr", iocshArgInt};
static const iocshArg * const tpx3HistogramConfigureArgs[] = {&tpx3HistogramConfigureArg0, &tpx3HistogramConfigureArg1};
static const iocshFuncDef tpx3HistogramConfigureFuncDef = {"tpx3HistogramConfigure", 2, tpx3HistogramConfigureArgs};

static void tpx3HistogramConfigureCallFunc(const iocshArgBuf *args)
{
    tpx3HistogramConfigure(args[0].sval, args[1].ival);
}

// Registration function
extern "C" void register_func_tpx3HistogramConfigure(void)
{
    // This function is called by the registrar directive
    // It registers the tpx3HistogramConfigure function
    iocshRegister(&tpx3HistogramConfigureFuncDef, tpx3HistogramConfigureCallFunc);
    printf("Timepix3 Histogram device support registered\n");
}

// Export the registrar function
epicsExportRegistrar(register_func_tpx3HistogramConfigure);

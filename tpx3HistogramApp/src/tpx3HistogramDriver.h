#ifndef tpx3HistogramDriver_H
#define tpx3HistogramDriver_H

#include <asynPortDriver.h>
#include <epicsMutex.h>
#include <epicsEvent.h>
#include <epicsThread.h>
#include <epicsTime.h>
#include <string>
#include <vector>
#include <cstdint>

// Constants
constexpr size_t MAX_BINS = 1000;
constexpr int DEFAULT_PORT = 8451;
constexpr const char* DEFAULT_HOST = "127.0.0.1";

class tpx3HistogramDriver : public asynPortDriver {
public:
    tpx3HistogramDriver(const char *portName, int maxAddr);
    virtual ~tpx3HistogramDriver();

    // asynPortDriver methods
    virtual asynStatus writeInt32(asynUser *pasynUser, epicsInt32 value);
    virtual asynStatus writeOctet(asynUser *pasynUser, const char *value, size_t maxChars, size_t *nActual);
    virtual asynStatus readInt32(asynUser *pasynUser, epicsInt32 *value);
    virtual asynStatus readOctet(asynUser *pasynUser, char *value, size_t maxChars, size_t *nActual, int *eomReason);

    // Public methods
    void connect();
    void disconnect();
    void reset();
    void start();
    void stop();
    void saveData(const std::string& filename);
    
    // Status methods
    bool isConnected() const { return connected_; }
    uint64_t getFrameCount() const { return frame_count_; }
    uint64_t getTotalCounts() const { return total_counts_; }
    const std::vector<uint64_t>& getBinValues() const { return bin_values_; }

private:
    // Parameter indices
    int connectIndex_;
    int disconnectIndex_;
    int resetIndex_;
    int startIndex_;
    int stopIndex_;
    int saveDataIndex_;
    int hostIndex_;
    int portIndex_;
    int frameCountIndex_;
    int totalCountsIndex_;
    int connectedIndex_;
    int statusIndex_;
    int errorCountIndex_;
    int acquisitionRateIndex_;
    int processingTimeIndex_;
    int memoryUsageIndex_;
    int binWidthIndex_;
    int totalTimeIndex_;
    int filenameIndex_;
    int histogramDataIndex_;

    // Network and data
    std::string host_;
    int port_;
    bool connected_;
    bool running_;
    
    // Histogram data
    uint64_t frame_count_;
    uint64_t total_counts_;
    std::vector<uint64_t> bin_values_;
    std::vector<double> bin_edges_;
    size_t bin_size_;
    
    // Threading and synchronization
    epicsMutexId mutex_;
    epicsEventId dataReady_;
    epicsThreadId workerThreadId_;
    epicsThreadId monitorThreadId_;
    
    // Status and error handling
    std::string status_;
    int error_count_;
    double acquisition_rate_;
    double processing_time_;
    double memory_usage_;
    
    // Methods
    void workerThread();
    void monitorThread();
    static void workerThreadC(void *pPvt);
    static void monitorThreadC(void *pPvt);
    void updateStatus();
    void setError(const char *errorMsg);
    void processHistogramData();
    void updateBinValues();
};

// Function to get driver instance
extern "C" tpx3HistogramDriver* getTpx3HistogramDriver();

// Function to perform cleanup
extern "C" void performTpx3HistogramCleanup();

#endif // tpx3HistogramDriver_H

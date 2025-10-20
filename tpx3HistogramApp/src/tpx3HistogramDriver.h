/*
 * tpx3HistogramDriver.h
 * 
 * Timepix3 Histogram IOC Driver Header
 * 
 * Author: K. Gofron
 * Last Modified: October 20, 2025
 * 
 * This header defines the tpx3HistogramDriver class for EPICS integration
 * with Timepix3 histogram data acquisition systems.
 */

#ifndef tpx3HistogramDriver_H
#define tpx3HistogramDriver_H

#include <asynPortDriver.h>
#include <epicsMutex.h>
#include <epicsEvent.h>
#include <epicsThread.h>
#include <string>
#include <vector>
#include <cstdint>
#include <atomic>
#include <mutex>
#include <memory>
#include <queue>
#include <functional>
#include <stdexcept>
#include <system_error>
#include <cstring>
#include <iomanip>
#include <filesystem>

// Network includes
#include <sys/socket.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>

// JSON parsing
#include <nlohmann/json.hpp>

// Use nlohmann namespace for convenience
using json = nlohmann::json;

// Forward declarations
class HistogramData;
class NetworkClient;

/**
 * @brief Represents histogram data with bin edges and values
 */
class HistogramData {
public:
    enum class DataType {
        FRAME_DATA,      // Individual frame data (32-bit)
        RUNNING_SUM      // Accumulated data (64-bit)
    };

    HistogramData(size_t bin_size, DataType type = DataType::FRAME_DATA);
    HistogramData(const HistogramData& other);
    HistogramData(HistogramData&& other) noexcept;
    HistogramData& operator=(const HistogramData& other);
    HistogramData& operator=(HistogramData&& other) noexcept;
    ~HistogramData() = default;

    // Getters
    size_t get_bin_size() const { return bin_size_; }
    DataType get_data_type() const { return data_type_; }
    const std::vector<double>& get_bin_edges() const { return bin_edges_; }
    
    // Access bin values based on type
    uint32_t get_bin_value_32(size_t index) const;
    uint64_t get_bin_value_64(size_t index) const;

    // Setters
    void set_bin_edge(size_t index, double value);
    void set_bin_value_32(size_t index, uint32_t value);
    void set_bin_value_64(size_t index, uint64_t value);

    // Calculate bin edges from parameters
    void calculate_bin_edges(int bin_width, int bin_offset);

    // Add another histogram to this one (for running sum)
    void add_histogram(const HistogramData& other);

private:
    size_t bin_size_;
    DataType data_type_;
    std::vector<double> bin_edges_;
    std::vector<uint32_t> bin_values_32_;
    std::vector<uint64_t> bin_values_64_;
};

/**
 * @brief Network client for TCP socket communication
 */
class NetworkClient {
public:
    NetworkClient();
    ~NetworkClient();

    // Disable copy
    NetworkClient(const NetworkClient&) = delete;
    NetworkClient& operator=(const NetworkClient&) = delete;

    // Allow move
    NetworkClient(NetworkClient&& other) noexcept;
    NetworkClient& operator=(NetworkClient&& other) noexcept;

    /**
     * @brief Connect to server
     * @param host Server hostname/IP
     * @param port Server port
     * @return true if successful, false otherwise
     */
    bool connect(const std::string& host, int port);

    /**
     * @brief Disconnect from server
     */
    void disconnect();

    /**
     * @brief Check if connected
     * @return true if connected
     */
    bool is_connected() const { return connected_; }

    /**
     * @brief Receive data from socket
     * @param buffer Buffer to store received data
     * @param max_size Maximum size to receive
     * @return Number of bytes received, -1 on error, 0 on connection closed
     */
    ssize_t receive(char* buffer, size_t max_size);

    /**
     * @brief Receive exact amount of data
     * @param buffer Buffer to store received data
     * @param size Exact size to receive
     * @return true if successful, false otherwise
     */
    bool receive_exact(char* buffer, size_t size);

private:
    int socket_fd_;
    bool connected_;
};

// Constants
constexpr double TPX3_TDC_CLOCK_PERIOD_SEC = (1.5625 / 6.0) * 1e-9;
constexpr size_t MAX_BUFFER_SIZE = 32768;
// MAX_BINS is now configurable via the maxBinsIndex_ parameter
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
    virtual asynStatus readInt64(asynUser *pasynUser, epicsInt64 *value);
    virtual asynStatus readOctet(asynUser *pasynUser, char *value, size_t maxChars, size_t *nActual, int *eomReason);
    virtual asynStatus readFloat64(asynUser *pasynUser, epicsFloat64 *value);
    virtual asynStatus readInt32Array(asynUser *pasynUser, epicsInt32 *value, size_t nElements, size_t *nIn);

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
    const HistogramData* getRunningSum() const { return running_sum_.get(); }

private:
    // Parameter indices
    int connectionStateIndex_;
    int resetIndex_;
    int acquisitionStateIndex_;
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
    int histogramFrameIndex_;    // Individual frame histogram data
    int histogramTimeMsIndex_;   // Time axis for histogram in milliseconds
    int numberOfBinsIndex_;
    int maxBinsIndex_;           // Maximum number of bins for array record
    // Individual bin display parameters removed - use HISTOGRAM_DATA and HISTOGRAM_FRAME arrays instead
    
    // Frame data parameters from JSON
    int timeAtFrameIndex_;       // Timestamp at frame in nanoseconds
    int frameBinSizeIndex_;      // Number of bins in current frame
    int frameBinWidthIndex_;     // Bin width parameter from frame
    int frameBinOffsetIndex_;    // Bin offset parameter from frame

    // Network and data
    std::string host_;
    int port_;
    bool connected_;
    bool running_;
    std::unique_ptr<NetworkClient> network_client_;
    
    // Histogram data
    uint64_t frame_count_;
    uint64_t total_counts_;
    std::unique_ptr<HistogramData> running_sum_;
    std::vector<char> line_buffer_;
    size_t total_read_;
    
    // Processing parameters
    int bin_width_;
    int bin_offset_;
    int number_of_bins_;
    
    // Frame data from JSON
    double time_at_frame_;
    int frame_bin_size_;
    int frame_bin_width_;
    int frame_bin_offset_;
    
    // Time axis data for histogram plotting (in milliseconds)
    std::vector<epicsFloat64> histogram_time_data_;
    
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
    
    // Configurable parameters
    int max_bins_;
    
    // Frame rate calculation variables
    int previous_frame_number_;
    double previous_time_at_frame_;
    bool first_frame_received_;
    
    // Acquisition rate averaging variables
    std::vector<double> rate_samples_;
    double last_rate_update_time_;
    static const size_t MAX_RATE_SAMPLES = 10;  // Keep last 10 samples for averaging
    
    // Methods
    void workerThread();
    void monitorThread();
    static void workerThreadC(void *pPvt);
    static void monitorThreadC(void *pPvt);
    void updateStatus();
    void setError(const char *errorMsg);
    void processHistogramData();
    void updateBinValues();
    
    // New histogram processing methods
    bool processDataLine(char* line_buffer, char* newline_pos, size_t total_read);
    void processFrame(const HistogramData& frame_data);
    void saveRunningSum();
    void saveHistogramToFile(const std::string& filename, const HistogramData& histogram);
};

// Function to get driver instance
extern "C" tpx3HistogramDriver* getTpx3HistogramDriver();

// Function to perform cleanup
extern "C" void performTpx3HistogramCleanup();

#endif // tpx3HistogramDriver_H

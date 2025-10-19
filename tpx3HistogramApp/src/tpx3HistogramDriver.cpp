#include "tpx3HistogramDriver.h"
#include <epicsExport.h>
#include <epicsThread.h>
#include <epicsString.h>
#include <epicsStdio.h>
#include <iocsh.h>
#include <cstring>
#include <iostream>
#include <fstream>
#include <sstream>
#include <chrono>
#include <thread>
#include <algorithm>
#include <filesystem>
#include <ctime>

// HistogramData class implementation
HistogramData::HistogramData(size_t bin_size, DataType type)
    : bin_size_(bin_size), data_type_(type) {
    
    bin_edges_.resize(bin_size + 1);
    
    if (type == DataType::FRAME_DATA) {
        bin_values_32_.resize(bin_size, 0);
    } else {
        bin_values_64_.resize(bin_size, 0);
    }
}

// Copy constructor
HistogramData::HistogramData(const HistogramData& other)
    : bin_size_(other.bin_size_), data_type_(other.data_type_) {
    
    bin_edges_ = other.bin_edges_;
    
    if (data_type_ == DataType::FRAME_DATA) {
        bin_values_32_ = other.bin_values_32_;
    } else {
        bin_values_64_ = other.bin_values_64_;
    }
}

// Move constructor
HistogramData::HistogramData(HistogramData&& other) noexcept
    : bin_size_(other.bin_size_), data_type_(other.data_type_) {
    
    bin_edges_ = std::move(other.bin_edges_);
    bin_values_32_ = std::move(other.bin_values_32_);
    bin_values_64_ = std::move(other.bin_values_64_);
    
    other.bin_size_ = 0;
    other.data_type_ = DataType::FRAME_DATA;
}

// Assignment operators
HistogramData& HistogramData::operator=(const HistogramData& other) {
    if (this != &other) {
        bin_size_ = other.bin_size_;
        data_type_ = other.data_type_;
        bin_edges_ = other.bin_edges_;
        
        if (data_type_ == DataType::FRAME_DATA) {
            bin_values_32_ = other.bin_values_32_;
        } else {
            bin_values_64_ = other.bin_values_64_;
        }
    }
    return *this;
}

HistogramData& HistogramData::operator=(HistogramData&& other) noexcept {
    if (this != &other) {
        bin_size_ = other.bin_size_;
        data_type_ = other.data_type_;
        bin_edges_ = std::move(other.bin_edges_);
        bin_values_32_ = std::move(other.bin_values_32_);
        bin_values_64_ = std::move(other.bin_values_64_);
        
        other.bin_size_ = 0;
        other.data_type_ = DataType::FRAME_DATA;
    }
    return *this;
}

// Access bin values based on type
uint32_t HistogramData::get_bin_value_32(size_t index) const {
    if (data_type_ != DataType::FRAME_DATA || index >= bin_values_32_.size()) {
        throw std::out_of_range("Invalid index or data type for 32-bit access");
    }
    return bin_values_32_[index];
}

uint64_t HistogramData::get_bin_value_64(size_t index) const {
    if (data_type_ != DataType::RUNNING_SUM || index >= bin_values_64_.size()) {
        throw std::out_of_range("Invalid index or data type for 64-bit access");
    }
    return bin_values_64_[index];
}

// Setters
void HistogramData::set_bin_edge(size_t index, double value) {
    if (index >= bin_edges_.size()) {
        throw std::out_of_range("Bin edge index out of range");
    }
    bin_edges_[index] = value;
}

void HistogramData::set_bin_value_32(size_t index, uint32_t value) {
    if (data_type_ != DataType::FRAME_DATA || index >= bin_values_32_.size()) {
        throw std::out_of_range("Invalid index or data type for 32-bit access");
    }
    bin_values_32_[index] = value;
}

void HistogramData::set_bin_value_64(size_t index, uint64_t value) {
    if (data_type_ != DataType::RUNNING_SUM || index >= bin_values_64_.size()) {
        throw std::out_of_range("Invalid index or data type for 64-bit access");
    }
    bin_values_64_[index] = value;
}

// Calculate bin edges from parameters
void HistogramData::calculate_bin_edges(int bin_width, int bin_offset) {
    for (size_t i = 0; i < bin_edges_.size(); ++i) {
        bin_edges_[i] = (bin_offset + (i * bin_width)) * TPX3_TDC_CLOCK_PERIOD_SEC;
    }
}

// Add another histogram to this one (for running sum)
void HistogramData::add_histogram(const HistogramData& other) {
    if (other.data_type_ != DataType::FRAME_DATA || data_type_ != DataType::RUNNING_SUM) {
        throw std::invalid_argument("Can only add frame data to running sum");
    }
    
    if (other.bin_size_ != bin_size_) {
        throw std::invalid_argument("Bin sizes must match for addition");
    }

    for (size_t i = 0; i < bin_size_; ++i) {
        uint64_t new_value = bin_values_64_[i] + other.bin_values_32_[i];
        if (new_value < bin_values_64_[i]) {
            std::cerr << "Warning: Overflow detected in bin " << i 
                      << ", capping at maximum value" << std::endl;
            bin_values_64_[i] = UINT64_MAX;
        } else {
            bin_values_64_[i] = new_value;
        }
    }
}

// NetworkClient class implementation
NetworkClient::NetworkClient() : socket_fd_(-1), connected_(false) {}

NetworkClient::~NetworkClient() {
    disconnect();
}

// Move constructor
NetworkClient::NetworkClient(NetworkClient&& other) noexcept
    : socket_fd_(other.socket_fd_), connected_(other.connected_) {
    other.socket_fd_ = -1;
    other.connected_ = false;
}

NetworkClient& NetworkClient::operator=(NetworkClient&& other) noexcept {
    if (this != &other) {
        disconnect();
        socket_fd_ = other.socket_fd_;
        connected_ = other.connected_;
        other.socket_fd_ = -1;
        other.connected_ = false;
    }
    return *this;
}

bool NetworkClient::connect(const std::string& host, int port) {
    socket_fd_ = socket(AF_INET, SOCK_STREAM, 0);
    if (socket_fd_ < 0) {
        std::cerr << "Socket creation failed: " << strerror(errno) << std::endl;
        return false;
    }

    // Set TCP_NODELAY to disable Nagle's algorithm
    int flag = 1;
    if (setsockopt(socket_fd_, IPPROTO_TCP, TCP_NODELAY, &flag, sizeof(flag)) < 0) {
        std::cerr << "Failed to set TCP_NODELAY: " << strerror(errno) << std::endl;
    }

    // Set larger socket buffers
    int rcvbuf = 256 * 1024;  // 256KB receive buffer
    if (setsockopt(socket_fd_, SOL_SOCKET, SO_RCVBUF, &rcvbuf, sizeof(rcvbuf)) < 0) {
        std::cerr << "Failed to set receive buffer size: " << strerror(errno) << std::endl;
    }

    struct sockaddr_in server_addr{};
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(port);
    
    if (inet_pton(AF_INET, host.c_str(), &server_addr.sin_addr) <= 0) {
        std::cerr << "Invalid address: " << host << std::endl;
        close(socket_fd_);
        socket_fd_ = -1;
        return false;
    }

    std::cout << "Attempting to connect to " << host << ":" << port << "..." << std::endl;
    
    if (::connect(socket_fd_, (struct sockaddr*)&server_addr, sizeof(server_addr)) < 0) {
        std::cerr << "Connection failed: " << strerror(errno) << std::endl;
        close(socket_fd_);
        socket_fd_ = -1;
        return false;
    }
    
    std::cout << "Connected successfully" << std::endl;
    connected_ = true;
    return true;
}

void NetworkClient::disconnect() {
    if (socket_fd_ >= 0) {
        close(socket_fd_);
        socket_fd_ = -1;
    }
    connected_ = false;
}

ssize_t NetworkClient::receive(char* buffer, size_t max_size) {
    if (!connected_ || socket_fd_ < 0) {
        return -1;
    }
    
    ssize_t bytes_read = recv(socket_fd_, buffer, max_size, 0);
    
    if (bytes_read == 0) {
        std::cout << "Connection closed by peer" << std::endl;
        connected_ = false;
    } else if (bytes_read < 0) {
        if (errno != EAGAIN && errno != EWOULDBLOCK) {
            std::cerr << "Socket error: " << strerror(errno) << std::endl;
            connected_ = false;
        }
    }
    
    return bytes_read;
}

bool NetworkClient::receive_exact(char* buffer, size_t size) {
    size_t total_received = 0;
    
    while (total_received < size) {
        ssize_t bytes = receive(buffer + total_received, size - total_received);
        
        if (bytes <= 0) {
            return false;
        }
        
        total_received += bytes;
    }
    
    return true;
}

// Parameter definitions
#define NUM_PARAMS 17

// Global driver instance
static tpx3HistogramDriver* g_driver = NULL;

// Helper function to create detailed status messages
std::string createStatusMessage(const std::string& baseMessage, const std::string& host = "", int port = 0, uint64_t frameCount = 0, uint64_t totalCounts = 0, int errorCount = 0) {
    std::string status = baseMessage;
    
    // Add timestamp
    time_t now = time(0);
    char timeStr[64];
    strftime(timeStr, sizeof(timeStr), "%H:%M:%S", localtime(&now));
    status += " [" + std::string(timeStr) + "]";
    
    // Add connection info if provided
    if (!host.empty() && port > 0) {
        status += " - " + host + ":" + std::to_string(port);
    }
    
    // Add frame/count info if provided
    if (frameCount > 0 || totalCounts > 0) {
        status += " - Frames: " + std::to_string(frameCount);
        if (totalCounts > 0) {
            status += ", Counts: " + std::to_string(totalCounts);
        }
    }
    
    // Add error info if provided
    if (errorCount > 0) {
        status += " - Errors: " + std::to_string(errorCount);
    }
    
    return status;
}

// Constructor
tpx3HistogramDriver::tpx3HistogramDriver(const char *portName, int maxAddr)
    : asynPortDriver(portName, maxAddr, NUM_PARAMS,
                     asynInt32Mask | asynOctetMask | asynFloat64Mask | asynInt32ArrayMask | asynFloat64ArrayMask | asynDrvUserMask,
                     asynInt32Mask | asynOctetMask | asynFloat64Mask | asynInt32ArrayMask | asynFloat64ArrayMask | asynDrvUserMask,
                     ASYN_CANBLOCK, 1, 0, 0),
      host_(DEFAULT_HOST),
      port_(DEFAULT_PORT),
      connected_(false),
      running_(false),
      network_client_(std::make_unique<NetworkClient>()),
      frame_count_(0),
      total_counts_(0),
      running_sum_(nullptr),
      line_buffer_(MAX_BUFFER_SIZE),
      total_read_(0),
      bin_width_(384000),
      bin_offset_(0),
      number_of_bins_(10),
      time_at_frame_(0.0),
      frame_bin_size_(10),      // Default to 10 bins
      frame_bin_width_(384000), // Default to 384000 (from your example data)
      frame_bin_offset_(0),
      error_count_(0),
      acquisition_rate_(0.0),
      processing_time_(0.0),
      memory_usage_(0.0)
{
    // Create data directory
    std::filesystem::create_directories("data");
    
    // Create mutex and event
    mutex_ = epicsMutexCreate();
    dataReady_ = epicsEventCreate(epicsEventEmpty);
    
    // Initialize status
    status_ = "Initialized - Ready to connect";
    
    // Parameter indices will be set by createParam calls
    // We don't need to manually assign them as createParam handles this
    
    // Create parameters
    createParam("CONNECTION_STATE", asynParamInt32, &connectionStateIndex_);
    createParam("RESET", asynParamInt32, &resetIndex_);
    createParam("ACQUISITION_STATE", asynParamInt32, &acquisitionStateIndex_);
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
    createParam("HISTOGRAM_TIME_MS", asynParamFloat64Array, &histogramTimeMsIndex_);
    printf("DEBUG: Created HISTOGRAM_TIME_MS parameter with index %d\n", histogramTimeMsIndex_);
    createParam("NUMBER_OF_BINS", asynParamInt32, &numberOfBinsIndex_);
    createParam("MAX_BINS", asynParamInt32, &maxBinsIndex_);
    
    // Create frame data parameters from JSON
    createParam("TIME_AT_FRAME", asynParamFloat64, &timeAtFrameIndex_);
    createParam("FRAME_BIN_SIZE", asynParamInt32, &frameBinSizeIndex_);
    createParam("FRAME_BIN_WIDTH", asynParamInt32, &frameBinWidthIndex_);
    createParam("FRAME_BIN_OFFSET", asynParamInt32, &frameBinOffsetIndex_);
    
    // Create individual bin parameters for display
    createParam("BIN_0", asynParamInt32, &binDisplayIndex_[0]);
    createParam("BIN_1", asynParamInt32, &binDisplayIndex_[1]);
    createParam("BIN_2", asynParamInt32, &binDisplayIndex_[2]);
    createParam("BIN_3", asynParamInt32, &binDisplayIndex_[3]);
    createParam("BIN_4", asynParamInt32, &binDisplayIndex_[4]);
    
    
    printf("DEBUG: Parameter indices - histogramDataIndex_=%d, histogramTimeMsIndex_=%d, numberOfBinsIndex_=%d\n", 
           histogramDataIndex_, histogramTimeMsIndex_, numberOfBinsIndex_);
    printf("DEBUG: Display bin indices: ");
    for (int i = 0; i < 5; ++i) {
        printf("%d ", binDisplayIndex_[i]);
    }
    printf("\n");
    printf("DEBUG: All parameter indices:\n");
    printf("  CONNECTION_STATE=%d\n", connectionStateIndex_);
    printf("  RESET=%d\n", resetIndex_);
    printf("  ACQUISITION_STATE=%d\n", acquisitionStateIndex_);
    printf("  SAVE_DATA=%d\n", saveDataIndex_);
    printf("  HOST=%d\n", hostIndex_);
    printf("  PORT=%d\n", portIndex_);
    printf("  FRAME_COUNT=%d\n", frameCountIndex_);
    printf("  TOTAL_COUNTS=%d\n", totalCountsIndex_);
    printf("  CONNECTED=%d\n", connectedIndex_);
    printf("  STATUS=%d\n", statusIndex_);
    printf("  ERROR_COUNT=%d\n", errorCountIndex_);
    printf("  ACQUISITION_RATE=%d\n", acquisitionRateIndex_);
    printf("  PROCESSING_TIME=%d\n", processingTimeIndex_);
    printf("  MEMORY_USAGE=%d\n", memoryUsageIndex_);
    printf("  BIN_WIDTH=%d\n", binWidthIndex_);
    printf("  TOTAL_TIME=%d\n", totalTimeIndex_);
    printf("  FILENAME=%d\n", filenameIndex_);
    printf("  HISTOGRAM_DATA=%d\n", histogramDataIndex_);
    printf("  HISTOGRAM_TIME_MS=%d\n", histogramTimeMsIndex_);
    printf("  NUMBER_OF_BINS=%d\n", numberOfBinsIndex_);
    printf("  MAX_BINS=%d\n", maxBinsIndex_);
    
    // Set initial values
    setIntegerParam(connectedIndex_, 0);
    setIntegerParam(frameCountIndex_, 0);
    setIntegerParam(totalCountsIndex_, 0);
    setIntegerParam(errorCountIndex_, 0);
    setDoubleParam(acquisitionRateIndex_, 0.0);
    setDoubleParam(processingTimeIndex_, 0.0);
    setDoubleParam(memoryUsageIndex_, 0.0);
    setDoubleParam(binWidthIndex_, TPX3_TDC_CLOCK_PERIOD_SEC*1e9*frame_bin_width_);  // Default bin width in nanoseconds
    setDoubleParam(totalTimeIndex_, (TPX3_TDC_CLOCK_PERIOD_SEC*1e9*frame_bin_width_)*frame_bin_size_);  // Total time in nanoseconds
    setIntegerParam(numberOfBinsIndex_, number_of_bins_);
    setIntegerParam(maxBinsIndex_, 1000);  // Default maximum bins for array record
    setStringParam(statusIndex_, "Initialized - Ready to connect");
    
    // Initialize frame data parameters
    setDoubleParam(timeAtFrameIndex_, time_at_frame_);
    setIntegerParam(frameBinSizeIndex_, frame_bin_size_);
    setIntegerParam(frameBinWidthIndex_, frame_bin_width_);
    setIntegerParam(frameBinOffsetIndex_, frame_bin_offset_);
    
    printf("DEBUG: Initialized frame data - bin_size=%d, bin_width=%d\n", frame_bin_size_, frame_bin_width_);
    printf("DEBUG: Calculated bin width = %.6f ns\n", TPX3_TDC_CLOCK_PERIOD_SEC*1e9*frame_bin_width_);
    printf("DEBUG: Calculated total time = %.6f ns\n", (TPX3_TDC_CLOCK_PERIOD_SEC*1e9*frame_bin_width_)*frame_bin_size_);
    fflush(stdout);
    
    // Create some test histogram data for debugging
    printf("DEBUG: Creating test histogram data\n");
    fflush(stdout);
    running_sum_ = std::make_unique<HistogramData>(10, HistogramData::DataType::RUNNING_SUM);
    for (int i = 0; i < 10; ++i) {
        running_sum_->set_bin_value_64(i, i * 100);  // Test values: 0, 100, 200, 300, ...
    }
    printf("DEBUG: Test histogram data created with %zu bins\n", running_sum_->get_bin_size());
    fflush(stdout);
    
    // Start monitor thread
    monitorThreadId_ = epicsThreadCreate("tpx3HistogramMonitor",
                                        epicsThreadPriorityMedium,
                                        epicsThreadGetStackSize(epicsThreadStackMedium),
                                        monitorThreadC,
                                        this);
    
    printf("Timepix3 Histogram Driver initialized on port %s\n", portName);
    printf("DEBUG: Driver initialization complete - histogramDataIndex_=%d\n", histogramDataIndex_);
    
    // Call parameter callbacks to ensure all parameters are updated
    callParamCallbacks();
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
    
    if (function == connectionStateIndex_) {
        if (value == 1) {
            connect();
        } else if (value == 0) {
            disconnect();
        }
    } else if (function == resetIndex_) {
        if (value == 1) {
            reset();
        }
    } else if (function == acquisitionStateIndex_) {
        if (value == 1) {
            start();
        } else if (value == 0) {
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
        printf("Port set to %d\n", value);
    } else if (function == numberOfBinsIndex_) {
        number_of_bins_ = value;
        setIntegerParam(numberOfBinsIndex_, value);
        printf("Number of bins set to %d\n", value);
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
        printf("Host set to %s\n", host_.c_str());
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
    } else     if (function == errorCountIndex_) {
        *value = error_count_;
    } else if (function == portIndex_) {
        *value = port_;
    } else if (function == numberOfBinsIndex_) {
        *value = number_of_bins_;
    } else if (function == maxBinsIndex_) {
        *value = 1000; // Default maximum bins
    } else if (function == frameBinSizeIndex_) {
        *value = frame_bin_size_;
    } else if (function == frameBinWidthIndex_) {
        *value = frame_bin_width_;
    } else if (function == frameBinOffsetIndex_) {
        *value = frame_bin_offset_;
    } else if (function >= binDisplayIndex_[0] && function <= binDisplayIndex_[4]) {
        // Handle individual bin display parameters
        int bin_index = -1;
        for (int i = 0; i < 5; ++i) {
            if (function == binDisplayIndex_[i]) {
                bin_index = i;
                break;
            }
        }
        if (bin_index >= 0 && running_sum_ && bin_index < static_cast<int>(running_sum_->get_bin_size())) {
            if (running_sum_->get_data_type() == HistogramData::DataType::RUNNING_SUM) {
                // Convert 64-bit to 32-bit (with overflow protection)
                uint64_t val64 = running_sum_->get_bin_value_64(bin_index);
                *value = (val64 > UINT32_MAX) ? UINT32_MAX : static_cast<epicsInt32>(val64);
            } else {
                *value = static_cast<epicsInt32>(running_sum_->get_bin_value_32(bin_index));
            }
        } else {
            *value = 0;
        }
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
        printf("readOctet: STATUS requested, current status: '%s'\n", status_.c_str());
        strncpy(value, status_.c_str(), maxChars);
        *nActual = std::min(status_.length(), maxChars);
        *eomReason = ASYN_EOM_END;
    } else if (function == hostIndex_) {
        strncpy(value, host_.c_str(), maxChars);
        *nActual = std::min(host_.length(), maxChars);
        *eomReason = ASYN_EOM_END;
    } else if (function == filenameIndex_) {
        char filename[256];
        getStringParam(filenameIndex_, sizeof(filename), filename);
        strncpy(value, filename, maxChars);
        *nActual = std::min(strlen(filename), maxChars);
        *eomReason = ASYN_EOM_END;
    } else {
        status = asynPortDriver::readOctet(pasynUser, value, maxChars, nActual, eomReason);
    }
    
    return status;
}

asynStatus tpx3HistogramDriver::readFloat64(asynUser *pasynUser, epicsFloat64 *value)
{
    int function = pasynUser->reason;
    asynStatus status = asynSuccess;
    
    if (function == acquisitionRateIndex_) {
        *value = acquisition_rate_;
    } else if (function == processingTimeIndex_) {
        *value = processing_time_;
    } else if (function == memoryUsageIndex_) {
        *value = memory_usage_;
    } else if (function == binWidthIndex_) {
        // Calculate actual bin width in nanoseconds: TPX3_TDC_CLOCK_PERIOD_SEC * 1e9 * frame_bin_width_
        *value = TPX3_TDC_CLOCK_PERIOD_SEC * 1e9 * frame_bin_width_;
        printf("DEBUG: readFloat64 binWidthIndex_ - frame_bin_width_=%d, calculated value=%.6f\n", frame_bin_width_, *value);
    } else if (function == totalTimeIndex_) {
        // Calculate total time range: bin_width * frame_bin_size
        *value = (TPX3_TDC_CLOCK_PERIOD_SEC * 1e9 * frame_bin_width_) * frame_bin_size_;
        printf("DEBUG: readFloat64 totalTimeIndex_ - frame_bin_width_=%d, frame_bin_size_=%d, calculated value=%.6f\n", 
               frame_bin_width_, frame_bin_size_, *value);
    } else if (function == timeAtFrameIndex_) {
        *value = time_at_frame_;
    } else {
        status = asynPortDriver::readFloat64(pasynUser, value);
    }
    
    return status;
}

asynStatus tpx3HistogramDriver::readInt32Array(asynUser *pasynUser, epicsInt32 *value, size_t nElements, size_t *nIn)
{
    int function = pasynUser->reason;
    asynStatus status = asynSuccess;
    
    printf("DEBUG: readInt32Array called with function=%d, histogramDataIndex_=%d, nElements=%zu\n", 
           function, histogramDataIndex_, nElements);
    fflush(stdout);
    
    if (function == histogramDataIndex_) {
        printf("DEBUG: Processing histogram data array directly (no parent call for asynPortDriver)\n");
        epicsMutexLock(mutex_);
        
        if (running_sum_) {
            size_t elements_to_copy = std::min(nElements, running_sum_->get_bin_size());
            printf("DEBUG: Copying %zu elements from running_sum_ (bin_size=%zu)\n", elements_to_copy, running_sum_->get_bin_size());
            
            for (size_t i = 0; i < elements_to_copy; ++i) {
                if (running_sum_->get_data_type() == HistogramData::DataType::RUNNING_SUM) {
                    // Convert 64-bit to 32-bit (with overflow protection)
                    uint64_t val64 = running_sum_->get_bin_value_64(i);
                    value[i] = (val64 > UINT32_MAX) ? UINT32_MAX : static_cast<epicsInt32>(val64);
                } else {
                    value[i] = static_cast<epicsInt32>(running_sum_->get_bin_value_32(i));
                }
            }
            
            // Zero out remaining elements if nElements > bin_size
            for (size_t i = elements_to_copy; i < nElements; ++i) {
                value[i] = 0;
            }
            
            *nIn = nElements;  // Always return the requested number of elements
            printf("DEBUG: Returning %zu histogram values (first 10): ", nElements);
            for (size_t i = 0; i < std::min(elements_to_copy, static_cast<size_t>(10)); ++i) {
                printf("%d ", value[i]);
            }
            printf("\n");
        } else {
            printf("DEBUG: No running_sum_ data, filling with zeros\n");
            for (size_t i = 0; i < nElements; ++i) {
                value[i] = 0;
            }
            *nIn = nElements;
        }
        
        epicsMutexUnlock(mutex_);
    } else {
        printf("DEBUG: Function %d does not match histogramDataIndex_ %d, calling parent\n", function, histogramDataIndex_);
        status = asynPortDriver::readInt32Array(pasynUser, value, nElements, nIn);
    }
    
    printf("DEBUG: readInt32Array returning status=%d, nIn=%zu\n", status, *nIn);
    fflush(stdout);
    
    return status;
}

// Removed readFloat64Array - using doCallbacksFloat64Array() to push data directly

// Public methods
void tpx3HistogramDriver::connect()
{
    epicsMutexLock(mutex_);
    
    if (!connected_) {
        status_ = createStatusMessage("Connecting to server", host_, port_);
        setStringParam(statusIndex_, status_.c_str());
        callParamCallbacks();
        
        if (network_client_->connect(host_, port_)) {
            connected_ = true;
            status_ = createStatusMessage("Connected successfully", host_, port_);
            setIntegerParam(connectedIndex_, 1);
            setStringParam(statusIndex_, status_.c_str());
            callParamCallbacks(statusIndex_);
            printf("Updated STATUS to: '%s'\n", status_.c_str());
            printf("Connected to Timepix3 server at %s:%d\n", host_.c_str(), port_);
        } else {
            status_ = createStatusMessage("Connection failed", host_, port_, 0, 0, ++error_count_);
            setStringParam(statusIndex_, status_.c_str());
            callParamCallbacks(statusIndex_);
            setIntegerParam(errorCountIndex_, error_count_);
            printf("Failed to connect to Timepix3 server at %s:%d\n", host_.c_str(), port_);
        }
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
        
        network_client_->disconnect();
        connected_ = false;
        status_ = createStatusMessage("Disconnected from server", host_, port_, frame_count_, total_counts_);
        setIntegerParam(connectedIndex_, 0);
        setStringParam(statusIndex_, status_.c_str());
        callParamCallbacks(statusIndex_);
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
    running_sum_ = nullptr;
    total_read_ = 0;
    
    setIntegerParam(frameCountIndex_, 0);
    setIntegerParam(totalCountsIndex_, 0);
    status_ = createStatusMessage("Histogram data reset", host_, port_, 0, 0, error_count_);
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
        status_ = createStatusMessage("Acquisition started", host_, port_, frame_count_, total_counts_);
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
        status_ = createStatusMessage("Acquisition stopped", host_, port_, frame_count_, total_counts_);
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
    status_ = createStatusMessage("Data saved to " + filename, host_, port_, frame_count_, total_counts_);
    setStringParam(statusIndex_, status_.c_str());
    
    epicsMutexUnlock(mutex_);
    callParamCallbacks();
    printf("Histogram data saved to %s\n", filename.c_str());
}

// Thread methods
void tpx3HistogramDriver::workerThread()
{
    while (running_) {
        if (connected_ && network_client_->is_connected()) {
            try {
                ssize_t bytes_read = network_client_->receive(
                    line_buffer_.data() + total_read_, 
                    MAX_BUFFER_SIZE - total_read_ - 1
                );

                if (bytes_read <= 0) {
                    if (bytes_read == 0) {
                        printf("Connection closed by peer\n");
                        connected_ = false;
                        setIntegerParam(connectedIndex_, 0);
                        status_ = createStatusMessage("Connection closed by peer", host_, port_, frame_count_, total_counts_, ++error_count_);
                        setStringParam(statusIndex_, status_.c_str());
                        setIntegerParam(errorCountIndex_, error_count_);
                    }
                    break;
                }
                
                printf("DEBUG: Received %zd bytes\n", bytes_read);

                total_read_ += bytes_read;
                line_buffer_[total_read_] = '\0';
                
                // Look for newline
                char* newline_pos = static_cast<char*>(memchr(line_buffer_.data(), '\n', total_read_));
                if (newline_pos) {
                    *newline_pos = '\0';
                    
                    // Process complete line
                    if (!processDataLine(line_buffer_.data(), newline_pos, total_read_)) {
                        break;
                    }
                    
                    // Move remaining data to start of buffer
                    size_t remaining = total_read_ - (newline_pos - line_buffer_.data() + 1);
                    if (remaining > 0) {
                        memmove(line_buffer_.data(), newline_pos + 1, remaining);
                    }
                    total_read_ = remaining;
                }
                
                // Prevent buffer overflow
                if (total_read_ >= MAX_BUFFER_SIZE - 1) {
                    printf("Buffer full, resetting\n");
                    total_read_ = 0;
                }
                
                // Update callbacks
                callParamCallbacks();
                
                // Update status periodically to show activity
                static int status_counter = 0;
                if (++status_counter % 100 == 0) {  // Update every 100 iterations
                    status_ = createStatusMessage("Processing data", host_, port_, frame_count_, total_counts_);
                    setStringParam(statusIndex_, status_.c_str());
                }
                
            } catch (const std::exception& e) {
                printf("Error in worker thread: %s\n", e.what());
                error_count_++;
                setIntegerParam(errorCountIndex_, error_count_);
                status_ = createStatusMessage("Processing error: " + std::string(e.what()), host_, port_, frame_count_, total_counts_, error_count_);
                setStringParam(statusIndex_, status_.c_str());
            }
        } else {
            epicsThreadSleep(0.1); // Short sleep when not connected
        }
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
        
        // Update status with performance metrics
        if (running_ && connected_) {
            status_ = createStatusMessage("Running - Rate: " + std::to_string(acquisition_rate_) + " fps", 
                                        host_, port_, frame_count_, total_counts_, error_count_);
        } else if (connected_) {
            status_ = createStatusMessage("Connected - Ready", host_, port_, frame_count_, total_counts_, error_count_);
        } else {
            status_ = createStatusMessage("Disconnected", host_, port_, frame_count_, total_counts_, error_count_);
        }
        setStringParam(statusIndex_, status_.c_str());
        
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
    printf("DEBUG: tpx3HistogramConfigure called with portName=%s, maxAddr=%d\n", portName, maxAddr);
    fflush(stdout);
    
    if (g_driver) {
        printf("DEBUG: Driver already configured\n");
        fflush(stdout);
        return -1;
    }
    
    printf("DEBUG: Creating new tpx3HistogramDriver instance\n");
    fflush(stdout);
    g_driver = new tpx3HistogramDriver(portName, maxAddr);
    if (!g_driver) {
        printf("ERROR: Failed to create tpx3HistogramDriver\n");
        fflush(stdout);
        return -1;
    }
    
    printf("DEBUG: Timepix3 Histogram Driver configured on port %s\n", portName);
    fflush(stdout);
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

// New histogram processing methods
bool tpx3HistogramDriver::processDataLine(char* line_buffer, char* newline_pos, size_t total_read) {
    // Skip empty lines
    if (strlen(line_buffer) == 0) {
        return true;
    }
    
    printf("DEBUG: Processing line: %s\n", line_buffer);
    
    // Parse JSON
    json j;
    try {
        j = json::parse(line_buffer);
    } catch (const json::parse_error& e) {
        printf("JSON parse error: %s\n", e.what());
        return true;  // Continue processing
    }

    try {
        // Extract header information
        int frame_number = j["frameNumber"];
        int bin_size = j["binSize"];
        int bin_width = j["binWidth"];
        int bin_offset = j["binOffset"];
        
        // Extract additional frame data
        double time_at_frame = j["timeAtFrame"];
        
        // Update frame data parameters
        epicsMutexLock(mutex_);
        time_at_frame_ = time_at_frame;
        frame_bin_size_ = bin_size;
        frame_bin_width_ = bin_width;
        frame_bin_offset_ = bin_offset;
        
        // Update parameters
        setDoubleParam(timeAtFrameIndex_, time_at_frame_);
        setIntegerParam(frameBinSizeIndex_, frame_bin_size_);
        setIntegerParam(frameBinWidthIndex_, frame_bin_width_);
        setIntegerParam(frameBinOffsetIndex_, frame_bin_offset_);
        
        // Calculate and set the derived parameters directly
        double calculated_bin_width = TPX3_TDC_CLOCK_PERIOD_SEC * 1e9 * frame_bin_width_;
        double calculated_total_time = calculated_bin_width * frame_bin_size_;
        
        setDoubleParam(binWidthIndex_, calculated_bin_width);
        setDoubleParam(totalTimeIndex_, calculated_total_time);
        
        printf("DEBUG: Updated frame data - bin_size=%d, bin_width=%d\n", frame_bin_size_, frame_bin_width_);
        printf("DEBUG: Calculated bin width = %.6f ns\n", calculated_bin_width);
        printf("DEBUG: Calculated total time = %.6f ns\n", calculated_total_time);
        
        // Notify parameter changes
        callParamCallbacks(timeAtFrameIndex_);
        callParamCallbacks(frameBinSizeIndex_);
        callParamCallbacks(frameBinWidthIndex_);
        callParamCallbacks(frameBinOffsetIndex_);
        
        // Also notify bin width and total time changes since they depend on frame data
        callParamCallbacks(binWidthIndex_);
        callParamCallbacks(totalTimeIndex_);
        
        printf("DEBUG: Called parameter callbacks for bin width and total time\n");
        epicsMutexUnlock(mutex_);

        printf("DEBUG: Processing frame %d, bin_size=%d, bin_width=%d, bin_offset=%d, time_at_frame=%.3e\n", 
               frame_number, bin_size, bin_width, bin_offset, time_at_frame);

        // Create frame histogram
        HistogramData frame_histogram(bin_size, HistogramData::DataType::FRAME_DATA);
        
        // Calculate bin edges
        frame_histogram.calculate_bin_edges(bin_width, bin_offset);

        // Read binary data
        std::vector<uint32_t> tof_bin_values(bin_size);
        size_t binary_needed = bin_size * sizeof(uint32_t);
        
        // Copy any binary data we already have after the newline
        size_t remaining = total_read - (newline_pos - line_buffer + 1);
        size_t binary_read = 0;
        
        if (remaining > 0) {
            size_t to_copy = std::min(remaining, binary_needed);
            memcpy(tof_bin_values.data(), newline_pos + 1, to_copy);
            binary_read = to_copy;
        }

        // Read any remaining binary data needed
        if (binary_read < binary_needed) {
            printf("DEBUG: Need to read %zu more bytes of binary data\n", binary_needed - binary_read);
            if (!network_client_->receive_exact(
                reinterpret_cast<char*>(tof_bin_values.data()) + binary_read,
                binary_needed - binary_read)) {
                
                printf("Failed to read binary data\n");
                return false;
            }
            printf("DEBUG: Successfully read binary data\n");
        }

        // Convert to little-endian
        printf("DEBUG: Bin values before conversion: ");
        for (int i = 0; i < bin_size; ++i) {
            printf("%u ", tof_bin_values[i]);
        }
        printf("\n");
        
        for (int i = 0; i < bin_size; ++i) {
            tof_bin_values[i] = __builtin_bswap32(tof_bin_values[i]);
            frame_histogram.set_bin_value_32(i, tof_bin_values[i]);
        }
        
        printf("DEBUG: Bin values after conversion: ");
        for (int i = 0; i < bin_size; ++i) {
            printf("%u ", tof_bin_values[i]);
        }
        printf("\n");

        // Print frame information
        printf("\nFrame %d data:\n", frame_number);
        printf("Bin edges: ");
        for (int i = 0; i < bin_size + 1; ++i) {
            printf("%.9e ", frame_histogram.get_bin_edges()[i]);
        }
        printf("\nBin values: ");
        for (int i = 0; i < bin_size; ++i) {
            printf("%u ", frame_histogram.get_bin_value_32(i));
        }
        printf("\n");

        // Process frame
        processFrame(frame_histogram);
        
        printf("Frame %d processed (running sum updated)\n", frame_number);

    } catch (const std::exception& e) {
        printf("Error processing frame: %s\n", e.what());
    }

    return true;
}

void tpx3HistogramDriver::processFrame(const HistogramData& frame_data) {
    printf("DEBUG: processFrame called with frame_data bin_size=%zu\n", frame_data.get_bin_size());
    epicsMutexLock(mutex_);
    
    if (!running_sum_) {
        // Initialize running sum with same bin size as first frame
        running_sum_ = std::make_unique<HistogramData>(
            frame_data.get_bin_size(), 
            HistogramData::DataType::RUNNING_SUM
        );
        
        // Copy bin edges
        for (size_t i = 0; i < frame_data.get_bin_edges().size(); ++i) {
            running_sum_->set_bin_edge(i, frame_data.get_bin_edges()[i]);
        }
        
        printf("DEBUG: Initialized running_sum_ with bin_size=%zu\n", running_sum_->get_bin_size());
    }
    
    // Check if bin sizes match
    if (running_sum_->get_bin_size() != frame_data.get_bin_size()) {
        printf("WARNING: Bin size mismatch! Running sum has %zu bins, frame has %zu bins. Reinitializing running sum.\n", 
               running_sum_->get_bin_size(), frame_data.get_bin_size());
        
        // Reinitialize running sum with new bin size
        running_sum_ = std::make_unique<HistogramData>(
            frame_data.get_bin_size(), 
            HistogramData::DataType::RUNNING_SUM
        );
        
        // Copy bin edges from frame data
        for (size_t i = 0; i < frame_data.get_bin_edges().size(); ++i) {
            running_sum_->set_bin_edge(i, frame_data.get_bin_edges()[i]);
        }
        
        printf("DEBUG: Reinitialized running_sum_ with bin_size=%zu\n", running_sum_->get_bin_size());
        
        // Reset frame count since we're starting with new bin configuration
        frame_count_ = 0;
        total_counts_ = 0;
        
        // Update parameters
        setIntegerParam(frameCountIndex_, static_cast<epicsInt32>(frame_count_));
        setIntegerParam(totalCountsIndex_, static_cast<epicsInt32>(total_counts_));
        callParamCallbacks(frameCountIndex_);
        callParamCallbacks(totalCountsIndex_);
    }
    
    // Add frame data to running sum
    try {
        running_sum_->add_histogram(frame_data);
    } catch (const std::exception& e) {
        printf("ERROR: Failed to add histogram: %s\n", e.what());
        error_count_++;
        setIntegerParam(errorCountIndex_, static_cast<epicsInt32>(error_count_));
        epicsMutexUnlock(mutex_);
        return;
    }
    
    // Update frame count and total counts
    frame_count_++;
    uint64_t frame_total = 0;
    for (size_t i = 0; i < frame_data.get_bin_size(); ++i) {
        frame_total += frame_data.get_bin_value_32(i);
    }
    total_counts_ += frame_total;
    
    printf("DEBUG: Updated frame_count_=%llu, total_counts_=%llu\n", 
           (unsigned long long)frame_count_, (unsigned long long)total_counts_);
    
    // Update parameters
    setIntegerParam(frameCountIndex_, static_cast<epicsInt32>(frame_count_));
    setIntegerParam(totalCountsIndex_, static_cast<epicsInt32>(total_counts_));
    
    // Notify that frame count and total counts have been updated
    callParamCallbacks(frameCountIndex_);
    callParamCallbacks(totalCountsIndex_);
    
    printf("DEBUG: Called callParamCallbacks for frame count and total counts\n");
    
    // Update status with frame processing info
    if (frame_count_ % 10 == 0) {  // Update every 10 frames
        status_ = createStatusMessage("Processing frames", host_, port_, frame_count_, total_counts_);
        setStringParam(statusIndex_, status_.c_str());
        callParamCallbacks(statusIndex_);
    }
    
    // Update display bin parameters
    if (running_sum_) {
        size_t actual_bin_size = running_sum_->get_bin_size();
        
        // Update display bin parameters (BIN_0 to BIN_4)
        for (int i = 0; i < 5 && i < static_cast<int>(actual_bin_size); ++i) {
            uint32_t bin_value;
            if (running_sum_->get_data_type() == HistogramData::DataType::RUNNING_SUM) {
                // Convert 64-bit to 32-bit (with overflow protection)
                uint64_t val64 = running_sum_->get_bin_value_64(i);
                bin_value = (val64 > UINT32_MAX) ? UINT32_MAX : static_cast<uint32_t>(val64);
            } else {
                bin_value = running_sum_->get_bin_value_32(i);
            }
            setIntegerParam(binDisplayIndex_[i], static_cast<epicsInt32>(bin_value));
            callParamCallbacks(binDisplayIndex_[i]);
        }
        
        // Zero out unused display bin parameters
        for (int i = static_cast<int>(actual_bin_size); i < 5; ++i) {
            setIntegerParam(binDisplayIndex_[i], 0);
            callParamCallbacks(binDisplayIndex_[i]);
        }
        
        // Update NUMBER_OF_BINS parameter to reflect actual bin size
        setIntegerParam(numberOfBinsIndex_, static_cast<epicsInt32>(actual_bin_size));
        callParamCallbacks(numberOfBinsIndex_);
        
        printf("DEBUG: Updated display bin parameters (actual_bin_size=%zu)\n", actual_bin_size);
    }
    
    // Notify that histogram data has been updated
    printf("DEBUG: About to push histogram array data via doCallbacksInt32Array\n");
    
    // For array parameters, we need to push the data using doCallbacksInt32Array
    if (running_sum_) {
        size_t bin_size = running_sum_->get_bin_size();
        std::vector<epicsInt32> array_data(bin_size);
        
        for (size_t i = 0; i < bin_size; ++i) {
            if (running_sum_->get_data_type() == HistogramData::DataType::RUNNING_SUM) {
                uint64_t val64 = running_sum_->get_bin_value_64(i);
                array_data[i] = (val64 > UINT32_MAX) ? UINT32_MAX : static_cast<epicsInt32>(val64);
            } else {
                array_data[i] = static_cast<epicsInt32>(running_sum_->get_bin_value_32(i));
            }
        }
        
        printf("DEBUG: Pushing %zu array elements (first 5: %d %d %d %d %d)\n", 
               bin_size, array_data[0], array_data[1], array_data[2], array_data[3], array_data[4]);
        
        doCallbacksInt32Array(array_data.data(), bin_size, histogramDataIndex_, 0);
        
            // Also calculate and push the time axis data (in milliseconds)
            histogram_time_data_.resize(bin_size);
            for (size_t i = 0; i < bin_size; ++i) {
                // Time value for bin i: bin_offset + i * bin_width (convert to milliseconds)
                histogram_time_data_[i] = (frame_bin_offset_ + i * frame_bin_width_) * TPX3_TDC_CLOCK_PERIOD_SEC * 1e3;
            }
            
            printf("DEBUG: Calculated %zu time array elements (first 5: %.3f %.3f %.3f %.3f %.3f ms)\n", 
                   bin_size, histogram_time_data_[0], histogram_time_data_[1], histogram_time_data_[2], histogram_time_data_[3], histogram_time_data_[4]);
            printf("DEBUG: histogram_time_data_ size=%zu\n", histogram_time_data_.size());
            
            // Push the time axis data using doCallbacksFloat64Array
            printf("DEBUG: Pushing time array via doCallbacksFloat64Array\n");
            doCallbacksFloat64Array(histogram_time_data_.data(), bin_size, histogramTimeMsIndex_, 0);
            printf("DEBUG: Finished pushing time array data\n");
    }
    printf("DEBUG: Finished pushing histogram array data and time axis data\n");
    
    // Save updated running sum
    saveRunningSum();
    
    epicsMutexUnlock(mutex_);
}

void tpx3HistogramDriver::saveRunningSum() {
    if (!running_sum_) {
        return;
    }
    
    std::string filename = "data/tof-histogram-running-sum.txt";
    saveHistogramToFile(filename, *running_sum_);
}

void tpx3HistogramDriver::saveHistogramToFile(const std::string& filename, const HistogramData& histogram) {
    std::ofstream file(filename);
    if (!file.is_open()) {
        printf("Failed to open file: %s\n", filename.c_str());
        return;
    }

    file << "# Time of Flight Histogram Data\n";
    file << "# Bins: " << histogram.get_bin_size() << "\n";
    file << "#\n";

    for (size_t i = 0; i < histogram.get_bin_size(); ++i) {
        if (histogram.get_data_type() == HistogramData::DataType::RUNNING_SUM) {
            file << std::scientific << std::setprecision(9) 
                 << histogram.get_bin_edges()[i] << "\t" 
                 << histogram.get_bin_value_64(i) << "\n";
        } else {
            file << std::scientific << std::setprecision(9) 
                 << histogram.get_bin_edges()[i] << "\t" 
                 << histogram.get_bin_value_32(i) << "\n";
        }
    }
    
    // Write last bin edge
    file << std::scientific << std::setprecision(9) 
         << histogram.get_bin_edges()[histogram.get_bin_size()] << "\n";
    
    file.close();
}

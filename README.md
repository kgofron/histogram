# Timepix3 Histogram EPICS IOC

This is an EPICS IOC (Input/Output Controller) for the Timepix3 Time-of-Flight histogram processor. It integrates with the existing `/home/kg1/Documents/src/github/tpx3Histogram` C++ code to provide EPICS control and monitoring capabilities.

## Features

- **EPICS Integration**: Full EPICS record support for controlling and monitoring the Timepix3 histogram
- **Real-time Data**: Live histogram data updates through EPICS records
- **Network Communication**: TCP socket communication with Timepix3 data servers
- **Histogram Processing**: Real-time frame processing and running sum accumulation
- **Data Export**: Save histogram data to files
- **Status Monitoring**: Connection status, frame counts, and error reporting

## Architecture

The IOC consists of several components:

- **EPICS Records**: Database records for control and monitoring
- **Device Support**: C++ device driver integrating with existing histogram code
- **Network Client**: TCP socket communication layer
- **Histogram Processor**: Data processing and accumulation engine

## Prerequisites

### EPICS Base
- EPICS Base 7.0 or later
- EPICS_BASE environment variable set

### System Dependencies
```bash
# Ubuntu/Debian
sudo apt-get install build-essential nlohmann-json3-dev

# CentOS/RHEL/Fedora
sudo yum install gcc-c++ make nlohmann-json-devel
```

## Building

```bash
# Build the IOC
make

# Clean build artifacts
make clean
```

## Running the IOC

```bash
# Start the IOC
cd iocBoot/ioctpx3Histogram
../../bin/linux-x86_64/tpx3Histogram st.cmd
```

## EPICS Records

### Connection Control
- `TPX3:CONNECT` - Connect to Timepix3 server
- `TPX3:DISCONNECT` - Disconnect from server
- `TPX3:CONNECTED` - Connection status

### Configuration
- `TPX3:HOST` - Server hostname/IP (default: 127.0.0.1)
- `TPX3:PORT` - Server port (default: 8451)

### Control
- `TPX3:RESET` - Reset histogram data
- `TPX3:START` - Start acquisition
- `TPX3:STOP` - Stop acquisition

### Status and Data
- `TPX3:FRAME_COUNT` - Total frames processed
- `TPX3:TOTAL_COUNTS` - Total counts accumulated
- `TPX3:ACQUISITION_RATE` - Frames per second
- `TPX3:BIN_0` through `TPX3:BIN_4` - Individual bin values
- `TPX3:HISTOGRAM_DATA` - Full histogram data (waveform record)

### Time Information
- `TPX3:BIN_WIDTH_PS` - Bin width in picoseconds
- `TPX3:TOTAL_TIME_NS` - Total time range in nanoseconds

### File Output
- `TPX3:SAVE_DATA` - Save histogram data to file
- `TPX3:FILENAME` - Output filename

### Monitoring
- `TPX3:STATUS` - Device status message
- `TPX3:ERROR_COUNT` - Error count
- `TPX3:PROCESSING_TIME_MS` - Average frame processing time
- `TPX3:MEMORY_USAGE_MB` - Memory usage

## Usage Examples

### Connect to Server
```bash
caput TPX3:HOST "192.168.1.100"
caput TPX3:PORT "9000"
caput TPX3:CONNECT 1
```

### Monitor Data
```bash
# Get frame count
caget TPX3:FRAME_COUNT

# Get total counts
caget TPX3:TOTAL_COUNTS

# Get connection status
caget TPX3:CONNECTED

# Get individual bin values
caget TPX3:BIN_0
caget TPX3:BIN_1
```

### Control Operations
```bash
# Reset histogram data
caput TPX3:RESET 1

# Start acquisition
caput TPX3:START 1

# Stop acquisition
caput TPX3:STOP 1
```

### Save Data
```bash
# Set filename
caput TPX3:FILENAME "my_histogram.txt"

# Save data
caput TPX3:SAVE_DATA 1
```

## Integration with Existing Code

This IOC integrates with the existing Timepix3 histogram C++ code by:

1. **Reusing Core Classes**: Leverages the existing `HistogramData`, `NetworkClient`, and `HistogramProcessor` classes
2. **EPICS Wrapper**: Provides EPICS device support wrapper around the existing functionality
3. **Data Flow**: Maintains the same data processing pipeline while adding EPICS record updates
4. **Configuration**: Uses the same network and processing parameters

## Data Format

The IOC expects the same data format as the original code:

```json
{
  "frameNumber": 59,
  "binSize": 10,
  "binWidth": 384000,
  "binOffset": 0,
  "dataSize": 40,
  "countSum": 7380
}
```

Followed by binary histogram data (32-bit integers in network byte order).

## Troubleshooting

### Build Issues
- Ensure EPICS_BASE is set correctly
- Check C++17 compiler support
- Verify nlohmann/json is installed

### Runtime Issues
- Check network connectivity to Timepix3 server
- Verify server is running on specified host/port
- Monitor EPICS record values for status information

### Data Issues
- Check connection status with `caget TPX3:CONNECTED`
- Monitor frame count and total counts
- Verify bin values are updating

## Development

### Adding New Records
1. Add record definition to `tpx3Histogram.db`
2. Update device support in `tpx3HistogramRegister.cpp`
3. Rebuild and test

### Modifying Device Support
1. Edit `tpx3HistogramRegister.cpp`
2. Update device support functions
3. Rebuild and test

## License

This project is licensed under the same terms as the original Timepix3 histogram code.

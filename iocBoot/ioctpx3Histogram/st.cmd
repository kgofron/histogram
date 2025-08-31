#!../../bin/linux-x86_64/tpx3Histogram

# Timepix3 Histogram IOC Startup Script
# This script initializes and starts the Timepix3 histogram IOC

< envPaths

# Set the IOC name
epicsEnvSet("IOC", "ioctpx3Histogram")
epicsEnvSet("Sys", "TPX3-TEST:")
epicsEnvSet("Dev", "Histogram:")

# Load the database definition
dbLoadDatabase("../../dbd/tpx3Histogram.dbd")

# Register all support components
tpx3Histogram_registerRecordDeviceDriver(pdbbase)

# Load the database records
dbLoadRecords("../../db/tpx3Histogram.db", "P=$(Sys),R=$(Dev),PORT=TPX3_PORT,ADDR=0,TIMEOUT=1.0")

# Initialize records
iocInit()

# Configure the driver after iocInit
tpx3HistogramConfigure("TPX3_PORT", 0)

# Set record values
dbpf("$(Sys)$(Dev)CONNECT", "0")
dbpf("$(Sys)$(Dev)DISCONNECT", "0")
dbpf("$(Sys)$(Dev)RESET", "0")
dbpf("$(Sys)$(Dev)START", "0")
dbpf("$(Sys)$(Dev)STOP", "0")
dbpf("$(Sys)$(Dev)HOST", "127.0.0.1")
dbpf("$(Sys)$(Dev)PORT", "8451")
dbpf("$(Sys)$(Dev)STATUS", "Initialized")
dbpf("$(Sys)$(Dev)BIN_WIDTH_PS", "0.260")
dbpf("$(Sys)$(Dev)TOTAL_TIME_NS", "260.0")

# Start the IOC
epicsThreadSleep(0.2)

#!../../bin/linux-x86_64/tpx3Histogram

# Timepix3 Histogram IOC Startup Script
# Author: K. Gofron
# Last Modified: October 20, 2025
# This script initializes and starts the Timepix3 histogram IOC

< envPaths

# Set the IOC name
epicsEnvSet("IOC", "ioctpx3Histogram")
epicsEnvSet("Sys", "TPX3-TEST:")
epicsEnvSet("Dev", "Histogram:")

# Set configurable parameters (can be overridden via command line)
# This is the maximum number of bins for the histogram
# The value 1,000,000 allows 16.6666 ns minimum bin width, at SNS 60 Hz neutron source.
epicsEnvSet("MAX_BINS", "1000000")

# Load the database definition
dbLoadDatabase("../../dbd/tpx3Histogram.dbd")

# Register all support components
tpx3Histogram_registerRecordDeviceDriver(pdbbase)

# Configure the driver BEFORE loading database
tpx3HistogramConfigure("TPX3_PORT", 0)

# Load the database records with configurable array parameters
dbLoadRecords("../../db/tpx3Histogram.db", "P=$(Sys),R=$(Dev),PORT=TPX3_PORT,ADDR=0,TIMEOUT=1.0,TYPE=Int32,FTVL=LONG,NELEMENTS=$(MAX_BINS)")

# Initialize records
iocInit()

# Set initial record values (driver will update these automatically)
dbpf("$(Sys)$(Dev)HOST", "127.0.0.1")
dbpf("$(Sys)$(Dev)PORT", "8451")
dbpf("$(Sys)$(Dev)MAX_BINS", "$(MAX_BINS)")

# Start the IOC
epicsThreadSleep(0.2)

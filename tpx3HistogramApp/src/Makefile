TOP=../..

include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE
#=============================

#=============================
# Build the IOC application

# Add the name of the IOC application
PROD_IOC = tpx3Histogram

# tpx3Histogram.dbd will be created and installed
DBD += tpx3Histogram.dbd

# tpx3Histogram.dbd will be made up from these files:
tpx3Histogram_DBD += base.dbd
tpx3Histogram_DBD += asyn.dbd
tpx3Histogram_DBD += tpx3HistogramSupport.dbd

# Add all the support libraries needed by this IOC
tpx3Histogram_LIBS += asyn

# Add C++11 support
tpx3Histogram_CXXFLAGS += -std=c++17

# tpx3Histogram_registerRecordDeviceDriver.cpp derives from tpx3Histogram.dbd
tpx3Histogram_SRCS += tpx3Histogram_registerRecordDeviceDriver.cpp
tpx3Histogram_SRCS += tpx3HistogramDriver.cpp
tpx3Histogram_SRCS += tpx3HistogramMain.cpp

# Add support libraries
tpx3Histogram_LIBS += asyn

# Finally link to the EPICS Base libraries
tpx3Histogram_LIBS += $(EPICS_BASE_IOC_LIBS)

#===========================

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE


# SPDX-FileCopyrightText: 1998 Argonne National Laboratory
#
# SPDX-License-Identifier: EPICS

TOP=../..

include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS BELOW HERE

# Use typed rset structure (see 3.16.1 release notes)
USR_CPPFLAGS += -DUSE_TYPED_RSET

USR_LDFLAGS += -L/opt/lib

# Auto-generate a header file containing a version string.
# Version comes from the VCS if available, else date+time.
GENVERSION = MSU_SPI_ioVersion.h
# Macro name
GENVERSIONMACRO = MSU_SPI_ioVERSION


# MSU_DigitalSupport.dbd will be created and installed
DBD += MSU_SPIDevice.dbd
# MSU_SPIDevice.dbd will include these files:
MSU_SPIDevice_DBD += base.dbd
MSU_SPIDevice_DBD += drvSPI.dbd

# Add all the support libraries needed by this library
#MSU_SPIDevice_LIBS += gpio
#MSU_SPIDevice_LIBS += DRV8912
#MSU_SPIDevice_Libs += AMC7823

# Build an IOC support library
LIBRARY += MSU_SPIDevice
MSU_SPIDevice_SRCS += drvSPI.c
MSU_SPIDevice_SRCS += printReportBuffer.c



# MSU_SPIDeviceSupport_registerRecordDeviceDriver.cpp derives from MSU_SPIDeviceSupport.dbd
MSU_SPIDevice_SRCS += MSU_SPIDevice_registerRecordDeviceDriver.cpp

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD EXTRA GNUMAKE RULES BELOW HERE

# Explicit dependency needed for generated header file
devMSU_SPIDeviceVersion$(DEP): $(COMMON_DIR)/$(GENVERSION)



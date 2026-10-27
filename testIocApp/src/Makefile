# SPDX-FileCopyrightText: 2003 Argonne National Laboratory
#
# SPDX-License-Identifier: EPICS

TOP=../..

include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE
#=============================

#=============================
# Build the IOC application

PROD_IOC = testIoc
# testIoc.dbd will be created and installed
DBD += testIoc.dbd

# testIoc.dbd will be made up from these files:
testIoc_DBD += base.dbd
testIoc_DBD += system.dbd
testIoc_DBD += MSU_SPIDevice.dbd

# Include dbd files from all support applications:
#testIoc_DBD += xxx.dbd

# Add all the support libraries needed by this IOC
#testIoc_LIBS += xxx
testIoc_LIBS += MSU_SPIDevice
testIoc_LIBS += GPIO
testIoc_LIBS += SPIDriver

SYS_PROD_LIBS += gpiod

# testIoc_registerRecordDeviceDriver.cpp derives from testIoc.dbd
testIoc_SRCS += testIoc_registerRecordDeviceDriver.cpp

# Build the main IOC entry point on workstation OSs.
testIoc_SRCS_DEFAULT += testIocMain.cpp
testIoc_SRCS_vxWorks += -nil-

# Add support from base/src/vxWorks if needed
#testIoc_OBJS_vxWorks += $(EPICS_BASE_BIN)/vxComLibrary

# Finally link to the EPICS Base libraries
testIoc_LIBS += $(EPICS_BASE_IOC_LIBS)

#===========================

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE


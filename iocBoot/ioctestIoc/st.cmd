#!../../bin/linux-aarch64/testIoc

#- SPDX-FileCopyrightText: 2000 Argonne National Laboratory
#-
#- SPDX-License-Identifier: EPICS

#- You may have to change testIoc to something else
#- everywhere it appears in this file

< envPaths

cd "${TOP}"

## Register all support components
dbLoadDatabase "dbd/testIoc.dbd"
testIoc_registerRecordDeviceDriver pdbbase

#define my SPI port
#               Device          Mode    Bits    Speed
addSPIDevice("mySPI", "/dev/spidev0.0",  1,      8,      300000)


## Load record instances
dbLoadRecords "${TOP}/db/AMC7823Support.vdb", "P=dudleyd_, S=test:, user=dudleyd"
dbLoadRecords "${TOP}/db/AMC7823Analogs.vdb", "P=dudleyd_, S=test:, user=dudleyd"
dbLoadRecords "${TOP}/db/gatedAmplifierTester.vdb", "P=dudleyd_, S=test:, user=dudleyd"

#- Set this to see messages from mySub
#-var mySubDebug 1

#- Run this to trace the stages of iocInit
#-traceIocInit

cd "${TOP}/iocBoot/${IOC}"
iocInit

## Start any sequence programs
#seq sncExample, "user=dudleyd"

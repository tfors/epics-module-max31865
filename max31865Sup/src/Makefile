TOP=../..

include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE
#=============================

#==================================================
# build a support library

LIBRARY_IOC += max31865

# xxxRecord.h will be created from xxxRecord.dbd
# DBDINC += devMax31865.dbd
# install max31865.dbd into <top>/dbd
DBD += menuScan.dbd
DBD += max31865.dbd

max31865_DBD += devMax31865.dbd

# specify all source files to be compiled and added to the library
max31865_SRCS += devMax31865.c

max31865_LIBS += $(EPICS_BASE_IOC_LIBS)

#===========================

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE


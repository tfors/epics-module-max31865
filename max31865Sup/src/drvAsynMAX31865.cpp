/* drvAsynMAX31865.cpp
 *
 * EPICS asynPortDriver for MAX31865 PT100 RTD Temperature
 * Sensor Amplifier under Linux using spidev.

 * Thomas Fors <tom@fors.net>
 * December, 2017
 */

#include <math.h>

#include <epicsExport.h>
#include <epicsThread.h>
#include <epicsTime.h>
#include <iocsh.h>

#include "drvAsynMAX31865.h"

/* Conversion parameters for PT100 RTD */
#define R_NOM (100)
#define R_REF (430)

#define RTD_A (3.9083e-3)
#define RTD_B (-5.775e-7)

#define Z1 (-RTD_A)
#define Z2 (RTD_A * RTD_A - (4 * RTD_B)
#define Z3 (4 * RTD_B) / R_NOM)
#define Z4 (2 * RTD_B)

static const char* driverName = "drvAsynMAX31865";

static void pollTask(void *drvPvt);

drvAsynMAX31865::drvAsynMAX31865(const char* portName, unsigned char spiBus,
                                 unsigned char spiChipSel,
                                 unsigned char spiMode, unsigned char spiBits,
                                 unsigned int spiSpeed)
    : drvAsynSPI(portName, spiBus, spiChipSel, spiMode, spiBits, spiSpeed,
                 1, /* maxAddr */
                    /* Interface mask */
                 (asynFloat64Mask | asynFloat64ArrayMask | asynDrvUserMask),
                    /* Interrupt mask */
                 (asynFloat64Mask | asynFloat64ArrayMask),
                 0, /* asynFlags (does not block and is not multi-device) */
                 1, /* Autoconnect */
                 0, /* Default priority */
                 0) /* Default stack size */
{
    const char* functionName = "drvAsynMAX31865";

    createParam(P_TempCString, asynParamFloat64, &P_Temp_C);
    createParam(P_TempFString, asynParamFloat64, &P_Temp_F);

    eventId_ = epicsEventCreate(epicsEventEmpty);
    if (epicsThreadCreate("drvAsynMAX31865Task", epicsThreadPriorityMedium,
                          epicsThreadGetStackSize(epicsThreadStackMedium),
                          (EPICSTHREADFUNC)::pollTask, this) == NULL) {
        printf("%s:%s: epicsThreadCreate failure\n", driverName, functionName);
        return;
    }

    // Call our connect method to work-around the fact that autoconnect flag
    // triggers the base class to call connect before our constructor runs.
    this->connect(this->pasynUserSelf);
}

asynStatus drvAsynMAX31865::connect(asynUser* pasynUser)
{
    asynStatus status = asynSuccess;
    unsigned char rx[2];

    /* Perform SPI connect */
    status = this->spi_connect(pasynUser);
    if (status != asynSuccess) {
        return status;
    }

    /* Config MAX31865 to auto sample */
    if (spi_wr_rd((unsigned char *)"\x80\xd0", rx, 2) != 0) {
        return asynError;
    }

    return status;
}

asynStatus drvAsynMAX31865::disconnect(asynUser* pasynUser)
{
    /* Perform SPI disconnect */
    return this->spi_disconnect(pasynUser);
}

static void pollTask(void* drvPvt)
{
    drvAsynMAX31865* pPvt = (drvAsynMAX31865*)drvPvt;
    pPvt->pollTask();
}

void drvAsynMAX31865::pollTask(void)
{
    epicsTimeStamp now;
    epicsUInt32 delay_ns;
    unsigned char rx[3];
    epicsFloat64 newVal;
    epicsFloat64 smoo = 0.983;

    lock();
    while (1) {
        unlock();

        epicsTimeGetCurrent(&now);
        delay_ns = 50000000 - (now.nsec % 50000000); /* 20 Hz */
        epicsEventWaitWithTimeout(eventId_, delay_ns / 1.e9);

        lock();

        if (spi_wr_rd((unsigned char*)"\x01\x00\x00", rx, 3) == 0) {

            resistance = ((rx[1] << 7) + (rx[2] >> 1)) / 32768.0 * R_REF;
            newVal     = (sqrt(Z2 + (Z3 * resistance)) + Z1) / Z4;
            celsius    = (1 - smoo) * newVal + smoo * celsius;
            setDoubleParam(P_Temp_C, celsius);
            setDoubleParam(P_Temp_F, celsius * 9 / 5 + 32);

            updateTimeStamp();
            callParamCallbacks();
        }
    }
}

extern "C" {

int drvAsynMAX31865Configure(const char* portName, int spiBus, int spiChipSel,
                             int spiMode, int spiBits, int spiSpeed)
{
    new drvAsynMAX31865(portName, (unsigned char)spiBus,
                        (unsigned char)spiChipSel, (unsigned char)spiMode,
                        (unsigned char)spiBits, (unsigned int)spiSpeed);
    return (asynSuccess);
}

/* EPICS iocsh shell commands */
static const iocshArg initArg0 = { "portName", iocshArgString };
static const iocshArg initArg1 = { "spi bus", iocshArgInt };
static const iocshArg initArg2 = { "spi chipsel", iocshArgInt };
static const iocshArg initArg3 = { "spi mode", iocshArgInt };
static const iocshArg initArg4 = { "spi bits", iocshArgInt };
static const iocshArg initArg5 = { "spi speed", iocshArgInt };
static const iocshArg* const initArgs[]
    = { &initArg0, &initArg1, &initArg2, &initArg3, &initArg4, &initArg5 };
static const iocshFuncDef initFuncDef
    = { "drvAsynMAX31865Configure", 6, initArgs };

static void initCallFunc(const iocshArgBuf* args)
{
    drvAsynMAX31865Configure(args[0].sval, args[1].ival, args[2].ival,
                             args[3].ival, args[4].ival, args[5].ival);
}

void drvAsynMAX31865Register(void)
{
    iocshRegister(&initFuncDef, initCallFunc);
}

epicsExportRegistrar(drvAsynMAX31865Register);
}

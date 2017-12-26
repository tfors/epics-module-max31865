/* drvAsynSPI.cpp
 *
 * EPICS asynPortDriver support for SPI devices under Linux
 * using spidev.
 *
 * Thomas Fors <tom@fors.net>
 * December, 2017
 */

#include <fcntl.h>
#include <linux/spi/spidev.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include "drvAsynSPI.h"

static const char* driverName = "drvAsynSPI";

drvAsynSPI::drvAsynSPI(const char* portName, unsigned char spiBus,
                       unsigned char spiChipSel, unsigned char spiMode,
                       unsigned char spiBits, unsigned int spiSpeed,
                       int maxAddrIn, int interfaceMask, int interruptMask,
                       int asynFlags, int autoConnect, int priority,
                       int stackSize)
    : asynPortDriver(portName, maxAddrIn, interfaceMask, interruptMask,
                     asynFlags, autoConnect, priority, stackSize)
{
    this->spiBus     = spiBus;
    this->spiChipSel = spiChipSel;
    this->spiMode    = spiMode;
    this->spiBits    = spiBits;
    this->spiSpeed   = spiSpeed;
    this->fd         = -1;
}

asynStatus drvAsynSPI::spi_connect(asynUser* pasynUser)
{
    const char* functionName = "connect";
    unsigned char uchar;
    unsigned int uint;

    /* Open SPIDEV Device */
    char device[16];
    sprintf(device, "/dev/spidev%d.%d", spiBus, spiChipSel);
    fd = open(device, O_RDWR);
    if (fd < 0) {
        asynPrint(pasynUser, ASYN_TRACE_ERROR,
                  "%s:%s: unable to open %s for r/w\n", driverName,
                  functionName, device);
        return asynDisconnected;
    }

    /* Set SPI Mode */
    if (ioctl(fd, SPI_IOC_WR_MODE, &spiMode) == -1) {
        asynPrint(pasynUser, ASYN_TRACE_ERROR,
                  "%s:%s: unable to write SPI mode on %s\n", driverName,
                  functionName, device);
        return asynDisconnected;
    }
    uchar = 0;
    if (ioctl(fd, SPI_IOC_RD_MODE, &uchar) == -1) {
        asynPrint(pasynUser, ASYN_TRACE_ERROR,
                  "%s:%s: unable to read SPI mode on %s\n", driverName,
                  functionName, device);
        return asynDisconnected;
    }
    if (uchar != spiMode) {
        asynPrint(pasynUser, ASYN_TRACE_ERROR,
                  "%s:%s: unable to set SPI mode on %s\n", driverName,
                  functionName, device);
        return asynDisconnected;
    }

    /* Set SPI num bits per word */
    if (ioctl(fd, SPI_IOC_WR_BITS_PER_WORD, &spiBits) == -1) {
        asynPrint(pasynUser, ASYN_TRACE_ERROR,
                  "%s:%s: unable to write SPI bits on %s\n", driverName,
                  functionName, device);
        return asynDisconnected;
    }
    uchar = 0;
    if (ioctl(fd, SPI_IOC_RD_BITS_PER_WORD, &uchar) == -1) {
        asynPrint(pasynUser, ASYN_TRACE_ERROR,
                  "%s:%s: unable to read SPI bits on %s\n", driverName,
                  functionName, device);
        return asynDisconnected;
    }
    if (uchar != spiBits) {
        asynPrint(pasynUser, ASYN_TRACE_ERROR,
                  "%s:%s: unable to set SPI bits on %s\n", driverName,
                  functionName, device);
        return asynDisconnected;
    }

    /* Set SPI speed */
    if (ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &spiSpeed) == -1) {
        asynPrint(pasynUser, ASYN_TRACE_ERROR,
                  "%s:%s: unable to write SPI speed on %s\n", driverName,
                  functionName, device);
        return asynDisconnected;
    }
    uint = 0;
    if (ioctl(fd, SPI_IOC_RD_MAX_SPEED_HZ, &uint) == -1) {
        asynPrint(pasynUser, ASYN_TRACE_ERROR,
                  "%s:%s: unable to read SPI speed on %s\n", driverName,
                  functionName, device);
        return asynDisconnected;
    }
    if (uint != spiSpeed) {
        asynPrint(pasynUser, ASYN_TRACE_ERROR,
                  "%s:%s: unable to set SPI speed on %s\n", driverName,
                  functionName, device);
        return asynDisconnected;
    }

    /* Signal that we're connected */
    pasynManager->exceptionConnect(pasynUser);

    return asynSuccess;
}

asynStatus drvAsynSPI::spi_disconnect(asynUser* pasynUser)
{
    asynStatus status = asynSuccess;

    /* Close i2c port */
    if (close(this->fd) != 0) {
        return asynError;
    }
    this->fd = -1;

    /* Signal that we're disconnected */
    pasynManager->exceptionDisconnect(pasynUser);

    return status;
}

int drvAsynSPI::spi_wr_rd(unsigned char* tx, unsigned char* rx,
                          unsigned short n)
{
    struct spi_ioc_transfer tr;

    tr.tx_buf        = (unsigned long)tx;
    tr.rx_buf        = (unsigned long)rx;
    tr.len           = n;
    tr.delay_usecs   = 0;
    tr.speed_hz      = this->spiSpeed;
    tr.bits_per_word = this->spiBits;

    if (ioctl(fd, SPI_IOC_MESSAGE(1), &tr) < 0) {
        return -1;
    }
    return 0;
}

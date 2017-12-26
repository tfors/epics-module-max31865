/* drvAsynSPI.h
 *
 * EPICS asynPortDriver support for SPI devices under Linux
 * using spidev.
 *
 * Thomas Fors <tom@fors.net>
 * December, 2017
 */

#include "asynPortDriver.h"

class drvAsynSPI : public asynPortDriver {

public:
    drvAsynSPI(const char* portName, unsigned char spiBus,
               unsigned char spiChipSel, unsigned char spiMode,
               unsigned char spiBits, unsigned int spiSpeed, int maxAddrIn,
               int interfaceMask, int interruptMask, int asynFlags,
               int autoConnect, int priority, int stackSize);

protected:
    int fd;
    unsigned char spiBus;
    unsigned char spiChipSel;
    unsigned char spiMode;
    unsigned char spiBits;
    unsigned int spiSpeed;

    asynStatus spi_connect(asynUser* pasynUser);
    asynStatus spi_disconnect(asynUser* pasynUser);
    int spi_wr_rd(unsigned char *tx, unsigned char *rx, unsigned short n);
};

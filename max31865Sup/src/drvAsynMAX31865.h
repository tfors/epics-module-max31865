/* drvAsynMAX31865.h
 *
 * EPICS asynPortDriver for MAX31865 PT100 RTD Temperature
 * Sensor Amplifier under Linux using spidev.

 * Thomas Fors <tom@fors.net>
 * December, 2017
 */

#include <epicsEvent.h>

#include "asynSPIDriver.h"

#define P_TempCString "Temperature(C)" /* asynFloat64 */
#define P_TempFString "Temperature(F)" /* asynFloat64 */

class drvAsynMAX31865 : public asynSPIDriver {

public:
    drvAsynMAX31865(const char* portName, unsigned char spiBus,
                    unsigned char spiChipSel, unsigned char spiMode,
                    unsigned char spiBits, unsigned int spiSpeed);

    virtual asynStatus connect(asynUser* pasynUser);
    virtual asynStatus disconnect(asynUser* pasynUser);

    void pollTask(void);

protected:
    int P_Temp_C;
    int P_Temp_F;

private:
    epicsEventId eventId_;
    double resistance;
    double celsius;

    int read_reg(unsigned char reg, unsigned char* value, unsigned short len);
    int write_reg(unsigned char reg, unsigned char value);
};

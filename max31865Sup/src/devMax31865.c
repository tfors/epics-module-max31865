/* devMax31865.c
 *
 * EPICS device support routines for MAX31865 PT100 RTD Temperature
 * Sensor Amplifier under Linux using spidev.
 *
 * Thomas Fors <tom@fors.net>
 * November, 2017
 */

#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>
#include <math.h>

#include <devSup.h>
#include <recGbl.h>
#include <link.h>
#include <dbAccess.h>
#include <aiRecord.h>
#include <epicsExport.h>
#include <errlog.h>

#define MAX_NUM_PORTS 4

#define DESIRED_MODE 1
#define DESIRED_BITS 8
#define DESIRED_SPEED 5000000

#define R_NOM (100)
#define R_REF (430)

#define RTD_A (3.9083e-3)
#define RTD_B (-5.775e-7)

#define Z1 (-RTD_A)
#define Z2 (RTD_A * RTD_A - (4 * RTD_B)
#define Z3 (4 * RTD_B) / R_NOM)
#define Z4 (2 * RTD_B)

typedef struct port {
    unsigned char bus;
    unsigned char chipSelect;
    unsigned char mode;
    unsigned char bits;
    unsigned int  speed;
    unsigned char present;
    int fd;
} port;

static struct port ports[MAX_NUM_PORTS];

int max31865Config(unsigned char bus, unsigned char chipSelect,
		   unsigned char mode, unsigned char bits, unsigned int speed);
static long report(int level);
static long init(int pass);
static long init_ai(struct aiRecord *pr);
static long read_ai(struct aiRecord *pr);

struct {
    long	number;
    DEVSUPFUN	dev_report;
    DEVSUPFUN	init;
    DEVSUPFUN	init_record;
    DEVSUPFUN	get_ioint_info;
    DEVSUPFUN	read_ai;
    DEVSUPFUN   special_linconv;
} devAiMax31865 = {
    6,
    report,
    init,
    init_ai,
    NULL,
    read_ai,
    NULL
};

epicsExportAddress(dset, devAiMax31865);

int spi_xfer(unsigned char bus, unsigned char chipSelect,
             char *tx, char *rx, int n) {
    unsigned char port = (bus << 1) | chipSelect;
    struct spi_ioc_transfer tr = {
        .tx_buf = (unsigned long)tx,
        .rx_buf = (unsigned long)rx,
        .len = n,
        .delay_usecs = 0,
        .speed_hz = ports[port].speed,
        .bits_per_word = ports[port].bits,
    };

    return ioctl(ports[port].fd, SPI_IOC_MESSAGE(1), &tr);
}

int spi_init(unsigned char bus, unsigned char chipSelect) {
    int ret;
    int fd;
    unsigned char mode;
    unsigned char bits;
    unsigned int speed;

    unsigned char port = (bus << 1) | chipSelect;

    if (ports[port].present) {
        return 0;
    }

    /* Open SPIDEV Device */
    char device[16];
    sprintf(device, "/dev/spidev%d.%d", bus, chipSelect);
    fd = open(device, O_RDWR);
    if (fd < 0) return 1;

    /* Set SPI Mode */
    mode = DESIRED_MODE;
    ret = ioctl(fd, SPI_IOC_WR_MODE, &mode);
    if (ret == -1) return 1;
    mode = 0;
    ret = ioctl(fd, SPI_IOC_RD_MODE, &mode);
    if (ret == -1) return 1;
    if (mode != DESIRED_MODE) return 1;

    /* Set SPI num bits per word */
    bits = DESIRED_BITS;
    ret = ioctl(fd, SPI_IOC_WR_BITS_PER_WORD, &bits);
    if (ret == -1) return 1;
    bits = 0;
    ret = ioctl(fd, SPI_IOC_RD_BITS_PER_WORD, &bits);
    if (ret == -1) return 1;
    if (bits != DESIRED_BITS) return 1;

    /* Set SPI speed */
    speed = DESIRED_SPEED;
    ret = ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed);
    if (ret == -1) return 1;
    speed = 0;
    ret = ioctl(fd, SPI_IOC_RD_MAX_SPEED_HZ, &speed);
    if (ret == -1) return 1;
    if (speed != DESIRED_SPEED) return 1;

    ports[port].fd = fd;
    ports[port].mode = mode;
    ports[port].bits = bits;
    ports[port].speed = speed;
    ports[port].present = 1;

    /* Config MAX31865 to auto sample */
    char rx[2] = {0,};
    ret = spi_xfer(bus, chipSelect, "\x80\xd0", rx, 2);
    if (ret < 1) return 1;

    return 0;
}

static long report(int level) {
    return 0;
}

static long init(int pass) {
    return 0;
}

static long init_ai(struct aiRecord *pr) {
    struct vmeio *pvmeio;

    if (pr->inp.type != VME_IO) {
        recGblRecordError(S_db_badField, (void *)pr,
                          "devMax31865: (init_record) Illegal INP field");
        return(S_db_badField);
    }

    pvmeio = (struct vmeio *) &pr->inp.value;

    unsigned char bus = pvmeio->card;
    unsigned char chipSelect = pvmeio->signal;
    unsigned char port = (bus << 1) | chipSelect;

    if ( (port < 0) || (port >= MAX_NUM_PORTS) ) {
        recGblRecordError(S_db_badField, (void *)pr,
                          "devMax31865: (init_record) Illegal INP field");
        return(S_db_badField);
    }

    if (spi_init(bus, chipSelect) ) {
        recGblRecordError(S_db_notInit, (void *)pr,
                          "devMax31865: (init_record) Error in spi_init");
        return(S_db_notInit);
    }

    return 0;
}

static long read_ai(struct aiRecord *pr) {
    int ret;
    float resistance, celsius, fahrenheit;
    unsigned char bus = pr->inp.value.vmeio.card;
    unsigned char chipSelect = pr->inp.value.vmeio.signal;
    char signal = pr->inp.value.vmeio.parm[0];

    /* Read the MAX31865 RTD registers */
    char rx[3] = {0,};
    ret = spi_xfer(bus, chipSelect, "\x01\x00\x00", rx, 3);
    if (ret < 1) return -1;

    resistance = ((rx[1] << 7) + (rx[2] >> 1)) / 32768.0 * R_REF;
    celsius = (sqrt(Z2 + (Z3*resistance)) + Z1) / Z4;
    fahrenheit = celsius * 9/5 + 32;

    if ( (signal == 'r') || (signal == 'R') ) {
	pr->val = resistance;
    } else if ( (signal == 'c') || (signal == 'C') ) {
	pr->val = celsius;
    } else if ( (signal == 'f') || (signal == 'F') ) {
	pr->val = fahrenheit;
    } else {
	return -1;
    }
             
    pr->udf = 0;
    return 2;
}

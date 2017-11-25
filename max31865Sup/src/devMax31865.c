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

#define MAX_NUM_CHIPS 2

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

typedef struct chip {
    unsigned char ce;
    unsigned char mode;
    unsigned char bits;
    unsigned int  speed;
    unsigned char present;
    int fd;
} chip;

static struct chip chips[MAX_NUM_CHIPS];

int max31865Config(unsigned char ce, unsigned char mode,
                   unsigned char bits, unsigned int speed);
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

int spi_xfer(int ce, char *tx, char *rx, int n) {
    struct spi_ioc_transfer tr = {
        .tx_buf = (unsigned long)tx,
        .rx_buf = (unsigned long)rx,
        .len = n,
        .delay_usecs = 0,
        .speed_hz = chips[ce].speed,
        .bits_per_word = chips[ce].bits,
    };

    return ioctl(chips[ce].fd, SPI_IOC_MESSAGE(1), &tr);
}

int spi_init(int ce) {
    int ret;
    int fd;
    unsigned char mode;
    unsigned char bits;
    unsigned int speed;

    if (chips[ce].present) {
        return 0;
    }

    /* Open SPIDEV Device */
    char device[16];
    sprintf(device, "/dev/spidev0.%d", ce);
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

    chips[ce].fd = fd;
    chips[ce].mode = mode;
    chips[ce].bits = bits;
    chips[ce].speed = speed;
    chips[ce].present = 1;

    /* Config MAX31865 to auto sample */
    char rx[2] = {0,};
    ret = spi_xfer(ce, "\x80\xd0", rx, 2);
    if (ret < 1) return 1;

    return 0;
}

static long report(int level) {
    return 0;
}

static long init(int pass) {
    epicsPrintf("init (pass=%d)\n", pass);
    return 0;
}

static long init_ai(struct aiRecord *pr) {
    struct vmeio *pvmeio;
    epicsPrintf("init_ai\n");

    if (pr->inp.type != VME_IO) {
        recGblRecordError(S_db_badField, (void *)pr,
                          "devMax31865: (init_record) Illegal INP field");
        return(S_db_badField);
    }

    pvmeio = (struct vmeio *) &pr->inp.value;

    if ( (pvmeio->card < 0) || (pvmeio->card >= MAX_NUM_CHIPS) ) {
        recGblRecordError(S_db_badField, (void *)pr,
                          "devMax31865: (init_record) Illegal INP field");
        return(S_db_badField);
    }

    if (spi_init(pvmeio->card) ) {
        recGblRecordError(S_db_notInit, (void *)pr,
                          "devMax31865: (init_record) Error in spi_init");
        return(S_db_notInit);
    }

    epicsPrintf("devMax31865: (init_record) %s\n", pvmeio->parm);

    return 0;
}

static long read_ai(struct aiRecord *pr) {
    int ret;
    int ce = pr->inp.value.vmeio.card;
    char units = pr->inp.value.vmeio.parm[0];

    /* Read the MAX31865 RTD registers */
    char rx[3] = {0,};
    ret = spi_xfer(ce, "\x01\x00\x00", rx, 3);
    if (ret < 1) return -1;

    float r = ((rx[1] << 7) + (rx[2] >> 1)) / 32768.0 * R_REF;
    float t = (sqrt(Z2 + (Z3*r)) + Z1) / Z4;
    
    if ( (units == 'f') || (units == 'F') ) { 
        t = t * 9/5 + 32;
    }
             
    pr->val = t;
    pr->udf = 0;
    return 2;
}

/*
Code to generate  bidirectional DSHOT protocol signals  to a Raspberry
Pi  GPIO output.  This code  uses 'pio'  library allowing  to generate
signal while not loading the main CPU.
*/

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <assert.h>
#include <stdint.h>
#include <time.h>
#include <string.h>

#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "bidshot.pio.h"

// #include "../motor-common.h"
#include "motor-bidshot.h"

// DSHOT version we generate, aka the bitrate.
// It can be one of 300, 600, 1200, ...
// This value may be modified from the program. However, any changes
// made after 'motorImplementationInitialize' do not take effect.
unsigned dshotVersion = 600;

// This is a pointer to the callback function which is called each
// time when a data frame is received and parsed. You are supposed to
// assign it in your code.
void (*bidshotTelemetryCallback)(int telemetryType, int motorIndex, double *value) = NULL;

// Strangely, my ESC does not conform to the specificaion. Its timing
// is 18% faster than the spec. DSHOT_CLOCK_DIVIDER_FACTOR is
// introduced to handle it. If set to the value 1.0, it will produce
// DSHOT frames with the exact timing as specified in the
// standard. Values above 1.0 will make broadcasting slower and values
// under 1.0 faster.
#define DSHOT_CLOCK_DIVIDER_FACTOR 	1.0

// Each PIO state machine generates DSHOT to (max) 2, 4, 8, 16 or 32
// continuous GPIO pins depending on the value of the following
// macro. Higher value means that you may use more pins and less state
// machines (if your motors are connected to continuous GPIO
// numbers). However, it also means that more data is exchanged in FIFOs.
// The optimal case is when you have all
// your motors connected to a single continuous range of GPIOs and you
// set PINS_PER_SM to the nearest power of 2.
// !!! If you change this value, you also have to change it in  !!!
// !!! 'dshot.pio' and recompile to dshot.pio.h                 !!!
#define PINS_PER_SM              8

// Following value indicates how many samples we scan when reading telemetry.
// We need to read 21 bits with up to 4 samples per each bit, but we do not know when exactly
// the telemetry starts. It shall start around 30us after the last bit sent.
// So, we start reading after 20us and then read a few more values to be sure to capture
// the whole message. Leading and trailing waste will be filtered out in Pi5.
// It is wise to set this value to a multiple of 32.
#define BIDSHOT_NUMBER_OF_SAMPLES_TO_READ (3*32)
// Delay to allow line switching. It is the time in micro seconds between we send the
// last bit and before we start to read telemetries.
#define BIDSHOT_TELEMETRY_DELAY_US     20

// Following special values are used in my simple protocol btween PIO and PI5 for
// possible resynchronization. They are used in the TX/RX FIFOs respectively.
// Shall be small values (max 5 and 10) bits respectively.
// The following value indicates the begin of dshot frames to be send out
#define MAGIC_NUMBER   		      0x13
// The value indicating the end of received data (end of samples)
#define MAGIC_NUMBER2  		      0x5a

// How many PIO SM machine cycles it takes to send a single dshot bit
// Do not change this value unless you recode bidshot.pio
#define DSHOT_CYCLES_PER_BIT_SENT         8
// How many PIO SM machine cycles it takes to read a single sample
#define DSHOT_CYCLES_PER_SAMPLE_RECEIVED  2

// Maximal number of PIO state machines. This number comes from RP1 hardware.
#define DSHOT_PIO_SM_MAX		4

// Max GPIO pins we can handle
#define DSHOT_NUM_PINS           	32

// DSHOT frame length from dshot specification
#define DSHOT_FRAME_LEN          	16

// How many FIFO words to read to get telemetry data
#define BIDSHOT_TELEMETRY_SAMPLES_WORDS  (BIDSHOT_NUMBER_OF_SAMPLES_TO_READ * PINS_PER_SM / 32)
#define SM_SAMPLES_IN_WORD 	        (32/PINS_PER_SM)

// Values for extended telemetry
#define DSHOT_EXT_TELEMETRY_TEMPERATURE 	0x02
#define DSHOT_EXT_TELEMETRY_VOLTAGE 		0x04
#define DSHOT_EXT_TELEMETRY_CURRENT 		0x06
#define DSHOT_EXT_TELEMETRY_STRESS_LEVEL	0x0c
#define DSHOT_EXT_TELEMETRY_STATUS		0x0e

#define DEBUG_LEVEL   0
#define WARNING_LEVEL 0

////////////////////////////////////////////////////////////////////////////

struct dshotPioPins {
    int pinBase;
    int pinCount;
};

struct pinToSmStr {
    int sm;
    int mask;
};

// the /dev/pio0 link.
static PIO  dshotPio;
// state machines index acquired from pio lib
static int  dshotPioSmi[DSHOT_PIO_SM_MAX];
// pin distribution among state machines
static struct dshotPioPins dshotSm[DSHOT_PIO_SM_MAX];
// the state machine and offset for each pin
static struct pinToSmStr dshotPinToSm[DSHOT_NUM_PINS];
// the motor index for each pin
static int dshotPinToMotori[DSHOT_NUM_PINS];
// offset where our binary is loaded into state machines memory.
static uint dshotLoadedOffset;
// Flag whether we are in 3D mode, if dshot3dMode != 0 then reverse rotation is enabled.
static int dshot3dMode = 0;

////////////////////////////////////////////////////////////////////////

static unsigned dshotAddChecksumAndTelemetry(int packet, int telem) {
    unsigned value, crc;
    value = (packet << 1) | (telem & 1);
    crc = (~(value ^ (value >> 4) ^ (value >> 8))) & 0x0F;
    return ((value<<4) | crc);
}

// precompute continuous areas of pins so that all pins are covered by 4 state machines
static int dshotAssignPinsToStateMachines(int motorPins[], int motorMax) {
    int		i, smi, pin, base, count;
    int8_t 	pinMap[DSHOT_NUM_PINS];

    memset(pinMap, 0, sizeof(pinMap));
    for(i=0; i<motorMax; i++) {
	pin = motorPins[i];
	assert(pin >= 0 && pin < DSHOT_NUM_PINS);
	pinMap[pin] = 1;
    }
    // assign continuos regions of pins to state machines
    memset(dshotSm, 0, sizeof(dshotSm));
    i = 0;
    while(pinMap[i] == 0 && i<DSHOT_NUM_PINS) i++;
    // No pin's found
    if (i>=DSHOT_NUM_PINS) return(-2);
    for(smi=0; smi<DSHOT_PIO_SM_MAX; smi++) {
	base = i;
	while(pinMap[i] != 0 && i<base+PINS_PER_SM && i<DSHOT_NUM_PINS) i++;
	count = i-base;
	dshotSm[smi].pinBase = base;
	dshotSm[smi].pinCount = count;
	if (DEBUG_LEVEL >= 5) {
	    fprintf(stdout, "SM%d: pinBase: %d; pinCount: %d\n", smi, dshotSm[smi].pinBase, dshotSm[smi].pinCount);
	}
	while(pinMap[i] == 0 && i<DSHOT_NUM_PINS) i++;
	if (i>=DSHOT_NUM_PINS) return(0);
    }
    // Hmm I am running out of state machines
    return(-1);
}

// precompute mapping from pin number to assigned state machine and pin's bit masks
static void dshotPinToSmTableInit() {
    int smi, j, pin;

    memset(dshotPinToSm, 0, sizeof(dshotPinToSm));
    for(smi=0; smi<DSHOT_PIO_SM_MAX; smi++) {
	for(j=0; j<dshotSm[smi].pinCount; j++) {
	    pin = dshotSm[smi].pinBase + j;
	    dshotPinToSm[pin].sm = smi;
	    dshotPinToSm[pin].mask = (1 << j);
	}
    }
}

// precompute mapping from pin number to motor index
static void dshotPinToMotoriInit(int motorPins[], int motorMax) {
    int i, pin;

    for(i=0; i<DSHOT_NUM_PINS; i++) dshotPinToMotori[i] = -1;
    for(i=0; i<motorMax; i++) {
	pin = motorPins[i];
	dshotPinToMotori[pin] = i;
    }
}

static inline int bidshotEstimateNumberOfBitsInSampleSuite(int len) {
    int res;
    // If we receive len consecutive values, estimate how many bits are there.
    // We expect to receive bits at the rate 5/4 * sending_bitrate
    // TODO: Maybe add consideration of current clock divider. However, it is a 'fixed point' number with 8 bits
    // fractional precission, so the rounding generates only a very little bias.
    res = (5 * len * DSHOT_CYCLES_PER_SAMPLE_RECEIVED + 2 * DSHOT_CYCLES_PER_BIT_SENT) / (4 * DSHOT_CYCLES_PER_BIT_SENT);
    // printf("est len %d --> bits %d\n", len, res);
    return(res);
}

// Parse samples coming from 1 ESC to a 21 bits word
static int32_t bidshotSampleBitsTo21BitMsg(uint8_t b[BIDSHOT_TELEMETRY_SAMPLES_WORDS*4]) {
    int 	i, si, bb, bits, totalBits;
    int32_t	res;

    res = 0;
#if 0
    fprintf(stdout, "Scanning samples: ");
    for(i=0; i<BIDSHOT_TELEMETRY_SAMPLES_WORDS*4; i++) {
	fprintf(stdout, "%d", b[i]);
    }
    fprintf(stdout, "\n");
    fflush(stdout);
#endif
    if (b[0] != 1) {
	if (WARNING_LEVEL > 0) fprintf(stderr,"Warning: a sample does not start with 1. Maybe too long BIDSHOT_TELEMETRY_DELAY_US.\n");
	return(-1);
    }
    // find the beginning of the broadcasting
    // TODO: Maybe use a sentinel by setting b[BIDSHOT_TELEMETRY_SAMPLES_WORDS*4-1] = 0xff;
    for(si=0; si<BIDSHOT_TELEMETRY_SAMPLES_WORDS*4 && b[si] == 1; si++) ;
    // parsing GCR means searching for sequences of consequitive zeros and ones and estimating
    // the number of bits included in such a sequence from its length. The actual frequency
    // is less important in this 'self clocking' protocol.
    totalBits = 0;
    i = si;
    for(;;) {
	// search for consecutive zeros
	for(bb=i; i<BIDSHOT_TELEMETRY_SAMPLES_WORDS*4 && b[i] == 0; i++) ;
	// we got i-bb consecutive zeros, estimate how many zero bits are there
	bits = bidshotEstimateNumberOfBitsInSampleSuite(i-bb);
	if (bits + totalBits > 21) bits = 21 - totalBits;
	// add those bits to the result
	res = res << bits;
	totalBits += bits;
	if (totalBits >= 21) break;
	// Do the same for consecutive ones
	for(bb=i; i<BIDSHOT_TELEMETRY_SAMPLES_WORDS*4 && b[i] == 1; i++) ;
	// we got i-bb consecutive ones, estimate how many one bits are there
	bits =  bidshotEstimateNumberOfBitsInSampleSuite(i-bb);
	if (bits + totalBits > 21) bits = 21 - totalBits;
	res = ((res+1) << bits) - 1;
	totalBits += bits;
	// fprintf(stdout, "Got 1 between %d-%d: bits = %d, res == %x\n", bb, i, bits, res);
	if (totalBits >= 21) break;
	// if we run out of sample and still do not have 21 bits, return error
	if (i>=BIDSHOT_TELEMETRY_SAMPLES_WORDS*4) {
	    res = -1;
	    break;
	}
    }
    // fprintf(stdout, "Scanned to %x\n", res);     fflush(stdout);
    return(res);
}


static void bidshotSamplesTo21BitMsgs(uint32_t d[BIDSHOT_TELEMETRY_SAMPLES_WORDS], int32_t msg[PINS_PER_SM], int pinCount) {
    int 	i,j,k,ii;
    uint32_t	bb;
    uint8_t	b[PINS_PER_SM][BIDSHOT_NUMBER_OF_SAMPLES_TO_READ];

    for(k=0; k<PINS_PER_SM; k++) msg[k] = -1;
    k = 0;
    for(i=0; i<BIDSHOT_TELEMETRY_SAMPLES_WORDS; i++) {
	for(j=0; j<SM_SAMPLES_IN_WORD; j++) {
	    bb = (d[i] >> j*PINS_PER_SM);
	    for(k=0; k<PINS_PER_SM; k++) {
		ii = i*SM_SAMPLES_IN_WORD+j;
		assert(ii>=0 && ii<BIDSHOT_NUMBER_OF_SAMPLES_TO_READ);
		// fprintf(stdout, "set b[%d][%d] = %d;\n", k, ii, (bb >> k) & 0x1);
		b[k][ii] = (bb >> k) & 0x1;
	    }
	}
    }

    for(k=0; k<pinCount; k++) {
	msg[k] = bidshotSampleBitsTo21BitMsg(b[k]);
    }
}

static void bidshotDecodeM16ToTelemetry(int32_t m16, int motorIndex) {
    int32_t	crc, em, emcrc, exp, base;
    int32_t	ttype, val;
    double	rpm, value, periodUs;
    
    crc = m16 & 0x0f;
    em = (m16 >> 4);
    emcrc = (~(em ^ (em >> 4) ^ (em >> 8))) & 0x0F;
    if (crc != emcrc) {
	if (WARNING_LEVEL >= 20) fprintf(stderr, "Warning: Motor %d: Wrong crc: %x != %x\n", motorIndex, crc, emcrc);
	return;
    }
    
    if ((em & 0xf00) == 0 || (em & 0x100) != 0) {
	// stadard rpm frame
	base = (em & 0x1ff);
	exp = (em >> 9);
	periodUs = (base << exp);
	if (DEBUG_LEVEL > 0) {
	    // avoid division by zero.
	    if (periodUs == 0) periodUs = 1;
	    rpm = 60000000.0 / (periodUs * 7.0); // common motors have 14 poles
	    fprintf(stdout, "Info: Motor %d: Got periodUs: %5.0f --> RPM: %8.0f\n", motorIndex, periodUs, rpm);
	}
	if (bidshotTelemetryCallback != NULL) bidshotTelemetryCallback(BIDSHOT_TT_PERIOD_US, motorIndex, &periodUs);
    } else {
	// extended telemetry frame
	ttype = (em >> 8);
	val = (em & 0xff);
        switch(ttype) {
	case DSHOT_EXT_TELEMETRY_TEMPERATURE:
	    value = val;
	    if (DEBUG_LEVEL > 0) fprintf(stdout, "Info: Motor %d: Got temperature: %f\n", motorIndex, value);
	    if (bidshotTelemetryCallback != NULL) bidshotTelemetryCallback(BIDSHOT_TT_TEMPERATURE, motorIndex, &value);
	    break;
	case DSHOT_EXT_TELEMETRY_VOLTAGE:
	    value = val * 0.25;
	    if (DEBUG_LEVEL > 0) fprintf(stdout, "Info: Motor %d: Got voltage: %f\n", motorIndex, value);
	    if (bidshotTelemetryCallback != NULL) bidshotTelemetryCallback(BIDSHOT_TT_VOLTAGE, motorIndex, &value);
	    break;
	case DSHOT_EXT_TELEMETRY_CURRENT:
	    value = val;
	    if (DEBUG_LEVEL > 0) fprintf(stdout, "Info: Motor %d: Got current %f\n", motorIndex, value);
	    if (bidshotTelemetryCallback != NULL) bidshotTelemetryCallback(BIDSHOT_TT_CURRENT, motorIndex, &value);
            break;
	case DSHOT_EXT_TELEMETRY_STRESS_LEVEL:
	    value = val;
	    if (DEBUG_LEVEL > 0) fprintf(stdout, "Info: Motor %d: Got stress level %f\n", motorIndex, value);
	    if (bidshotTelemetryCallback != NULL) bidshotTelemetryCallback(BIDSHOT_TT_STRESS_LEVEL, motorIndex, &value);
            break;
	case DSHOT_EXT_TELEMETRY_STATUS:
	    value = val;
	    if (DEBUG_LEVEL > 0) fprintf(stdout, "Info: Motor %d: Got status %x\n", motorIndex, val);
	    if (bidshotTelemetryCallback != NULL) bidshotTelemetryCallback(BIDSHOT_TT_STATUS, motorIndex, &value);
            break;
	default:
	    value = val;
	    if (WARNING_LEVEL > 10) fprintf(stdout, "Warning: Motor %d: Unknow extended telemetry received %d, value %f\n", motorIndex, ttype, value);
	    if (bidshotTelemetryCallback != NULL) bidshotTelemetryCallback(BIDSHOT_TT_NONE, motorIndex, &value);
            break;
	}
    }
}

static uint8_t decodeGCRNibble(uint8_t gcr) {
    // from https://brushlesswhoop.com/dshot-and-bidirectional-dshot/
    switch(gcr)	{
    case 0x19:
	return 0x0;
    case 0x1B:
	return 0x1;
    case 0x12:
	return 0x2;
    case 0x13:
	return 0x3;
    case 0x1D:
	return 0x4;
    case 0x15:
	return 0x5;
    case 0x16:
	return 0x6;
    case 0x17:
	return 0x7;
    case 0x1A:
	return 0x8;
    case 0x09:
	return 0x9;
    case 0x0A:
	return 0xA;
    case 0x0B:
	return 0xB;
    case 0x1E:
	return 0xC;
    case 0x0D:
	return 0xD;
    case 0x0E:
	return 0xE;
    case 0x0F:
	return 0xF;
    default:
	if (WARNING_LEVEL >= 30) fprintf(stderr, "Warning: Unexpected nibble value %d\n", gcr);
	return 0xFF;
    }
}

static void bidshotMsg21ToMsg16(int32_t msg[PINS_PER_SM], int32_t msg16[PINS_PER_SM], int pinCount) {
    int			i, j;
    int32_t 		m21, m20, m16;
    uint8_t 		quintet, nibble;
    
    for(i=0; i<pinCount; i++) {
	msg16[i] = -1;
	m21 = msg[i];
	// a negative value means an error at reading
	if (m21 < 0) goto nextpin;
	// shrink to 20 bits
	m20 = (m21 ^ (m21 >> 1));
	// fprintf(stdout, "m21 == %x; m20 == %x;\n", m21, m20);
	m16 = 0;
	for(j=0; j<4; j++) {
	    // break the 20 bits into 4 quintets for converting back to regular 16 bit binary
	    quintet = (m20 >> 15) & 0x1f;
	    m20 = (m20 << 5);
	    nibble = decodeGCRNibble(quintet);
	    if (nibble > 0x0f) goto nextpin;
	    m16 = (m16 << 4) | nibble;
	}
	// I should have 16 bit telemetry in m16
	if (DEBUG_LEVEL > 10) fprintf(stdout, "     Telemetry msg from pin %d is decoded as erpm: %x\n", i, m16);
	msg16[i] = m16;
    nextpin:;
    }
}

static void bidshotFlushRxFifo(int i) {
    int n, sm;
    
    // read everything what is left in RX FIFO of state machine i (but at most 100 words)
    sm = dshotPioSmi[i];
    for(n=0; n<100 && pio_sm_get_rx_fifo_level(dshotPio, sm) > 0; n++) {
	pio_sm_get(dshotPio, sm);
    }
}

static void dshotGetAndParseTelemetrySamples(uint16_t *b, int smi) {
    // for a possible future assync DMA use static array
    static uint32_t 	d[DSHOT_PIO_SM_MAX][BIDSHOT_TELEMETRY_SAMPLES_WORDS+1];
    int32_t 		msg21[PINS_PER_SM];
    int32_t 		msg16[PINS_PER_SM];
    int			r, sm, j, pin, motorIndex;
    
    sm = dshotPioSmi[smi];
    if (DEBUG_LEVEL >= 10) {
	fprintf(stdout, "SM%d: Starting read transfer of %lu bytes\n", smi, (BIDSHOT_TELEMETRY_SAMPLES_WORDS+1)*sizeof(d[0][0]));
	fflush(stdout);
    }

    r = pio_sm_xfer_data(dshotPio, sm, PIO_DIR_FROM_SM, (BIDSHOT_TELEMETRY_SAMPLES_WORDS+1)*sizeof(d[0][0]), d[smi]);
    if (r) {
	if (WARNING_LEVEL >= 10) fprintf(stderr, "Warning: SM%d: pio_sm_xfer_data from PIO to Pi5 returned %d\n", smi, r);
	return;
    }

    if (DEBUG_LEVEL >= 30) {
	fprintf(stdout, "SM%d: First word is %x; Last word is %x\n", smi, d[smi][0], d[smi][BIDSHOT_TELEMETRY_SAMPLES_WORDS]);
	fflush(stdout);
    }

    if (d[smi][BIDSHOT_TELEMETRY_SAMPLES_WORDS] != MAGIC_NUMBER2) {
	if (WARNING_LEVEL >= 10) fprintf(stderr, "Warning: SM%d: data samples do not end with magic number %x\n", smi, MAGIC_NUMBER2);
	bidshotFlushRxFifo(smi);
	return;
    }
    
    if (DEBUG_LEVEL >= 10) {
	fprintf(stdout, "SM%d: Got samples: least 4 bits:  ", smi);
	for(j=0; j<BIDSHOT_TELEMETRY_SAMPLES_WORDS; j++)  {
	    fprintf(stdout, "%1x%1x%1x%1x", d[smi][j]&0xf, (d[smi][j]>>8)&0xf, (d[smi][j]>>16)&0xf, (d[smi][j]>>24)&0xf);
	}
	fprintf(stdout, "\n");
	fflush(stdout);
    }
    
    // Combine telemetry readings to 21 bits msg
    bidshotSamplesTo21BitMsgs(d[smi], msg21, dshotSm[smi].pinCount);
    bidshotMsg21ToMsg16(msg21, msg16, dshotSm[smi].pinCount);
    for(j=0; j<dshotSm[smi].pinCount; j++) {
	pin = dshotSm[smi].pinBase + j;
	motorIndex = dshotPinToMotori[pin];
	bidshotDecodeM16ToTelemetry(msg16[j], motorIndex);
    }
}

// Pack DSHOT frames from buffer 'b' into 32bits words and send it 'i'-th state machine TX FIFO.
static void dshotPackFramesIntoFifoUsingDma(uint16_t *b, int smi) {
    // for a possible future assync DMA use static array
    static uint32_t 	d[DSHOT_PIO_SM_MAX][DSHOT_FRAME_LEN+4];
    int 		r, dlen, k, j, sm, waitCounter, nsamples;
    uint32_t    	bb;
    
    sm = dshotPioSmi[smi];
    dlen = 0;
    // first push MAGIC_NUMBER
    d[smi][dlen++] = MAGIC_NUMBER;
    // push bidshot frames
    for(j=0; j<DSHOT_FRAME_LEN; j+=32/PINS_PER_SM) {
	bb = 0;
	for(k=0; k<32/PINS_PER_SM; k++) {
	    bb |= (b[j+k] << k*PINS_PER_SM);
	}
	// bidirectional
	bb = ~ bb;
	d[smi][dlen++] = bb;
    }
    // add zero to set pin directions to 'read'
    d[smi][dlen++] = 0;
    // add wait counter, number of samples requested and MAGIC_NUMBER2
    // wait counter shall lead to around 25us delay in an empty PIO SM loop
    waitCounter = BIDSHOT_TELEMETRY_DELAY_US * DSHOT_CYCLES_PER_BIT_SENT * dshotVersion / 1000;
    if (waitCounter < 2) waitCounter = 2;
    nsamples = BIDSHOT_NUMBER_OF_SAMPLES_TO_READ;
    d[smi][dlen++] = ((((uint32_t)MAGIC_NUMBER2 << 11)  + nsamples-1) << 11) + (waitCounter-2);
    // add word with all bits 1 to set pin directions back to 'write'
    d[smi][dlen++] = 0xffffffff;
    
    // all ready: check length, print debug and send it to SM
    assert(dlen <= sizeof(d[smi]) / sizeof(d[smi][0]));		// dlen < DIM(d[i])
    if (DEBUG_LEVEL > 20) {
	fprintf(stdout, "SM%d: sending: ", smi);
	for(j=0; j<dlen; j++)  {
	    // fprintf(stdout, "%d%d%d%d", d[smi][j]&0x1, (d[smi][j]>>8)&0x1, (d[smi][j]>>16)&0x1, (d[smi][j]>>24)&0x1);
	    fprintf(stdout, "%08x", d[smi][j]);
	}
	fprintf(stdout, "; req. %d samples; waitCounter == %d\n", nsamples, waitCounter);
	fflush(stdout);
    }
    r = pio_sm_xfer_data(dshotPio, sm, PIO_DIR_TO_SM, dlen * sizeof(d[0][0]), d[smi]);
    if (r) {
	if (WARNING_LEVEL >= 10) fprintf(stderr, "Warning: SM%d: pio_sm_xfer_data from Pi5 to PIO returned %d\n", smi, r);
    }
    
}

// send dshot 'frames' and read telemetry to/from 'motorPins'
static void bidshotSendFramesAndGetTelemetry(int motorPins[], int motorMax, unsigned frames[]) {
    int         smi, bi;
    unsigned    bit;
    uint16_t    b[DSHOT_PIO_SM_MAX][DSHOT_FRAME_LEN];
    int	      	pin, sm, mask;
    
    assert(motorMax < DSHOT_NUM_PINS);

    memset(b, 0, sizeof(b));

    // precompute broadcasting to state machines for each message bit
    for(smi=0; smi<motorMax; smi++) {
	pin = motorPins[smi];
	assert(pin < DSHOT_NUM_PINS);
	sm = dshotPinToSm[pin].sm;
	mask = dshotPinToSm[pin].mask;
	bit = 0x8000;
	for(bi=0; bi<DSHOT_FRAME_LEN; bi++) {
	    if ((frames[smi] & bit)) b[sm][bi] |= mask;
	    bit = (bit >> 1);
	}
    }

    // send precomputed frames to each state machine and get telemetry answer
    for(smi=0; smi<DSHOT_PIO_SM_MAX; smi++) {
	sm = dshotPioSmi[smi];
	if (sm >= 0 && dshotSm[smi].pinCount > 0) {
	    if (pio_sm_is_tx_fifo_empty(dshotPio, sm)) {
		dshotPackFramesIntoFifoUsingDma(b[smi], smi);
		dshotGetAndParseTelemetrySamples(b[smi], smi);
	    } else {
		if (WARNING_LEVEL >= 20) fprintf(stderr, "Warning: SM%d: a dshot frame skipped\n", smi);
	    }
	}
    }
}

static void dshotPioStateMachineInit(PIO pio, uint sm, uint offset, uint pin, uint pincount, double clkDivider) {
    pio_sm_set_consecutive_pindirs(pio, sm, pin, pincount, true);
    pio_sm_config c = dshot_program_get_default_config(offset);
    sm_config_set_in_pins(&c, pin);
    sm_config_set_in_shift(&c, true, true, 32);
    sm_config_set_out_pins(&c, pin, pincount);
    sm_config_set_out_shift(&c, true, true, 32);
    sm_config_set_set_pins(&c, pin, pincount);
    sm_config_set_clkdiv (&c, clkDivider);
    pio_sm_init(pio, sm, offset, &c);
}

static unsigned dshotThrottleToDshotFrame(double tt) {
    int 	ff;
    unsigned 	res;
    
    if (dshot3dMode) {
	// translate throttles ranging <-1, 1> to dshot frames.
	if (tt >= 0) {
	    ff = tt * 999 + 1048;
	} else {
	    ff = -tt * 999 + 48;
	}
    } else {
	// translate throttles ranging <0, 1> to dshot frames.
	ff = tt * 1999 + 48;
    }
    // I used to issue DSHOT_CMD_MOTOR_STOP if thrust == 0. It is not possible anymore
    // because in 3d mode it seems to reset the motor. So, you have to arm motors in some different way.
    // Now the DSHOT_CMD_MOTOR_STOP is issued only for values out of range.
    if (/*tt == 0 || */ ff < 48 || ff >= 2048) ff = DSHOT_CMD_MOTOR_STOP;
    res = dshotAddChecksumAndTelemetry(ff, 0);
    return(res);
}

//////////////////////////////////////////////////////////////////////////////////////////////
// Main exported functions of the module implementing raspilot motor instance.
//////////////////////////////////////////////////////////////////////////////////////////////

// Send a command repeatedly. It is used to arm motors
void dshotRepeatSendCommand(int motorPins[], int motorMax, int cmd, int telemetry, int repeatCounter) {
    unsigned    ff;
    unsigned    frame[DSHOT_NUM_PINS+1];
    int         i;

    ff = dshotAddChecksumAndTelemetry(cmd, telemetry);
    for(i=0; i<motorMax; i++) frame[i] = ff;
    for(i=0; i<repeatCounter; i++) {
        bidshotSendFramesAndGetTelemetry(motorPins, motorMax, frame);
        usleep(1000);
    }
}

// This function allows to set bidirectional rotation (mode3dFlag!=0) and reverse rotation logic (reverseDirectionFlag!=0).
// Changing 3D mode is interfering with rotation direction (at least on my ESC), so always reset the direction when changing 3D
void motorImplementationSet3dModeAndSpinDirection(int motorPins[], int motorMax, int mode3dFlag, int reverseDirectionFlag) {
    int         repeatCounter;

    // First, stop/arm motors for around 3 seconds. Strangely my ESC requires so much time to re-init.
    dshotRepeatSendCommand(motorPins, motorMax, DSHOT_CMD_MOTOR_STOP, 0, 3000);

    // Those commands have to be sent repeatedly in order to be applied.
    repeatCounter = 10;

    dshot3dMode = mode3dFlag;
    // Set 3d mode. Some sources say that the telemetry bit shall be set in this command. It seems to
    // work both ways.
    if (dshot3dMode) {
        dshotRepeatSendCommand(motorPins, motorMax, DSHOT_CMD_3D_MODE_ON, 1, repeatCounter);
    } else {
        dshotRepeatSendCommand(motorPins, motorMax, DSHOT_CMD_3D_MODE_OFF, 1, repeatCounter);
    }
    dshotRepeatSendCommand(motorPins, motorMax, DSHOT_CMD_SAVE_SETTINGS, 0, repeatCounter);
    usleep(50000);

    // Set rotation direction mode. Telemetry as previously.
    if (reverseDirectionFlag) {
        dshotRepeatSendCommand(motorPins, motorMax, DSHOT_CMD_SPIN_DIRECTION_REVERSED, 1, repeatCounter);
    } else {
        dshotRepeatSendCommand(motorPins, motorMax, DSHOT_CMD_SPIN_DIRECTION_NORMAL, 1, repeatCounter);
    }
    dshotRepeatSendCommand(motorPins, motorMax, DSHOT_CMD_SAVE_SETTINGS, 0, repeatCounter);
    usleep(50000);
}

void motorImplementationInitialize(int motorPins[], int motorMax) {
    int    smi, r;
    double divider;
    
    // initialize pio library
    stdio_init_all();
    dshotPio = pio0;
    
    r = dshotAssignPinsToStateMachines(motorPins, motorMax) ;
    if (r != 0) {
	fprintf(stderr, "Error: MotorPins aren't in %d continuous intervals. Can't map.\n", DSHOT_PIO_SM_MAX);
    }

    if (BIDSHOT_NUMBER_OF_SAMPLES_TO_READ * PINS_PER_SM % 32 != 0) {
	fprintf(stderr, "Error: BIDSHOT_NUMBER_OF_TELEMETRY_SAMPLES * PINS_PER_SM is not a multiple of 32.\n");
    }

    dshotPinToSmTableInit() ;
    dshotPinToMotoriInit(motorPins, motorMax);
    
    for(smi=0; smi<motorMax; smi++) {
	pio_gpio_init(dshotPio, motorPins[smi]);
	gpio_set_pulls(motorPins[smi], true, false);  // up, down
    }

    for(smi=0; smi<DSHOT_PIO_SM_MAX; smi++) {
	if (dshotSm[smi].pinCount > 0) {
	    dshotPioSmi[smi] = pio_claim_unused_sm(dshotPio, false);
	    if (dshotPioSmi[smi] < 0) fprintf(stderr, "Error: Can't claim state machine number %d\n", smi);
	}
    }

    dshotLoadedOffset = pio_add_program(dshotPio, &dshot_program);

    // Set clock divider to the value specified in dshot specification (we use DSHOT_CYCLES_PER_BIT ticks to broadcast one bit).
    divider = (double)clock_get_hz(clk_sys) / (dshotVersion * 1000.0 * DSHOT_CYCLES_PER_BIT_SENT);
    // My cheap ESC is nearly 20% off the standard. Adjust divider by an ad-hoc value.
    divider *= DSHOT_CLOCK_DIVIDER_FACTOR;
    if (DEBUG_LEVEL > 20) printf("Info: clock hz == %d; divider == %f\n", clock_get_hz(clk_sys), divider);
    if (divider < 1.0) {
	fprintf(stderr, "Error: clock frequency is too small for this timing.\n");
	divider = 1.0;
    }
    
    for(smi=0; smi<DSHOT_PIO_SM_MAX; smi++) {
	if (dshotSm[smi].pinCount > 0 && dshotPioSmi[smi] >= 0) {
	    pio_sm_config_xfer(dshotPio, dshotPioSmi[smi], PIO_DIR_TO_SM, 1024, 1);
	    pio_sm_config_xfer(dshotPio, dshotPioSmi[smi], PIO_DIR_FROM_SM, 1024, 1);
	    // printf("init program in sm %d, offset %d, pinbase %d, pincount %d\n", smi[i], loaded_offset, dshotSm[i].pinBase, dshotSm[i].pinCount);
	    dshotPioStateMachineInit(dshotPio, dshotPioSmi[smi], dshotLoadedOffset, dshotSm[smi].pinBase, dshotSm[smi].pinCount, divider);
	    pio_sm_set_enabled(dshotPio, dshotPioSmi[smi], true);
	}
    }
    
}

void motorImplementationFinalize(int motorPins[], int motorMax) {
    /* Not yet implemented. 
     */
    int smi;
    
    for(smi=0; smi<DSHOT_PIO_SM_MAX; smi++) {
	if (dshotPioSmi[smi] >= 0) {
	    pio_sm_set_enabled(dshotPio, dshotPioSmi[smi], false);
	}
    }

    for(smi=0; smi<motorMax; smi++) {
	gpio_set_pulls(motorPins[smi], false, true);  // up, down
    }

    /*
    // Strangely, this code makes state machines to freeze
    // TODO: figure out the proper deinitialization of RP1 state machines

    // pio_remove_program (pio, &dshot_program, loaded_offset);
    for(smi=0; smi<DSHOT_PIO_SM_MAX; smi++) {
	if (dshotPioSmi[smi] >= 0) {
	    pio_sm_unclaim(dshotPio, dshotPioSmi[smi]);
	}
    }
    */
}

void motorImplementationSendThrottles(int motorPins[], int motorMax, double motorThrottle[]) {
    int         i;
    unsigned    frames[DSHOT_NUM_PINS+1];

    assert(motorMax < DSHOT_NUM_PINS);

    for(i=0; i<motorMax; i++) {
        frames[i] = dshotThrottleToDshotFrame(motorThrottle[i]);
    }
    bidshotSendFramesAndGetTelemetry(motorPins, motorMax, frames);
}

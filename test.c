#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#include "motor-bidshot.h"

#define MAXN 	40

int motorPins[MAXN];
double throttles[MAXN];

// This is the fuction which will be executed each time when a telemetry
// frame is received from any motor. Pointer to it shall be assigned to
// the variable bidshotTelemetryCallback. Arguments are:
// - the type of telemetry read
// - the index of the motor from where it comes
// - a pointer to the actual value received
void myTelemetryCallback(int ttype, int motorIndex, double *value) {
    static int 	counter;
    double 	rpm;
    unsigned char	status;

    switch(ttype) {
    case BIDSHOT_TT_PERIOD_US:
	// print only one of 100 of those messages
	if (counter ++ % 100 != 0) break;
	if (*value < 256) {
	    // There seems to be a bug in my ESC. He is sending those values
	    // when motor turns slowly while those values are ment for fast rotation
	    rpm = 0;
	} else if (*value >= 65408) {
	    // max value means motor is stopped
	    rpm = 0;
	} else {
	    rpm = 60000000.0 / (*value * 12.0); // 12 is for my motor. Count your poles to get your RPM
	}
	fprintf(stdout, "Motor %d: RPM: %8.0f\n", motorIndex, rpm);
	break;
	
    case BIDSHOT_TT_TEMPERATURE:
	fprintf(stdout, "Motor %d: temperature: %f\n", motorIndex, *value);
	break;
    case BIDSHOT_TT_VOLTAGE:
	fprintf(stdout, "Motor %d: voltage: %f\n", motorIndex, *value);
	break;
    case BIDSHOT_TT_CURRENT:
	fprintf(stdout, "Motor %d: current %f\n", motorIndex, *value);
	break;
    case BIDSHOT_TT_STRESS_LEVEL:
	fprintf(stdout, "Motor %d: stress level %f\n", motorIndex, *value);
	break;
    case BIDSHOT_TT_STATUS:
	// print only one of 100 of those messages
	if (counter ++ % 100 != 0) break;
	status = (unsigned char) *value;
	fprintf(stdout, "Motor %d: status: ", motorIndex);
	if (status & 0x80) fprintf(stdout, "alert ");
	if (status & 0x40) fprintf(stdout, "warning ");
	if (status & 0x20) fprintf(stdout, "error ");
	fprintf(stdout, "stress level %d\n", status & 0x0f);
	break;
    default:
	fprintf(stdout, "An unknow telemetry received: type %d: motor %d; value %f\n", ttype, motorIndex, *value);
	break;
    }
    fflush(stdout);
}

//////////////////////////////////////////////////////////////////////////////////////////


int main(int argc, char **argv) {
    int i, n;

    if (argc < 2) {
	printf("usage:   ./test <gpio_pin_1> ... <gpio_pin_n>\n");
	printf("example: sudo ./test 16 19 20 21\n");
	exit(1);
    }

    // Parse argumens which are GPIO pins where motors are connected
    n = 0;
    for(i=1; i<argc; i++) {
	motorPins[n] = atoi(argv[i]);
	if (motorPins[n] > 0 && motorPins[n] < 28) {
	    n ++;
	} else {
	    printf("pin %d out of range 1..27\n", motorPins[n]);
	}
    }

    // set the callback for received telemetry
    bidshotTelemetryCallback = myTelemetryCallback;
    
    // init bidirectional dshot for my motors
    motorImplementationInitialize(motorPins, n);
    usleep(100000);
    
    printf("Initializing ESC / Arm, waiting 5 seconds.\n"); fflush(stdout);
    // Arm motors, to arm send DSHOT_CMD_MOTOR_STOP for 5 seconds
    dshotRepeatSendCommand(motorPins, n, DSHOT_CMD_MOTOR_STOP, 0, 5000);
    usleep(100000);
    
    // Enable receiving extended telemetry, i.e. temperature, voltage, ...
    dshotRepeatSendCommand(motorPins, n, DSHOT_CMD_EXTENDED_TELEMETRY_ENABLE, 0, 1000);
    usleep(100000);

    // Start rotating motors
    printf("Spinning.\n");
    fflush(stdout);
    // make motors spinning on 15% throttle during 5 seconds
    for(i=0; i<n; i++) throttles[i] = 0.15;
    for(i=0; i<5000; i++) {
	motorImplementationSendThrottles(motorPins, n, throttles);
	usleep(1000);
    }
    
    printf("Stop.\n");
    fflush(stdout);
    // stop motors
    for(i=0; i<n; i++) throttles[i] = 0;
    motorImplementationSendThrottles(motorPins, n, throttles);

    // finalize
    motorImplementationFinalize(motorPins, n);
    
    return(0);
}

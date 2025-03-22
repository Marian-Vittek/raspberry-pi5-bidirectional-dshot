
enum callbackTelemetryTypeEnum {
    BIDSHOT_TT_NONE,
    BIDSHOT_TT_PERIOD_US,		// to get rpm from this use formula (60000000.0 / (BIDSHOT_TT_PERIOD_US * poles / 2.0))
    BIDSHOT_TT_TEMPERATURE,
    BIDSHOT_TT_VOLTAGE,
    BIDSHOT_TT_CURRENT,
    BIDSHOT_TT_STRESS_LEVEL,
    BIDSHOT_TT_STATUS,
    BIDSHOT_TT_MAX,
};

// DSHOT version we generate, aka the bitrate.
// It can be one of 300, 600, 1200, ...
// This value may be modified from the program. However, any changes
// made after 'motorImplementationInitialize' do not take effect.
extern unsigned dshotVersion;

// This is a callback function which is executed when the data frame
// is received and parsed. It is supposed that you set it in your code.
extern void (*bidshotTelemetryCallback)(int telemetryType, int motorIndex, double *value);


// DSHOT special commands.
enum {
    DSHOT_CMD_MOTOR_STOP = 0,
    DSHOT_CMD_BEACON1,
    DSHOT_CMD_BEACON2,
    DSHOT_CMD_BEACON3,
    DSHOT_CMD_BEACON4,
    DSHOT_CMD_BEACON5,
    DSHOT_CMD_ESC_INFO, // V2 includes settings
    DSHOT_CMD_SPIN_DIRECTION_1,
    DSHOT_CMD_SPIN_DIRECTION_2,
    DSHOT_CMD_3D_MODE_OFF,
    DSHOT_CMD_3D_MODE_ON,
    DSHOT_CMD_SETTINGS_REQUEST,  	      // Currently not implemented
    DSHOT_CMD_SAVE_SETTINGS,  		      // Need 6x, wait at least 35ms before next command
    DSHOT_CMD_EXTENDED_TELEMETRY_ENABLE,       // Need 6x, wait at least 35ms before next command
    DSHOT_CMD_EXTENDED_TELEMETRY_DISABLE,      // Need 6x, wait at least 35ms before next command
    DSHOT_CMD_SPIN_DIRECTION_NORMAL = 20,
    DSHOT_CMD_SPIN_DIRECTION_REVERSED = 21,
    DSHOT_CMD_LED0_ON, // BLHeli32 only
    DSHOT_CMD_LED1_ON, // BLHeli32 only
    DSHOT_CMD_LED2_ON, // BLHeli32 only
    DSHOT_CMD_LED3_ON, // BLHeli32 only
    DSHOT_CMD_LED0_OFF, // BLHeli32 only
    DSHOT_CMD_LED1_OFF, // BLHeli32 only
    DSHOT_CMD_LED2_OFF, // BLHeli32 only
    DSHOT_CMD_LED3_OFF, // BLHeli32 only
    DSHOT_CMD_AUDIO_STREAM_MODE_ON_OFF = 30, // KISS audio Stream mode on/Off
    DSHOT_CMD_SILENT_MODE_ON_OFF = 31, // KISS silent Mode on/Off
    DSHOT_CMD_SIGNAL_LINE_TELEMETRY_DISABLE = 32,
    DSHOT_CMD_SIGNAL_LINE_CONTINUOUS_ERPM_TELEMETRY = 33,
    DSHOT_CMD_MAX = 47
};

// Exported functions
void dshotRepeatSendCommand(int motorPins[], int motorMax, int cmd, int telemetry, int repeatCounter) ;
void motorImplementationSet3dModeAndSpinDirection(int motorPins[], int motorMax, int mode3dFlag, int reverseDirectionFlag) ;
void motorImplementationInitialize(int motorPins[], int motorMax) ;
void motorImplementationFinalize(int motorPins[], int motorMax) ;
void motorImplementationSendThrottles(int motorPins[], int motorMax, double motorThrottle[]) ;

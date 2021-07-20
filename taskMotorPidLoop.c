// Defines
#define BLK_MOTOR_PID_P				0.051
#define BLK_MOTOR_PID_I				0.7
#define BLK_MOTOR_FILTER_SZ		5
#ifdef BLK_TEST
#define FAST_RAM_ZERO_INIT
#define FAST_CODE_NOINLINE
#define MAX_SUPPORTED_MOTORS	4
#define EMU_MOTOR_TIME_CST		0.05
#endif

// Globals
FAST_RAM_ZERO_INIT uint16_t		blkMotorSpeedRef[MAX_SUPPORTED_MOTORS];				// Regulation reference
FAST_RAM_ZERO_INIT uint16_t		blkMotorFilteredSpeed[MAX_SUPPORTED_MOTORS];	// Regulation measurement
FAST_RAM_ZERO_INIT uint8_t		blkPidInitFlag;																// PID needs reset
FAST_RAM_ZERO_INIT uint8_t		blkPidActiveFlag;															// PID is active only when Betalink MSP command received
#ifdef BLK_TEST
float 												motor_disarmed[MAX_SUPPORTED_MOTORS];
uint32_t 											targetPidLooptime = 125;
#endif

// Motor speed regulation
FAST_CODE_NOINLINE void taskMotorPidLoop( void )	{
	static uint16_t blkMotorFilterHistory[MAX_SUPPORTED_MOTORS][BLK_MOTOR_FILTER_SZ];
	static float 		blkMotorPidUi1[MAX_SUPPORTED_MOTORS];
	static float 		blkMotorPidE1[MAX_SUPPORTED_MOTORS];
	float 					blkMotorPidUi;
	float 					blkMotorPidUp;
	float 					blkMotorPidE;
	uint16_t 				blkMotorMedianFilterHistory[MAX_SUPPORTED_MOTORS][BLK_MOTOR_FILTER_SZ];
	
	// Skip if PID not active
	if (!blkPidActiveFlag)
		return;
		
	// Initialize PID if needed
	if ( !blkPidInitFlag )	{
		for (unsigned i = 0; i < MAX_SUPPORTED_MOTORS; i++) {
			blkMotorPidUi1[i] = 0.0;
			blkMotorPidE1[i] = 0.0;
			for (unsigned j = 0; j < BLK_MOTOR_FILTER_SZ; j++) {
				blkMotorFilterHistory[i][j] = 0.0;
			}
		}
		blkPidInitFlag = 1;
	}
	
	// Filtering motor speed
	for (unsigned i = 0; i < MAX_SUPPORTED_MOTORS; i++) {
	
		// Shifting history buffer one sample in the past
		for (unsigned ii = BLK_MOTOR_FILTER_SZ-1; ii > 0; ii--) {
			blkMotorFilterHistory[i][ii] = blkMotorFilterHistory[i][ii-1];
		}
		#ifdef BLK_TEST
		blkMotorFilterHistory[i][0] = getDshotTelemetry(i);
		#else
		blkMotorFilterHistory[i][0] = (uint16_t)lrint( (float)getDshotTelemetry(i) * 100.0 * 2.0 / motorConfig()->motorPoleCount );
		#endif
		
		// Median filtering: bubble sorting
		for (unsigned ii = 0; ii < BLK_MOTOR_FILTER_SZ; ii++)
			blkMotorMedianFilterHistory[i][ii] = blkMotorFilterHistory[i][ii];
		for (unsigned k = 0; k < BLK_MOTOR_FILTER_SZ - 1; k++) {
    	for (unsigned l = 0; l < BLK_MOTOR_FILTER_SZ - k - 1; l++) {
        if (blkMotorMedianFilterHistory[i][l] > blkMotorMedianFilterHistory[i][l + 1]) {
					unsigned temp = blkMotorMedianFilterHistory[i][l];
					blkMotorMedianFilterHistory[i][l] = blkMotorMedianFilterHistory[i][l + 1];
					blkMotorMedianFilterHistory[i][l + 1] = temp;
        }
    	}
		}
		blkMotorFilteredSpeed[i] = blkMotorMedianFilterHistory[i][(unsigned)((BLK_MOTOR_FILTER_SZ-1)/2)];
		
	}
	
	// Calculate PI control law
  // U(z)=P.E(z)+I.Ts/2(z+1)/(z-1)
	
	for (unsigned i = 0; i < getMotorCount(); i++) {
		// Calculate the error
		blkMotorPidE = blkMotorSpeedRef[i] - (float)blkMotorFilteredSpeed[i];
		
		// Calculate the control signal
		blkMotorPidUp = BLK_MOTOR_PID_P * blkMotorPidE;
		blkMotorPidUi = blkMotorPidUi1[i] + 
										BLK_MOTOR_PID_I * targetPidLooptime * 1e-6 / 2.0 * ( blkMotorPidE + blkMotorPidE1[i] );
		
		// Anti-windup
		if ( blkMotorPidUp + blkMotorPidUi > PWM_RANGE_MAX )	{
			blkMotorPidUi = PWM_RANGE_MAX - blkMotorPidUp;
		}
		if ( blkMotorPidUp + blkMotorPidUi < PWM_RANGE_MIN + 1 )	{
			blkMotorPidUi = PWM_RANGE_MIN + 1 - blkMotorPidUp;
		}
		
		// Send control signal
		motor_disarmed[i] = motorConvertFromExternal(blkMotorPidUp + blkMotorPidUi);
		
		// Update history
		blkMotorPidUi1[i] = blkMotorPidUi;
		blkMotorPidE1[i] = blkMotorPidE;
	}
}

// Unitary tests
// gcc -o test -Wall -D BLK_TEST -g taskMotorPidLoop.c
#ifdef BLK_TEST
float motorConvertFromExternal(uint16_t externalValue)
{
  return (float)externalValue;
}

uint16_t getDshotTelemetry(uint8_t index)
{
  return dshotTelemetryState.motorState[index].telemetryValue;
}
#endif































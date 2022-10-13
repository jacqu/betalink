// Defines
#define BLK_MOTOR_PID_P				0.075								// P gain
#define BLK_MOTOR_PID_I				1.5									// I gain
#define BLK_MOTOR_FILTER_SZ		8										// Median filter dimension
#define BLK_MOTOR_FILTER_EX		0										// Averaging filtering exclusion band
//#define BLK_FILTER_ACTIVE													// Flag to activate filtering
#define BLK_FILTER_LP															// Flag to activate lowpass filtering
//#define BLK_FILTER_MED														// Flag to activate median filtering

#ifdef BLK_TEST
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <time.h>
#include <math.h>
#define FAST_DATA_ZERO_INIT
#define FAST_CODE_NOINLINE
#define MAX_SUPPORTED_MOTORS	1
#define PWM_RANGE_MIN 				1000
#define PWM_RANGE_MAX 				2000
#define BLK_EMU_MOTOR_VEL			3000.0
#define BLK_EMU_MOTOR_REF			3500
#define BLK_NB_ITER_TEST			8000
#define BLK_NOISE_SPIKE				500.0
#define BLK_SPIKE_PROBA				0.05
#endif

// Externs
extern FAST_DATA_ZERO_INIT float  filteredMotorErpm[MAX_SUPPORTED_MOTORS];

// Globals
FAST_DATA_ZERO_INIT uint16_t			blkMotorSpeedRef[MAX_SUPPORTED_MOTORS];				// Regulation reference
FAST_DATA_ZERO_INIT uint16_t			blkMotorFilteredSpeed[MAX_SUPPORTED_MOTORS];	// Regulation measurement
FAST_DATA_ZERO_INIT uint8_t				blkPidInitFlag;																// PID needs reset
FAST_DATA_ZERO_INIT uint8_t				blkPidActiveFlag;															// PID is active only when Betalink MSP command received
#ifdef BLK_TEST
float 														motor_disarmed[MAX_SUPPORTED_MOTORS];
uint32_t 													targetPidLooptime = 125;

float motorConvertFromExternal(uint16_t externalValue)
{
  return (float)externalValue;
}

uint16_t getDshotTelemetry(uint8_t index)
{
	if ( rand() < BLK_SPIKE_PROBA * (float)RAND_MAX )
		return (uint16_t)lrint(BLK_EMU_MOTOR_VEL + BLK_NOISE_SPIKE * (float)rand() / RAND_MAX);
	else
		return (uint16_t)lrint(BLK_EMU_MOTOR_VEL);
}

uint8_t getMotorCount(void)
{
    return MAX_SUPPORTED_MOTORS;
}
#endif

// Motor speed regulation
FAST_CODE_NOINLINE void taskMotorPidLoop( void )	{
	static float 		blkMotorPidUi1[MAX_SUPPORTED_MOTORS];
	static float 		blkMotorPidE1[MAX_SUPPORTED_MOTORS];
	float 					blkMotorPidUi;
	float 					blkMotorPidUp;
	float 					blkMotorPidE;
	#ifdef BLK_FILTER_ACTIVE
	float 					blkAvg;
	static uint16_t blkMotorFilterHistory[MAX_SUPPORTED_MOTORS][BLK_MOTOR_FILTER_SZ];
	uint16_t 				blkMotorMedianFilterHistory[MAX_SUPPORTED_MOTORS][BLK_MOTOR_FILTER_SZ];
	#endif
	
	// Skip if PID not active
	if (!blkPidActiveFlag)
		return;
		
	// Initialize PID if needed
	if ( !blkPidInitFlag )	{
		for (unsigned i = 0; i < MAX_SUPPORTED_MOTORS; i++) {
			blkMotorPidUi1[i] = 0.0;
			blkMotorPidE1[i] = 0.0;
			#ifdef BLK_FILTER_ACTIVE
			for (unsigned j = 0; j < BLK_MOTOR_FILTER_SZ; j++) {
				blkMotorFilterHistory[i][j] = 0.0;
			}
			#endif
		}
		blkPidInitFlag = 1;
	}
	
	// Iterating through all motors
	for (unsigned i = 0; i < MAX_SUPPORTED_MOTORS; i++) {
	
		// Filtering motor speed
		#ifdef BLK_FILTER_ACTIVE
		
		// Shifting history buffer one sample in the past
		for (unsigned ii = BLK_MOTOR_FILTER_SZ-1; ii > 0; ii--) {
			blkMotorFilterHistory[i][ii] = blkMotorFilterHistory[i][ii-1];
		}
		#ifdef BLK_TEST
		blkMotorFilterHistory[i][0] = getDshotTelemetry(i);
		#else
		#ifdef BLK_FILTER_LP
		blkMotorFilterHistory[i][0] = (uint16_t)lrint( (float)filteredMotorErpm[i] * 100.0 * 2.0 / motorConfig()->motorPoleCount );
		#else
		blkMotorFilterHistory[i][0] = (uint16_t)lrint( (float)getDshotTelemetry(i) * 100.0 * 2.0 / motorConfig()->motorPoleCount );
		#endif
		#endif
		
		for (unsigned ii = 0; ii < BLK_MOTOR_FILTER_SZ; ii++)	{
			blkMotorMedianFilterHistory[i][ii] = blkMotorFilterHistory[i][ii];
		}
		
		// Median filtering: bubble sorting
		#ifdef BLK_FILTER_MED
		for (unsigned k = 0; k < BLK_MOTOR_FILTER_SZ - 1; k++) {
    	for (unsigned l = 0; l < BLK_MOTOR_FILTER_SZ - k - 1; l++) {
        if (blkMotorMedianFilterHistory[i][l] > blkMotorMedianFilterHistory[i][l + 1]) {
					unsigned temp = blkMotorMedianFilterHistory[i][l];
					blkMotorMedianFilterHistory[i][l] = blkMotorMedianFilterHistory[i][l + 1];
					blkMotorMedianFilterHistory[i][l + 1] = temp;
        }
    	}
		}
		#endif
		
		// Averaging the median samples. If median deactivated -> averaging filter
		blkAvg = 0.0;
		for (unsigned ii = BLK_MOTOR_FILTER_EX; ii < BLK_MOTOR_FILTER_SZ-BLK_MOTOR_FILTER_EX; ii++)	{
			blkAvg += blkMotorMedianFilterHistory[i][ii];	
		}
		blkMotorFilteredSpeed[i] = (uint16_t)lrint( blkAvg / (BLK_MOTOR_FILTER_SZ-2*BLK_MOTOR_FILTER_EX) );
		#else
		// Raw data, no filtering
		blkMotorFilteredSpeed[i] = (uint16_t)lrint( (float)getDshotTelemetry(i) * 100.0 * 2.0 / motorConfig()->motorPoleCount );
		#endif
	}
	
	// Calculate PI control law
  // U(z)=P.E(z)+I.Ts/2(z+1)/(z-1)
	
	for (unsigned i = 0; i < getMotorCount(); i++) {
		
		// If reference below or equal to PWM_RANGE_MIN: force to PWM_RANGE_MIN (motor stop)
		if ( blkMotorSpeedRef[i] <= PWM_RANGE_MIN )	{
			motor_disarmed[i] = motorConvertFromExternal(PWM_RANGE_MIN);
			continue;
		}
			
		// If reference in the range PWM_RANGE_MIN+1 -> PWM_RANGE_MAX: throttle = reference
		if ( ( blkMotorSpeedRef[i] > PWM_RANGE_MIN ) && ( blkMotorSpeedRef[i] <= PWM_RANGE_MAX ) )	{
			motor_disarmed[i] = motorConvertFromExternal(blkMotorSpeedRef[i]);
			continue;
		}
		
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
int main( void )	{
	
	srand( time(NULL) );
	
	blkPidActiveFlag = 1;
	
	for (unsigned i = 0; i < MAX_SUPPORTED_MOTORS; i++)
		blkMotorSpeedRef[i] = BLK_EMU_MOTOR_REF;
	
	for (unsigned i = 0; i < BLK_NB_ITER_TEST; i++)	{
		
		taskMotorPidLoop(  );
		
		for (unsigned ii = 0; ii < MAX_SUPPORTED_MOTORS; ii++)	{
			printf( "%d\t%d\t%f\t", blkMotorSpeedRef[ii], blkMotorFilteredSpeed[ii], motor_disarmed[ii] );
		}
		printf( "\n" );
	}
	
	return 0;
}
#endif
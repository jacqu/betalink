/*
 *  Definitions for betalink.c
 */
 
#ifndef __BLK_H
#define __BLK_H

// Defines
#define BLK_MAX_DEVICES					5 				// Max number of FC boards
#define BLK_MAX_MOTORS					8					// Max nb of motors per FC
#define BLK_DSHOT_MIN_THROTTLE  48				// Min throttle
#define BLK_DSHOT_MAX_THROTTLE  2047			// Max throttle
#define BLK_DSHOT_MOTOR_OFF			0					// Motor stop command
#define BLK_MSP_ACC_SCALING			(1/51.2)	// Accelerometer scaling factor
#define BLK_MSP_GYR_SCALING			(4/16.4)	// Gyrometer scaling factor
#define BLK_MSP_MAG_SCALING			(1/1090.0)// Magnetometer scaling factor
#define BLK_MSP_BATV_SCALING		(1/100.0)	// Battery voltage scaling factor
#define BLK_MSP_BATA_SCALING		(1/100.0)	// Battery current scaling factor
#define BLK_MSP_INV_SCALING			(1/100.0)	// Invalide DSHOT percent scaling factor
#define BLK_MSP_MOTV_SCALING		(1/100.0)	// Motor voltage scaling factor
#define BLK_MSP_MOTA_SCALING		(1/100.0)	// Motor current scaling factor
#define BLK_MSP_ANGLE_SCALING		(1/10.0)	// Euler angles scaling factor
#define BLK_PWM_RANGE_MIN 			1000
#define BLK_PWM_RANGE_MAX 			2000

#define BLK_ERROR_FD        		-1        // Non existant file descriptor
#define BLK_ERROR_DEV       		-2        // Non existant serial device
#define BLK_ERROR_MAX_DEV   		-3        // Maximum devices limit 
#define BLK_ERROR_WRITE_SER 		-4        // Write error on serial
#define BLK_ERROR_BAD_PK_SZ 		-5        // Bad incoming packet size error
#define BLK_ERROR_INIT_STATE		-6				// unable to initlize FC state
#define BLK_MSP_ERROR_INT				-7				// MSP internal error
#define BLK_MSP_ERROR_DEC				-8				// MSP decoder error
#define BLK_MSP_ERROR_WRITE			-9				// MSP write error
#define BLK_MSP_ERROR_READ			-10				// MSP read error
#define BLK_ERROR_INT						-255			// Betalink internal error

//
//	betalink API
//

typedef struct blk_state_s {
	uint8_t					motor_count;
	uint8_t					betalink_detected;
	pthread_mutex_t read_mutex;
	pthread_mutex_t update_mutex;
	uint32_t        timestamp;
	uint16_t				throttle[BLK_MAX_MOTORS];
	uint32_t				rpm[BLK_MAX_MOTORS];
	uint16_t				inv[BLK_MAX_MOTORS];
	uint8_t					temp[BLK_MAX_MOTORS];
	uint16_t				volt[BLK_MAX_MOTORS];
	uint16_t				amp[BLK_MAX_MOTORS];
	uint16_t				mah[BLK_MAX_MOTORS];
	uint8_t					bat_cell_cnt;
	uint16_t				bat_capa;
	uint16_t				bat_volt;
	int16_t					bat_amp;
	uint16_t				bat_mah;
	int16_t					acc[3];
	int16_t					gyr[3];
	int16_t					mag[3];
	int16_t         roll;
	int16_t         pitch;
	int16_t         yaw;
} blk_state_t;

uint32_t  blk_crc32b( char *message );
char      *blk_name_from_serial( uint32_t );
int       blk_get_fd( uint32_t );
int       blk_init_port( uint32_t );
void      blk_release_port( uint32_t );
int 	    blk_write( uint32_t serial_nb, uint8_t* buf, uint16_t size );
int 	    blk_read( uint32_t serial_nb, uint8_t* buf, uint16_t max_size );

//
// 	MSP API
//
int msp_encode( int fd_idx, uint16_t code, uint8_t *data, uint16_t data_length );
int msp_decode( int fd_idx, uint8_t* data, uint16_t size );
int blk_get_imu( uint32_t serial_nb );
int blk_enable_motor( uint32_t serial_nb, uint8_t flag );
int blk_get_motor_telemetry( uint32_t serial_nb );
int blk_get_motor_throttle( uint32_t serial_nb );
int blk_set_motor( 	uint32_t serial_nb, uint16_t *throttle );
int blk_get_battery_state( uint32_t serial_nb );
int blk_copy_state( uint32_t serial_nb, blk_state_t *state );
void blk_dump_fc_state( uint32_t serial_nb );
int blk_detect( uint32_t serial_nb );
int blk_update( 	uint32_t serial_nb, uint16_t *throttle );
int blk_update_threaded( 	uint32_t serial_nb, uint16_t *throttle );
uint8_t msp_crc8_dvb_s2_data( uint8_t *data, uint8_t start, uint8_t end );
uint8_t msp_crc8_dvb_s2( uint8_t crc, uint8_t ch );

//
// streambuf API 
// Simple buffer-based serializer/deserializer
//

typedef struct sbuf_s {
	uint8_t *ptr;          // data pointer must be first (sbuf_t* is equivalent to uint8_t **)
	uint8_t *begin;
	uint8_t *end;
} sbuf_t;

sbuf_t *sbufInit(sbuf_t *sbuf, uint8_t *ptr, uint8_t *end);
uint32_t sbufIndex(sbuf_t *sbuf);
void sbufWriteU8(sbuf_t *dst, uint8_t val);
void sbufWriteU16(sbuf_t *dst, uint16_t val);
void sbufWriteU32(sbuf_t *dst, uint32_t val);
void sbufWriteU16BigEndian(sbuf_t *dst, uint16_t val);
void sbufWriteU32BigEndian(sbuf_t *dst, uint32_t val);
void sbufFill(sbuf_t *dst, uint8_t data, int len);
void sbufWriteData(sbuf_t *dst, const void *data, int len);
void sbufWriteString(sbuf_t *dst, const char *string);
void sbufWriteStringWithZeroTerminator(sbuf_t *dst, const char *string);

uint8_t sbufReadU8(sbuf_t *src);
uint16_t sbufReadU16(sbuf_t *src);
uint32_t sbufReadU32(sbuf_t *src);
void sbufReadData(sbuf_t *dst, void *data, int len);

int sbufBytesRemaining(sbuf_t *buf);
uint8_t* sbufPtr(sbuf_t *buf);
const uint8_t* sbufConstPtr(const sbuf_t *buf);
void sbufAdvance(sbuf_t *buf, int size);

void sbufSwitchToReader(sbuf_t *buf, uint8_t * base);
#endif
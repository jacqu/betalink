/*
 *	Communication with a betaflight FC using multiwii protocol
 *  JG, June 2021
 *  To compile : gcc -Wall -pthread -o betalink  betalink.c
 */

#include <fcntl.h>
#include <termios.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <unistd.h>
#include <time.h>
#include <string.h>
#include <strings.h>
#include <limits.h>
#include <dirent.h>
#include <sys/ioctl.h>
#include <sys/param.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <pthread.h>
#if defined(__linux__)
#include <linux/serial.h>
#include <linux/input.h>
#endif
#include "betalink.h"
#include "msp_codes.h"

// Flags
#define BLK_STANDALONE 										// main is added for testing
#define BLK_THREADED_UPDATE								// activate threaded update
//#define BLK_TWO_FC                        // Test with rwo FCs
//#define BLK_DEBUG													// Display additional debug infos

// Defines

#define BLK_DEV_SERIALNB  643986269		    // Serial number of the test FC
#define BLK_DEV2_SERIALNB 707372631       // Serial number of the second test FC
#define BLK_DEV_SERIALLG	16							// Max length of a serial number
#define BLK_BAUDRATE      B115200        	// Serial baudrate
#define BLK_READ_TIMEOUT  5              	// Tenth of second
#define BLK_NB_PING       100           	// Nb roundtrip communication
#define BLK_STEP_REF      200            	// Velocity reference step size (10 rpm motor)
#define BLK_PERIOD        3000          	// Period of serial exchange (us)
#define BLK_STEP_PERIOD   200            	// Duration of a step (itertions)
#define BLK_STEP_THROTTLE	1050						// Step amplitude in throttle scale

#if defined(__linux__)
#define BLK_SERIAL_DEV_DIR "/dev/serial/by-id/"
#elif defined(__APPLE__)
#define BLK_SERIAL_DEV_DIR "/dev/"
#else
#define BLK_SERIAL_DEV_DIR "/dev/"
#endif

#define MSP_PROTOCOL_MAX_BUF_SZ									5000

#define MSP_SYM_BEGIN													 	'$'
#define MSP_SYM_PROTO_V1												'M'
#define MSP_SYM_PROTO_V2 												'X'
#define MSP_SYM_FROM_MWC 												'>'
#define MSP_SYM_TO_MWC 													'<'
#define MSP_SYM_UNSUPPORTED 										'!'

#define MSP_PROTOCOL_V1                					1
#define MSP_PROTOCOL_V2                					2
#define MSP_JUMBO_FRAME_MIN_SIZE       					255

#define MSP_IDLE                       					0
#define MSP_PROTO_IDENTIFIER           					1
#define MSP_DIRECTION_V1               					2
#define MSP_DIRECTION_V2               					3
#define MSP_FLAG_V2                    					4
#define MSP_PAYLOAD_LENGTH_V1          					5
#define MSP_PAYLOAD_LENGTH_JUMBO_LOW   					6
#define MSP_PAYLOAD_LENGTH_JUMBO_HIGH  					7
#define MSP_PAYLOAD_LENGTH_V2_LOW      					8
#define MSP_PAYLOAD_LENGTH_V2_HIGH     					9
#define MSP_CODE_V1                    					10
#define MSP_CODE_JUMBO_V1              					11
#define MSP_CODE_V2_LOW                					12
#define MSP_CODE_V2_HIGH               					13
#define MSP_PAYLOAD_V1                 					14
#define MSP_PAYLOAD_V2                 					15
#define MSP_CHECKSUM_V1                					16
#define MSP_CHECKSUM_V2                					17
#define MSP_JUMBO_FRAME_SIZE_LIMIT 							255

#define MSP_V1_LENGTH_OFFSET										3
#define MSP_V2_LENGTH_OFFSET										6
#define MSP_V1_OVERHEAD													6
#define MSP_V2_OVERHEAD													9

// Typedefs
typedef struct blk_msp_s {
	uint8_t 	msp_state;
	uint8_t 	msp_message_direction;
	uint8_t 	msp_code;
	uint16_t 	msp_message_length_expected;
	uint16_t 	msp_message_length_received;
	uint8_t 	msp_message_checksum;
	uint8_t 	msp_messageIsJumboFrame;
	uint8_t 	msp_crcError;
	uint8_t 	msp_packet_error;
	uint8_t 	msp_unsupported;
	uint8_t 	msp_buf_out[MSP_PROTOCOL_MAX_BUF_SZ];
	uint8_t 	msp_buf_in[MSP_PROTOCOL_MAX_BUF_SZ];
} blk_msp_t;

// Globals
int                 blk_fd[BLK_MAX_DEVICES] = 
                          { BLK_ERROR_FD, 
                            BLK_ERROR_FD,
                            BLK_ERROR_FD,
                            BLK_ERROR_FD,
                            BLK_ERROR_FD };        				// Serial port file descriptor		
                            
uint32_t            blk_devname[BLK_MAX_DEVICES] = { 0 }; // Serial port crc32 number used to get fd with open
                            
struct termios      blk_oldtio[BLK_MAX_DEVICES];  				// Backup of initial tty configuration

blk_state_t					blk_state[BLK_MAX_DEVICES] = 	{ 0 };		// Main betalink state structure
blk_msp_t						blk_msp[BLK_MAX_DEVICES] = 		{ 0 };		// MSP state

//
// betalink API
//

//
// crc32 helper function
//
uint32_t blk_crc32b( char *message ) {
   int      i, j;
   uint32_t byte, crc, mask;

   i = 0;
   crc = 0xFFFFFFFF;
   while (message[i] != 0) {
      byte = message[i];            // Get next byte.
      crc = crc ^ byte;
      for (j = 7; j >= 0; j--) {    // Do eight times.
         mask = -(crc & 1);
         crc = (crc >> 1) ^ (0xEDB88320 & mask);
      }
      i = i + 1;
   }
   return ~crc;
}

//
//  Get the device name from the device serial number
//
char *blk_name_from_serial( uint32_t serial_nb ) {
  DIR           *d;
  struct dirent *dir;
  static char   portname[PATH_MAX];
  uint32_t      crc;
  
  // Open directory where serial devices can be found
  d = opendir( BLK_SERIAL_DEV_DIR );
  
  // Look for a device name contining teensy serial number
  if ( d ) {
    printf( "BLK: scanning of connected UART devices...\n" );
    // Scan each file in the directory
    while ( ( dir = readdir( d ) ) != NULL ) {
      if ( ( !strcmp( dir->d_name, "." ) ) || ( !strcmp( dir->d_name, ".." ) ) )
        continue;
      crc = blk_crc32b( dir->d_name );
      printf( "  %s: serial number=%u\n", dir->d_name, crc );
      if ( crc == serial_nb ) {
      
        // A match is a device name containing the serial number
        snprintf( portname, PATH_MAX, "%s%s", BLK_SERIAL_DEV_DIR, dir->d_name );
        printf( "BLK: portname found=%s\n", portname );
        closedir( d );
        return portname;
      }
    }
    closedir( d );
  }
  
  return NULL;
}

//
//  Get the file descriptor index which device name contains
//  specified serial number. 
//  Returns -1 if no matching fd is found.
//
int blk_get_fd( uint32_t serial_nb ) {
  int   i;
     
  for ( i = 0; i < BLK_MAX_DEVICES; i++ )
    if ( blk_fd[i] != BLK_ERROR_FD )
        if ( blk_devname[i] == serial_nb )
          return i;

  return BLK_ERROR_FD;
}

//
//  Initialize serial port
//
int blk_init_port( uint32_t serial_nb )  {
  struct  termios newtio;
  int     check_fd;
  int     fd_idx;
  char    *portname;
  int			ret;
  
  // Check if device plugged in
  portname = blk_name_from_serial( serial_nb );
  if ( !portname )
    return BLK_ERROR_DEV;

  // Open device
  check_fd = open( portname, O_RDWR | O_NOCTTY | O_NONBLOCK );
  
  if ( check_fd < 0 )  {
    perror( portname );
    return BLK_ERROR_DEV;
  }
  
  // Look for an empty slot to store the fd
  for ( fd_idx = 0; fd_idx < BLK_MAX_DEVICES; fd_idx++ )
    if ( blk_fd[fd_idx] == BLK_ERROR_FD )
      break;
      
  // Close fd and throw an error if all slots are used
  if ( fd_idx == BLK_MAX_DEVICES ) {
    close( check_fd );
    return BLK_ERROR_MAX_DEV;
  }
  
  // Store the fd and the corresponding devname
  blk_fd[fd_idx] = check_fd;
  blk_devname[fd_idx] = serial_nb;

  /* Save current port settings */
  tcgetattr( check_fd, &blk_oldtio[fd_idx] );

  /* Define new settings */
  bzero( &newtio, sizeof(newtio) );
  cfmakeraw( &newtio );

  newtio.c_cflag =      BLK_BAUDRATE | CS8 | CLOCAL | CREAD;
  newtio.c_iflag =      IGNPAR;
  newtio.c_oflag =      0;
  newtio.c_lflag =      0;
  newtio.c_cc[VTIME] =  0;
  newtio.c_cc[VMIN] =   0;

  #if defined(__APPLE__)
  cfsetispeed( &newtio, BLK_BAUDRATE );
  cfsetospeed( &newtio, BLK_BAUDRATE );
  #endif

  /* Apply the settings */
  tcflush( check_fd, TCIFLUSH );
  tcsetattr( check_fd, TCSANOW, &newtio );
  
  // Initialize main data structure
  if ( 	blk_get_battery_state( serial_nb ) ||
  			blk_get_motor_throttle( serial_nb ) ||
  			blk_get_motor_telemetry( serial_nb ) ||
  			blk_get_imu( serial_nb ) ) {
  	return BLK_ERROR_INIT_STATE;
  }
  
  // Initialize mutex
  pthread_mutex_init( &blk_state[fd_idx].read_mutex, NULL );
  pthread_mutex_init( &blk_state[fd_idx].update_mutex, NULL );
  
  // Check if betalink firmware is available
  ret = blk_detect( serial_nb );
  if ( ret )	{
	  fprintf( stderr, "BLK: error %d in blk_detect\n", ret );
  }

  return 0;
}

//
//  Release serial port
//
void blk_release_port( uint32_t serial_nb )  {
  int   fd_idx;
  
  // Get fd index from serial number
  fd_idx = blk_get_fd( serial_nb );
  
  if ( fd_idx != BLK_ERROR_FD ) {
    
    // Wait for pending threads to finish before proceeding
    pthread_mutex_lock( &blk_state[fd_idx].read_mutex );
    
    // Restore initial settings if needed
    tcsetattr( blk_fd[fd_idx], TCSANOW, &blk_oldtio[fd_idx] );
    close( blk_fd[fd_idx] );
    
    // Clear fd and corresponding devname
    blk_fd[fd_idx] = BLK_ERROR_FD;
    blk_devname[fd_idx] = 0;
    
    // Destroy update mutex
    if ( pthread_mutex_destroy( &blk_state[fd_idx].update_mutex ) )	{
	    perror( "BLK: pthread_mutex_destroy update_mutex" );
    }
    
    pthread_mutex_unlock( &blk_state[fd_idx].read_mutex );
    
    // Destroy read mutex
    if ( pthread_mutex_destroy( &blk_state[fd_idx].read_mutex ) )	{
	    perror( "BLK: pthread_mutex_destroy read_mutex" );
    }
  }
}

//
// serial write
//
int blk_write( uint32_t serial_nb, uint8_t* buf, uint16_t size ) {
	int res = 0, fd_idx;
	
	// Get fd index
  fd_idx = blk_get_fd( serial_nb );
  
  // Check if fd index is valid
  if ( fd_idx == BLK_ERROR_FD )
    return BLK_ERROR_FD;
  
  // Send output buffer data
  res = write( blk_fd[fd_idx], buf, size );
  if ( res < 0 )  {
    perror( "BLK: write blk_comm" );
    return BLK_ERROR_WRITE_SER;
  }
  
  // Flush output buffer
  fsync( blk_fd[fd_idx] );
  
  return res;
}

//
// serial read
//
int blk_read( uint32_t serial_nb, uint8_t* buf, uint16_t max_size ) {
	int                 ret, res = 0, fd_idx;
  uint8_t             data_in;
  uint16_t						size = max_size;
  struct timespec     start, cur;
  unsigned long long  elapsed_us;
  
  // Get fd index
  fd_idx = blk_get_fd( serial_nb );
  
  // Check if fd index is valid
  if ( fd_idx == BLK_ERROR_FD )
    return BLK_ERROR_FD;
  
  // Wait for response

  // Get current time
  clock_gettime( CLOCK_MONOTONIC, &start );

  // Reset byte counter
  res = 0;
	
	#ifdef BLK_DEBUG
	fprintf( stderr, "FC->BLK:\n" );
	#endif
	
  do  {
    ret = read( blk_fd[fd_idx], &buf[res], 1 );

    // Data received
    if ( ret > 0 )  {
      #ifdef BLK_DEBUG
			fprintf( stderr, "%d:%d\t\t[%c]\n", res, buf[res], buf[res] );
			#endif
			res += ret;
    }

    // Read error
    if ( ret < 0 )
      break;
		
		// Read expected size
		if ( size == max_size )	{
			if ( res > 2 )	{
				if ( ( buf[0] ==  MSP_SYM_BEGIN ) && ( buf[1] ==  MSP_SYM_PROTO_V1 ) )	{
					if ( res > MSP_V1_LENGTH_OFFSET + 1 ) {
						size = buf[MSP_V1_LENGTH_OFFSET] + MSP_V1_OVERHEAD;
						#ifdef BLK_DEBUG
						fprintf( stderr, "BLK: v1 packet size=%d\n", size );
						#endif
						}
				}
				if ( ( buf[0] ==  MSP_SYM_BEGIN ) && ( buf[1] ==  MSP_SYM_PROTO_V2 ) )	{
					if ( res > MSP_V2_LENGTH_OFFSET + 2 ) {
						size = buf[MSP_V2_LENGTH_OFFSET] + ( buf[MSP_V2_LENGTH_OFFSET+1] << 8 ) + MSP_V2_OVERHEAD;
						#ifdef BLK_DEBUG
						fprintf( stderr, "BLK: v2 packet size=%d\n", size );
						#endif
					}
				}
			}
			if ( size > max_size )	{
				fprintf( stderr, "BLK: Packet too large for buffer size.\n" );
				size = max_size;
				break;
			}
		}
		
    // Compute time elapsed
    clock_gettime( CLOCK_MONOTONIC, &cur );
    elapsed_us =  ( cur.tv_sec * 1e6 + cur.tv_nsec / 1e3 ) -
                  ( start.tv_sec * 1e6 + start.tv_nsec / 1e3 );

    // Timeout
    if ( elapsed_us / 100000 > BLK_READ_TIMEOUT )
      break;

  } while ( res < size );
  
  #ifdef BLK_DEBUG
  fprintf( stderr, "BLK: read duration = %d us.\n", elapsed_us );
  #endif

  // Check response size
  if ( res != size )  {
    fprintf( stderr, "BLK: Packet with unexpected size received.\n" );

    // Flush input buffer
    while ( ( ret = read( blk_fd[fd_idx], &data_in, 1 ) ) )
      if ( ret <= 0 )
        break;
        
    return BLK_ERROR_BAD_PK_SZ;
  }
  
  return res;
}

//
// Get motor telemetry data
//
int blk_get_motor_telemetry( 	uint32_t serial_nb ) {
	uint8_t buf[MSP_PROTOCOL_MAX_BUF_SZ];
  int 		ret, ret2;
  sbuf_t	sbuf;
  int			fd_idx;
  int			i;
  
  // Get fd index
  fd_idx = blk_get_fd( serial_nb );
  
  // Check if fd index is valid
  if ( fd_idx == BLK_ERROR_FD )
    return BLK_ERROR_FD;
  
  ret = msp_encode( fd_idx, MSP_MOTOR_TELEMETRY, NULL, 0 );
  if ( ret > 0 )	{
		ret2 = blk_write( serial_nb, blk_msp[fd_idx].msp_buf_out, ret );
	  if ( ret2 == ret  )	{
	    pthread_mutex_lock( &blk_state[fd_idx].read_mutex );
	  	ret = blk_read( serial_nb, buf, MSP_PROTOCOL_MAX_BUF_SZ );
	  	pthread_mutex_unlock( &blk_state[fd_idx].read_mutex );
		  if ( ret > 0 )	{
		  	ret = msp_decode( fd_idx, buf, ret );
			  if ( ret > 0 )	{
			  
				  // Packet is sccessfully decoded
				  sbufInit( &sbuf, blk_msp[fd_idx].msp_buf_in, (uint8_t*)(blk_msp[fd_idx].msp_buf_in+ret) );
				  
				  pthread_mutex_lock( &blk_state[fd_idx].update_mutex );
				  // Read motor count
				  blk_state[fd_idx].motor_count = sbufReadU8( &sbuf );
				  if ( blk_state[fd_idx].motor_count > BLK_MAX_MOTORS )
				  	blk_state[fd_idx].motor_count = BLK_MAX_MOTORS;
				  
				  // Read motor data
				  for ( i = 0; i < blk_state[fd_idx].motor_count; i++ )	{
					  blk_state[fd_idx].rpm[i] = sbufReadU32( &sbuf );
					  blk_state[fd_idx].inv[i] = sbufReadU16( &sbuf );
					  blk_state[fd_idx].temp[i] = sbufReadU8( &sbuf );
					  blk_state[fd_idx].volt[i] = sbufReadU16( &sbuf );
					  blk_state[fd_idx].amp[i] = sbufReadU16( &sbuf );
					  blk_state[fd_idx].mah[i] = sbufReadU16( &sbuf );
				  }
				  pthread_mutex_unlock( &blk_state[fd_idx].update_mutex );
			  }
				else {
					return BLK_MSP_ERROR_DEC;
				}
		  }
		  else {
			  return BLK_MSP_ERROR_READ;
		  }
	  }
	  else {
		  return BLK_MSP_ERROR_WRITE;
	  }
  }
  else {
	  return ret;
  }

  return 0;
}

//
// Get motor current throttle
//
int blk_get_motor_throttle( 	uint32_t serial_nb ) {
	uint8_t buf[MSP_PROTOCOL_MAX_BUF_SZ];
  int 		ret, ret2;
  sbuf_t	sbuf;
  int			fd_idx;
  int			i;
  
  // Get fd index
  fd_idx = blk_get_fd( serial_nb );
  
  // Check if fd index is valid
  if ( fd_idx == BLK_ERROR_FD )
    return BLK_ERROR_FD;
  
  ret = msp_encode( fd_idx, MSP_MOTOR, NULL, 0 );
  if ( ret > 0 )	{
  	ret2 = blk_write( serial_nb, blk_msp[fd_idx].msp_buf_out, ret );
	  if ( ret2 == ret  )	{
	    pthread_mutex_lock( &blk_state[fd_idx].read_mutex );
	  	ret = blk_read( serial_nb, buf, MSP_PROTOCOL_MAX_BUF_SZ );
	  	pthread_mutex_unlock( &blk_state[fd_idx].read_mutex );
		  if ( ret > 0 )	{
		  	ret = msp_decode( fd_idx, buf, ret );
			  if ( ret > 0 )	{
			  
				  // Packet is sccessfully decoded
				  sbufInit( &sbuf, blk_msp[fd_idx].msp_buf_in, (uint8_t*)(blk_msp[fd_idx].msp_buf_in+ret) );
				  
				  // Read motor throttle
				  pthread_mutex_lock( &blk_state[fd_idx].update_mutex );
				  for ( i = 0; i < blk_state[fd_idx].motor_count; i++ )	{
					  blk_state[fd_idx].throttle[i] = sbufReadU16( &sbuf );
				  }
				  pthread_mutex_unlock( &blk_state[fd_idx].update_mutex );
			  }
				else {
					return BLK_MSP_ERROR_DEC;
				}
		  }
		  else {
			  return BLK_MSP_ERROR_READ;
		  }
	  }
	  else {
		  return BLK_MSP_ERROR_WRITE;
	  }
  }
  else {
	  return ret;
  }

  return 0;
}

//
// Get battery state
//
int blk_get_battery_state( 	uint32_t serial_nb ) {
	uint8_t buf[MSP_PROTOCOL_MAX_BUF_SZ];
  int 		ret, ret2;
  sbuf_t	sbuf;
  int			fd_idx;
  
  // Get fd index
  fd_idx = blk_get_fd( serial_nb );
  
  // Check if fd index is valid
  if ( fd_idx == BLK_ERROR_FD )
    return BLK_ERROR_FD;
  
  ret = msp_encode( fd_idx, MSP_BATTERY_STATE, NULL, 0 );
  if ( ret > 0 )	{
  	ret2 = blk_write( serial_nb, blk_msp[fd_idx].msp_buf_out, ret );
	  if ( ret2 == ret  )	{
	    pthread_mutex_lock( &blk_state[fd_idx].read_mutex );
	  	ret = blk_read( serial_nb, buf, MSP_PROTOCOL_MAX_BUF_SZ );
	  	pthread_mutex_unlock( &blk_state[fd_idx].read_mutex );
		  if ( ret > 0 )	{
		  	ret = msp_decode( fd_idx, buf, ret );
			  if ( ret > 0 )	{
			  
				  // Packet is sccessfully decoded
				  sbufInit( &sbuf, blk_msp[fd_idx].msp_buf_in, (uint8_t*)(blk_msp[fd_idx].msp_buf_in+ret) );
				  
				  pthread_mutex_lock( &blk_state[fd_idx].update_mutex );
				  // Read cell count
					blk_state[fd_idx].bat_cell_cnt = sbufReadU8( &sbuf );
					
					// Read capacity
					blk_state[fd_idx].bat_capa = sbufReadU16( &sbuf );
					
					// Read legacy voltage
					sbufReadU8( &sbuf );
					
					// Read mAh drawn
					blk_state[fd_idx].bat_mah = sbufReadU16( &sbuf );
					
					// Read current
					blk_state[fd_idx].bat_amp = sbufReadU16( &sbuf );
					
					// Read alerts
					sbufReadU8( &sbuf );
					
					// Read voltage
					blk_state[fd_idx].bat_volt = sbufReadU16( &sbuf );
					
					pthread_mutex_unlock( &blk_state[fd_idx].update_mutex );
				  
			  }
				else {
					return BLK_MSP_ERROR_DEC;
				}
		  }
		  else {
			  return BLK_MSP_ERROR_READ;
		  }
	  }
	  else {
		  return BLK_MSP_ERROR_WRITE;
	  }
  }
  else {
	  return ret;
  }

  return 0;
}

//
// Get IMU raw data
//
int blk_get_imu( 	uint32_t serial_nb ) {
  uint8_t buf[MSP_PROTOCOL_MAX_BUF_SZ];
  int 		ret, ret2;
  sbuf_t	sbuf;
  int			fd_idx;
  int			i;
  
  // Get fd index
  fd_idx = blk_get_fd( serial_nb );
  
  // Check if fd index is valid
  if ( fd_idx == BLK_ERROR_FD )
    return BLK_ERROR_FD;
  
  ret = msp_encode( fd_idx, MSP_RAW_IMU, NULL, 0 );
  if ( ret > 0 )	{
  	ret2 = blk_write( serial_nb, blk_msp[fd_idx].msp_buf_out, ret );
	  if ( ret2 == ret  )	{
	    pthread_mutex_lock( &blk_state[fd_idx].read_mutex );
	  	ret = blk_read( serial_nb, buf, MSP_PROTOCOL_MAX_BUF_SZ );
	  	pthread_mutex_unlock( &blk_state[fd_idx].read_mutex );
		  if ( ret > 0 )	{
		  	ret = msp_decode( fd_idx, buf, ret );
			  if ( ret > 0 )	{
			  
				  // Packet is sccessfully decoded
				  sbufInit( &sbuf, blk_msp[fd_idx].msp_buf_in, (uint8_t*)(blk_msp[fd_idx].msp_buf_in+ret) );
				  
				  pthread_mutex_lock( &blk_state[fd_idx].update_mutex );
				  // Read accelerometers
				  for ( i = 0; i < 3; i++ )
				  	blk_state[fd_idx].acc[i] = sbufReadU16( &sbuf );
				  	
				  // Read gyros
				  for ( i = 0; i < 3; i++ )
				  	blk_state[fd_idx].gyr[i] = sbufReadU16( &sbuf );
				  	
				  // Read magnetometers
				  for ( i = 0; i < 3; i++ )
				  	blk_state[fd_idx].mag[i] = sbufReadU16( &sbuf );
				  pthread_mutex_unlock( &blk_state[fd_idx].update_mutex );
			  }
				else {
					return BLK_MSP_ERROR_DEC;
				}
		  }
		  else {
			  return BLK_MSP_ERROR_READ;
		  }
	  }
	  else {
		  return BLK_MSP_ERROR_WRITE;
	  }
  }
  else {
	  return ret;
  }

  return 0;
}

//
// Motor arming enable control
//
int blk_enable_motor( 	uint32_t serial_nb, uint8_t flag ) {
  uint8_t buf[MSP_PROTOCOL_MAX_BUF_SZ];
  int 		ret, ret2;
  sbuf_t	sbuf;
  int			fd_idx;
  
  // Get fd index
  fd_idx = blk_get_fd( serial_nb );
  
  // Check if fd index is valid
  if ( fd_idx == BLK_ERROR_FD )
    return BLK_ERROR_FD;
  
  // Define MSP data
  sbufInit( &sbuf, buf, (uint8_t*)(buf+MSP_PROTOCOL_MAX_BUF_SZ) );
  if ( flag ) {
  	sbufWriteU8( &sbuf, false );		// ArmingDisabled
  	sbufWriteU8( &sbuf, true );			// disableRunawayTakeoff
  	}
  else {
	  sbufWriteU8( &sbuf, true );			// ArmingDisabled
  	sbufWriteU8( &sbuf, false );		// disableRunawayTakeoff
  }
  
  // Start communication
  ret = msp_encode( fd_idx, MSP_SET_ARMING_DISABLED, buf, sbufIndex(&sbuf) );
  if ( ret > 0 )	{
  	ret2 = blk_write( serial_nb, blk_msp[fd_idx].msp_buf_out, ret );
	  if ( ret2 == ret  )	{
	    pthread_mutex_lock( &blk_state[fd_idx].read_mutex );
	  	ret = blk_read( serial_nb, buf, MSP_PROTOCOL_MAX_BUF_SZ );
	  	pthread_mutex_unlock( &blk_state[fd_idx].read_mutex );
		  if ( ret > 0 )	{
		  	ret = msp_decode( fd_idx, buf, ret );
			  if ( ret == 0 )	{
			  
				  // Packet is sccessfully decoded
				  return 0;
			  }
				else {
					return BLK_MSP_ERROR_DEC;
				}
		  }
		  else {
			  return BLK_MSP_ERROR_READ;
		  }
	  }
	  else {
		  return BLK_MSP_ERROR_WRITE;
	  }
  }
  else {
	  return ret;
  }
}

//
// Motor throttle_control
// BLK_PWM_RANGE_MIN : motor stop
//
int blk_set_motor( 	uint32_t serial_nb, uint16_t *throttle ) {
  uint8_t buf[MSP_PROTOCOL_MAX_BUF_SZ];
  int 		ret, ret2;
  sbuf_t	sbuf;
  int			fd_idx;
  int			i;
  
  // Get fd index
  fd_idx = blk_get_fd( serial_nb );
  
  // Check if fd index is valid
  if ( fd_idx == BLK_ERROR_FD )
    return BLK_ERROR_FD;
  
  // Consistency check
  if ( ( blk_state[fd_idx].motor_count == 0 ) || 
  		 ( blk_state[fd_idx].motor_count > BLK_MAX_MOTORS ) )
  	return BLK_ERROR_INT;
  	
  // Define MSP data
  sbufInit( &sbuf, buf, (uint8_t*)(buf+MSP_PROTOCOL_MAX_BUF_SZ) );
  for ( i = 0; i < blk_state[fd_idx].motor_count; i++ )	{
  	if ( throttle[i] < BLK_PWM_RANGE_MIN )
  		sbufWriteU16( &sbuf, BLK_PWM_RANGE_MIN );
  	else if ( throttle[i] > BLK_PWM_RANGE_MAX )
  		sbufWriteU16( &sbuf, BLK_PWM_RANGE_MAX );
  	else
	  	sbufWriteU16( &sbuf, throttle[i] );
	}
  
  // Start communication
  ret = msp_encode( fd_idx, MSP_SET_MOTOR, buf, sbufIndex(&sbuf) );
  if ( ret > 0 )	{
  	ret2 = blk_write( serial_nb, blk_msp[fd_idx].msp_buf_out, ret );
	  if ( ret2 == ret  )	{
	    pthread_mutex_lock( &blk_state[fd_idx].read_mutex );
	  	ret = blk_read( serial_nb, buf, MSP_PROTOCOL_MAX_BUF_SZ );
	  	pthread_mutex_unlock( &blk_state[fd_idx].read_mutex );
		  if ( ret > 0 )	{
		  	ret = msp_decode( fd_idx, buf, ret );
			  if ( ret == 0 )	{
			  
				  // Packet is sccessfully decoded
				  return 0;
			  }
				else {
					return BLK_MSP_ERROR_DEC;
				}
		  }
		  else {
			  return BLK_MSP_ERROR_READ;
		  }
	  }
	  else {
		  return BLK_MSP_ERROR_WRITE;
	  }
  }
  else {
	  return ret;
  }
}

//
// Detect if betalink firmware is available
//
int blk_detect( uint32_t serial_nb )	{
	uint8_t 	buf[MSP_PROTOCOL_MAX_BUF_SZ];
  int 			ret, ret2;
  sbuf_t		sbuf;
  int				fd_idx;
  int				i;
  uint16_t	throttle[BLK_MAX_MOTORS] = { 	BLK_PWM_RANGE_MIN,
																					BLK_PWM_RANGE_MIN,
																					BLK_PWM_RANGE_MIN,
																					BLK_PWM_RANGE_MIN,
																					BLK_PWM_RANGE_MIN,
																					BLK_PWM_RANGE_MIN,
																					BLK_PWM_RANGE_MIN,
																					BLK_PWM_RANGE_MIN };
  
  // Get fd index
  fd_idx = blk_get_fd( serial_nb );
  
  // Check if fd index is valid
  if ( fd_idx == BLK_ERROR_FD )
    return BLK_ERROR_FD;
    
  // By default, no betalink firmware
  blk_state[fd_idx].betalink_detected = false;
  
  // Consistency check
  if ( ( blk_state[fd_idx].motor_count == 0 ) || 
  		 ( blk_state[fd_idx].motor_count > BLK_MAX_MOTORS ) )
  	return BLK_ERROR_INT;
  	
  // Define MSP data
  sbufInit( &sbuf, buf, (uint8_t*)(buf+MSP_PROTOCOL_MAX_BUF_SZ) );
  for ( i = 0; i < blk_state[fd_idx].motor_count; i++ )	{
  	if ( throttle[i] < BLK_PWM_RANGE_MIN )
  		sbufWriteU16( &sbuf, BLK_PWM_RANGE_MIN );
  	else if ( throttle[i] > BLK_PWM_RANGE_MAX )
  		sbufWriteU16( &sbuf, BLK_PWM_RANGE_MAX );
  	else
	  	sbufWriteU16( &sbuf, throttle[i] );
	}
  
  // Start communication
  ret = msp_encode( fd_idx, MSP_BETALINK, buf, sbufIndex(&sbuf) );
  if ( ret > 0 )	{
  	ret2 = blk_write( serial_nb, blk_msp[fd_idx].msp_buf_out, ret );
	  if ( ret2 == ret  )	{
	    pthread_mutex_lock( &blk_state[fd_idx].read_mutex );
	  	ret = blk_read( serial_nb, buf, MSP_PROTOCOL_MAX_BUF_SZ );
	  	pthread_mutex_unlock( &blk_state[fd_idx].read_mutex );
		  if ( ret > 0 )	{
		  	ret = msp_decode( fd_idx, buf, ret );
			  if ( ret > 0 )	{
			  
				  // Packet is sccessfully decoded
				  sbufInit( &sbuf, blk_msp[fd_idx].msp_buf_in, (uint8_t*)(blk_msp[fd_idx].msp_buf_in+ret) );
				  
				  pthread_mutex_lock( &blk_state[fd_idx].update_mutex );
				  // Read timestamp
		      blk_state[fd_idx].timestamp = sbufReadU32( &sbuf );
		  
				  // Read voltage
					blk_state[fd_idx].bat_volt = sbufReadU16( &sbuf );
					
					// Read current
					blk_state[fd_idx].bat_amp = sbufReadU16( &sbuf );
					
					// Read mAh drawn
					blk_state[fd_idx].bat_mah = sbufReadU16( &sbuf );
					
					// Read accelerometers
				  for ( i = 0; i < 3; i++ )
				  	blk_state[fd_idx].acc[i] = sbufReadU16( &sbuf );
				  	
				  // Read gyros
				  for ( i = 0; i < 3; i++ )
				  	blk_state[fd_idx].gyr[i] = sbufReadU16( &sbuf );
				  	
				  // Read magnetometers
				  for ( i = 0; i < 3; i++ )
				  	blk_state[fd_idx].mag[i] = sbufReadU16( &sbuf );
				  	
				  // Read Euler angles
				  blk_state[fd_idx].roll = sbufReadU16( &sbuf );
				  blk_state[fd_idx].pitch = sbufReadU16( &sbuf );
				  blk_state[fd_idx].yaw = sbufReadU16( &sbuf );
					
					// Read motor data
				  for ( i = 0; i < blk_state[fd_idx].motor_count; i++ )	{
					  blk_state[fd_idx].rpm[i] = sbufReadU32( &sbuf );
					  blk_state[fd_idx].inv[i] = sbufReadU16( &sbuf );
				  }
					
					blk_state[fd_idx].betalink_detected = true;
					pthread_mutex_unlock( &blk_state[fd_idx].update_mutex );
					fprintf( stderr, "BLK: betalink firmware detected!\n" );
					
				  return 0;
			  }
				else {
					return BLK_MSP_ERROR_DEC;
				}
		  }
		  else {
			  return BLK_MSP_ERROR_READ;
		  }
	  }
	  else {
		  return BLK_MSP_ERROR_WRITE;
	  }
  }
  else {
	  return ret;
  }
}

//
// Send motor throttle and refresh sensor values
// BLK_PWM_RANGE_MIN : motor stop
//
int blk_update( 	uint32_t serial_nb, uint16_t *throttle ) {
	uint8_t buf[MSP_PROTOCOL_MAX_BUF_SZ];
  int 		ret, ret2;
  sbuf_t	sbuf;
  int			fd_idx;
  int			i;
  
  // Get fd index
  fd_idx = blk_get_fd( serial_nb );
  
  // Check if fd index is valid
  if ( fd_idx == BLK_ERROR_FD )
    return BLK_ERROR_FD;
  
  // Consistency check
  if ( ( blk_state[fd_idx].motor_count == 0 ) || 
  		 ( blk_state[fd_idx].motor_count > BLK_MAX_MOTORS ) )
  	return BLK_ERROR_INT;
  
  // Multiple transactions when no betalink firmware is detected
  if ( blk_state[fd_idx].betalink_detected == false )	{
  	ret = blk_set_motor( serial_nb, throttle );
  	if ( ret )
  		return ret;
	  ret = blk_get_imu( serial_nb );
	  if ( ret )
  		return ret;
	  ret = blk_get_battery_state( serial_nb );
	  if ( ret )
  		return ret;
	  ret = blk_get_motor_telemetry( serial_nb );
	  return ret;
  }
  
  // Define MSP data
  sbufInit( &sbuf, buf, (uint8_t*)(buf+MSP_PROTOCOL_MAX_BUF_SZ) );
  for ( i = 0; i < blk_state[fd_idx].motor_count; i++ )	{
  	if ( throttle[i] < BLK_PWM_RANGE_MIN )
  		sbufWriteU16( &sbuf, BLK_PWM_RANGE_MIN );
  	else if ( throttle[i] > BLK_PWM_RANGE_MAX )
  		sbufWriteU16( &sbuf, BLK_PWM_RANGE_MAX );
  	else
	  	sbufWriteU16( &sbuf, throttle[i] );
	}
  
  // Start communication
  ret = msp_encode( fd_idx, MSP_BETALINK, buf, sbufIndex(&sbuf) );
  if ( ret > 0 )	{
  	ret2 = blk_write( serial_nb, blk_msp[fd_idx].msp_buf_out, ret );
	  if ( ret2 == ret  )	{
	    pthread_mutex_lock( &blk_state[fd_idx].read_mutex );
	  	ret = blk_read( serial_nb, buf, MSP_PROTOCOL_MAX_BUF_SZ );
	  	pthread_mutex_unlock( &blk_state[fd_idx].read_mutex );
		  if ( ret > 0 )	{
		  	ret = msp_decode( fd_idx, buf, ret );
			  if ( ret > 0 )	{
			  
				  // Packet is sccessfully decoded
				  sbufInit( &sbuf, blk_msp[fd_idx].msp_buf_in, (uint8_t*)(blk_msp[fd_idx].msp_buf_in+ret) );
				  
				  pthread_mutex_lock( &blk_state[fd_idx].update_mutex );
				  // Read timestamp
		      blk_state[fd_idx].timestamp = sbufReadU32( &sbuf );
		      
				  // Read voltage
					blk_state[fd_idx].bat_volt = sbufReadU16( &sbuf );
					
					// Read current
					blk_state[fd_idx].bat_amp = sbufReadU16( &sbuf );
					
					// Read mAh drawn
					blk_state[fd_idx].bat_mah = sbufReadU16( &sbuf );
					
					// Read accelerometers
				  for ( i = 0; i < 3; i++ )
				  	blk_state[fd_idx].acc[i] = sbufReadU16( &sbuf );
				  	
				  // Read gyros
				  for ( i = 0; i < 3; i++ )
				  	blk_state[fd_idx].gyr[i] = sbufReadU16( &sbuf );
				  	
				  // Read magnetometers
				  for ( i = 0; i < 3; i++ )
				  	blk_state[fd_idx].mag[i] = sbufReadU16( &sbuf );
				  
				  // Read Euler angles
				  blk_state[fd_idx].roll = sbufReadU16( &sbuf );
				  blk_state[fd_idx].pitch = sbufReadU16( &sbuf );
				  blk_state[fd_idx].yaw = sbufReadU16( &sbuf );
					
					// Read motor data
				  for ( i = 0; i < blk_state[fd_idx].motor_count; i++ )	{
					  blk_state[fd_idx].rpm[i] = sbufReadU32( &sbuf );
					  blk_state[fd_idx].inv[i] = sbufReadU16( &sbuf );
				  }
				  pthread_mutex_unlock( &blk_state[fd_idx].update_mutex );
					
				  return 0;
			  }
				else {
					return BLK_MSP_ERROR_DEC;
				}
		  }
		  else {
			  return BLK_MSP_ERROR_READ;
		  }
	  }
	  else {
		  return BLK_MSP_ERROR_WRITE;
	  }
  }
  else {
	  return ret;
  }
}

//
// Send motor throttle and refresh sensor values
// BLK_PWM_RANGE_MIN : motor stop
// threaded version: no waiting time for packet response
//
void *blk_update_helper_thread( void *ptr )	{
	uint64_t 	serial_nb;
	int				fd_idx;
	int 			i, ret;
	sbuf_t		sbuf;
	uint8_t 	buf[MSP_PROTOCOL_MAX_BUF_SZ];
	
	// Declare the thread as detached so its ressources are released at termination
	pthread_detach( pthread_self() );
	
	serial_nb = (uint64_t)(*(uint64_t*)ptr);
	
	// Get fd index
  fd_idx = blk_get_fd( serial_nb );
  
  // Check if fd index is valid
  if ( fd_idx == BLK_ERROR_FD )	{
	  free( ptr );
	  pthread_mutex_unlock( &blk_state[fd_idx].read_mutex );
    return NULL;
  }
	
	// Read response
	ret = blk_read( serial_nb, buf, MSP_PROTOCOL_MAX_BUF_SZ );
	
	// Critical section
	pthread_mutex_lock( &blk_state[fd_idx].update_mutex );
	if ( ret > 0 )	{
		ret = msp_decode( fd_idx, buf, ret );
	  if ( ret > 0 )	{
	  
		  // Packet is sccessfully decoded
		  sbufInit( &sbuf, blk_msp[fd_idx].msp_buf_in, (uint8_t*)(blk_msp[fd_idx].msp_buf_in+ret) );
		  
		  // Read timestamp
		  blk_state[fd_idx].timestamp = sbufReadU32( &sbuf );
		  
		  // Read voltage
			blk_state[fd_idx].bat_volt = sbufReadU16( &sbuf );
			
			// Read current
			blk_state[fd_idx].bat_amp = sbufReadU16( &sbuf );
			
			// Read mAh drawn
			blk_state[fd_idx].bat_mah = sbufReadU16( &sbuf );
			
			// Read accelerometers
		  for ( i = 0; i < 3; i++ )
		  	blk_state[fd_idx].acc[i] = sbufReadU16( &sbuf );
		  	
		  // Read gyros
		  for ( i = 0; i < 3; i++ )
		  	blk_state[fd_idx].gyr[i] = sbufReadU16( &sbuf );
		  	
		  // Read magnetometers
		  for ( i = 0; i < 3; i++ )
		  	blk_state[fd_idx].mag[i] = sbufReadU16( &sbuf );
		  
		  // Read Euler angles
		  blk_state[fd_idx].roll = sbufReadU16( &sbuf );
		  blk_state[fd_idx].pitch = sbufReadU16( &sbuf );
		  blk_state[fd_idx].yaw = sbufReadU16( &sbuf );
			
			// Read motor data
		  for ( i = 0; i < blk_state[fd_idx].motor_count; i++ )	{
			  blk_state[fd_idx].rpm[i] = sbufReadU32( &sbuf );
			  blk_state[fd_idx].inv[i] = sbufReadU16( &sbuf );
		  }
	  }
	}
	pthread_mutex_unlock( &blk_state[fd_idx].update_mutex );
	
	free( ptr );
	
	pthread_mutex_unlock( &blk_state[fd_idx].read_mutex );
	return NULL;
}

int blk_update_threaded( 	uint32_t serial_nb, uint16_t *throttle ) {
	uint8_t buf[MSP_PROTOCOL_MAX_BUF_SZ];
  int 		ret, ret2;
  sbuf_t	sbuf;
  int			fd_idx;
  int			i;
  
  // Get fd index
  fd_idx = blk_get_fd( serial_nb );
  
  // Check if fd index is valid
  if ( fd_idx == BLK_ERROR_FD )
    return BLK_ERROR_FD;
  
  // Consistency check
  if ( ( blk_state[fd_idx].motor_count == 0 ) || 
  		 ( blk_state[fd_idx].motor_count > BLK_MAX_MOTORS ) )
  	return BLK_ERROR_INT;
  
  // Multiple transactions when no betalink firmware is detected
  if ( blk_state[fd_idx].betalink_detected == false )	{
  	ret = blk_set_motor( serial_nb, throttle );
  	if ( ret )
  		return ret;
	  ret = blk_get_imu( serial_nb );
	  if ( ret )
  		return ret;
	  ret = blk_get_battery_state( serial_nb );
	  if ( ret )
  		return ret;
	  ret = blk_get_motor_telemetry( serial_nb );
	  return ret;
  }
  
  // Define MSP data
  sbufInit( &sbuf, buf, (uint8_t*)(buf+MSP_PROTOCOL_MAX_BUF_SZ) );
  for ( i = 0; i < blk_state[fd_idx].motor_count; i++ )	{
  	if ( throttle[i] < BLK_PWM_RANGE_MIN )
  		sbufWriteU16( &sbuf, BLK_PWM_RANGE_MIN );
  	else if ( throttle[i] > BLK_PWM_RANGE_MAX )
  		sbufWriteU16( &sbuf, BLK_PWM_RANGE_MAX );
  	else
	  	sbufWriteU16( &sbuf, throttle[i] );
	}
  
  // Start communication
  ret = msp_encode( fd_idx, MSP_BETALINK, buf, sbufIndex(&sbuf) );
  if ( ret > 0 )	{
  	ret2 = blk_write( serial_nb, blk_msp[fd_idx].msp_buf_out, ret );
	  if ( ret2 == ret  )	{
	  	pthread_t reader_thread;
	  	uint64_t	*serial_nb_pt = malloc( sizeof( uint64_t ) );
	  	
	  	*serial_nb_pt = serial_nb;
	  	
	  	// Create listening thread
	  	// Mutex allows for a single thread to be created at a time
	  	pthread_mutex_lock( &blk_state[fd_idx].read_mutex );
	  	pthread_create( &reader_thread, NULL , blk_update_helper_thread, (void*)(serial_nb_pt) );
	  	return 0;
	  }
	  else {
		  return BLK_MSP_ERROR_WRITE;
	  }
  }
  else {
	  return ret;
  }
}

//
// Copy state in local structure atomically
//
int blk_copy_state( uint32_t serial_nb, blk_state_t *state )	{
	int fd_idx;
	
	fd_idx = blk_get_fd( serial_nb );
  
  // Check if fd index is valid
  if ( fd_idx == BLK_ERROR_FD )
    return BLK_ERROR_FD;
  
  // Critical section
	pthread_mutex_lock( &blk_state[fd_idx].update_mutex );
	if ( state )
		memcpy( state, &blk_state[fd_idx], sizeof( blk_state_t ) );
	pthread_mutex_unlock( &blk_state[fd_idx].update_mutex );
	
	return 0;	
}

//
// Display all states
//
void blk_dump_fc_state( uint32_t serial_nb )	{
	int 				i, fd_idx, ret;
	blk_state_t state;
	
	// Get fd index
  fd_idx = blk_get_fd( BLK_DEV_SERIALNB );
  
  // Check if fd index is valid
  if ( fd_idx == BLK_ERROR_FD )	{
  	fprintf( stderr, "BLK: unable to find serial port descriptor.\n" );
    return;
  }
	
	// Read current state
	ret = blk_copy_state( serial_nb, &state );
	if ( ret )
		return;
		
	// Dump initial FC state
  printf( "> Motor count:              %d\n", state.motor_count );
  printf( "> Battery cell count:       %d\n", state.bat_cell_cnt );
  printf( "> Battery capacity [mAh]:   %d\n", state.bat_capa );
  printf( "> Battery voltage [V]:      %.2f\n", (float)state.bat_volt * BLK_MSP_BATV_SCALING );
  printf( "> Battery mAh drawn [mAh]:  %d\n", state.bat_mah );
  printf( "> Battery current [A]:      %.2f\n", (float)state.bat_amp * BLK_MSP_BATA_SCALING );
  printf( "> Accelerometer [m/s/s]:    x:%.2f\ty:%.2f\tz:%.2f\n",
                (float)state.acc[0] * BLK_MSP_ACC_SCALING,
                (float)state.acc[1] * BLK_MSP_ACC_SCALING,
                (float)state.acc[2] * BLK_MSP_ACC_SCALING );
  printf( "> Gyrometer [deg/s]:        x:%.2f\ty:%.2f\tz:%.2f\n",
                (float)state.gyr[0] * BLK_MSP_GYR_SCALING,
                (float)state.gyr[1] * BLK_MSP_GYR_SCALING,
                (float)state.gyr[2] * BLK_MSP_GYR_SCALING );
  printf( "> Magnetometer [uT]:        x:%.2f\ty:%.2f\tz:%.2f\n",
                (float)state.mag[0] * BLK_MSP_MAG_SCALING,
                (float)state.mag[1] * BLK_MSP_MAG_SCALING,
                (float)state.mag[2] * BLK_MSP_MAG_SCALING );
  printf( "> Roll [deg]:               %.2f\n", (float)state.roll * BLK_MSP_ANGLE_SCALING );
  printf( "> Pitch [deg]:              %.2f\n", (float)state.pitch * BLK_MSP_ANGLE_SCALING );
  printf( "> Yaw [deg]:                %.2f\n", (float)state.yaw * BLK_MSP_ANGLE_SCALING );
                
  for ( i = 0; i < state.motor_count; i++ )	{
	  printf( "> Motor #%d:\n", i ),
	  printf( ">  Velocity [rpm]:          %d\n", state.rpm[i] );
	  printf( ">  Invalid DSHOT [%%]:       %.2f\n", (float)state.inv[i] * BLK_MSP_INV_SCALING );
	  printf( ">  Temperature [degC]:      %d\n", state.temp[i] );
	  printf( ">  Voltage [V]:             %.2f\n", (float)state.volt[i] * BLK_MSP_MOTV_SCALING );
	  printf( ">  Current [A]:             %.2f\n", (float)state.amp[i] * BLK_MSP_MOTA_SCALING );
	  printf( ">  Consumption [mAh]:       %d\n", state.mah[i] );
  }
}

//
//	MSP from betaflight-configurator (translated from javascript)
//

//
//	MSP decoder
//
int msp_decode( int fd_idx, uint8_t* data, uint16_t size )	{
	uint16_t 	i, ii;
	uint8_t 	chunk;
  
  // Reset variables
  blk_msp[fd_idx].msp_state = MSP_IDLE;
  blk_msp[fd_idx].msp_message_length_received = 0;
  blk_msp[fd_idx].msp_messageIsJumboFrame = false;
  blk_msp[fd_idx].msp_crcError = false;
  
	for ( i = 0; i < size; i++ ) {
		chunk = data[i];
  	switch ( blk_msp[fd_idx].msp_state ) {
      case MSP_IDLE: // sync char 1
          if ( chunk == MSP_SYM_BEGIN ) {
          		// Reset variables
						  blk_msp[fd_idx].msp_message_length_received = 0;
						  blk_msp[fd_idx].msp_messageIsJumboFrame = false;
						  blk_msp[fd_idx].msp_crcError = false;
              blk_msp[fd_idx].msp_state = MSP_PROTO_IDENTIFIER;
          }
          break;
      case MSP_PROTO_IDENTIFIER: // sync char 2
          switch ( chunk ) {
              case MSP_SYM_PROTO_V1:
                  blk_msp[fd_idx].msp_state = MSP_DIRECTION_V1;
                  break;
              case MSP_SYM_PROTO_V2:
                  blk_msp[fd_idx].msp_state = MSP_DIRECTION_V2;
                  break;
              default:
                  fprintf( stderr, "BLK: Unknown protocol char %c\n", chunk );
                  blk_msp[fd_idx].msp_state = MSP_IDLE;
          }
          break;
      case MSP_DIRECTION_V1: // direction (should be >)
      case MSP_DIRECTION_V2:
          blk_msp[fd_idx].msp_unsupported = 0;
          switch ( chunk ) {
              case MSP_SYM_FROM_MWC:
                  blk_msp[fd_idx].msp_message_direction = 1;
                  break;
              case MSP_SYM_TO_MWC:
                  blk_msp[fd_idx].msp_message_direction = 0;
                  break;
              case MSP_SYM_UNSUPPORTED:
                  blk_msp[fd_idx].msp_unsupported = 1;
                  break;
          }
          blk_msp[fd_idx].msp_state = blk_msp[fd_idx].msp_state == MSP_DIRECTION_V1 ?
                  MSP_PAYLOAD_LENGTH_V1 :
                  MSP_FLAG_V2;
          break;
      case MSP_FLAG_V2:
          // Ignored for now
          blk_msp[fd_idx].msp_state = MSP_CODE_V2_LOW;
          break;
      case MSP_PAYLOAD_LENGTH_V1:
          blk_msp[fd_idx].msp_message_length_expected = chunk;

          if ( blk_msp[fd_idx].msp_message_length_expected == MSP_JUMBO_FRAME_MIN_SIZE ) {
          		blk_msp[fd_idx].msp_messageIsJumboFrame = true;
              blk_msp[fd_idx].msp_state = MSP_CODE_JUMBO_V1;
          } else {
              blk_msp[fd_idx].msp_state = MSP_CODE_V1;
          }
          break;
      case MSP_PAYLOAD_LENGTH_V2_LOW:
          blk_msp[fd_idx].msp_message_length_expected = chunk;
          blk_msp[fd_idx].msp_state = MSP_PAYLOAD_LENGTH_V2_HIGH;
          break;
      case MSP_PAYLOAD_LENGTH_V2_HIGH:
          blk_msp[fd_idx].msp_message_length_expected |= chunk << 8;
          blk_msp[fd_idx].msp_state = blk_msp[fd_idx].msp_message_length_expected > 0 ?
              MSP_PAYLOAD_V2 :
              MSP_CHECKSUM_V2;
          break;
      case MSP_CODE_V1:
      case MSP_CODE_JUMBO_V1:
          blk_msp[fd_idx].msp_code = chunk;
          if ( blk_msp[fd_idx].msp_message_length_expected > 0 ) {
              // process payload
              if ( blk_msp[fd_idx].msp_state == MSP_CODE_JUMBO_V1 ) {
                  blk_msp[fd_idx].msp_state = MSP_PAYLOAD_LENGTH_JUMBO_LOW;
              } else {
                  blk_msp[fd_idx].msp_state = MSP_PAYLOAD_V1;
              }
          } else {
              // no payload
              blk_msp[fd_idx].msp_state = MSP_CHECKSUM_V1;
          }
          break;
      case MSP_CODE_V2_LOW:
          blk_msp[fd_idx].msp_code = chunk;
          blk_msp[fd_idx].msp_state = MSP_CODE_V2_HIGH;
          break;
      case MSP_CODE_V2_HIGH:
          blk_msp[fd_idx].msp_code |= chunk << 8;
          blk_msp[fd_idx].msp_state = MSP_PAYLOAD_LENGTH_V2_LOW;
          break;
      case MSP_PAYLOAD_LENGTH_JUMBO_LOW:
          blk_msp[fd_idx].msp_message_length_expected = chunk;
          blk_msp[fd_idx].msp_state = MSP_PAYLOAD_LENGTH_JUMBO_HIGH;
          break;
      case MSP_PAYLOAD_LENGTH_JUMBO_HIGH:
          blk_msp[fd_idx].msp_message_length_expected |= chunk << 8;
          blk_msp[fd_idx].msp_state = MSP_PAYLOAD_V1;
          break;
      case MSP_PAYLOAD_V1:
      case MSP_PAYLOAD_V2:
      		if ( blk_msp[fd_idx].msp_message_length_expected > MSP_PROTOCOL_MAX_BUF_SZ )	{
        		fprintf( stderr, "BLK: Payload too big (%d)\n", blk_msp[fd_idx].msp_message_length_expected );
        		blk_msp[fd_idx].msp_state = MSP_IDLE;
      		}
          blk_msp[fd_idx].msp_buf_in[blk_msp[fd_idx].msp_message_length_received] = chunk;
          blk_msp[fd_idx].msp_message_length_received++;

          if ( blk_msp[fd_idx].msp_message_length_received >= blk_msp[fd_idx].msp_message_length_expected ) {
              blk_msp[fd_idx].msp_state = blk_msp[fd_idx].msp_state == MSP_PAYLOAD_V1 ?
                  MSP_CHECKSUM_V1 :
                  MSP_CHECKSUM_V2;
          }
          break;
      case MSP_CHECKSUM_V1:
          if ( blk_msp[fd_idx].msp_message_length_expected >= MSP_JUMBO_FRAME_MIN_SIZE ) {
              blk_msp[fd_idx].msp_message_checksum = MSP_JUMBO_FRAME_MIN_SIZE;
          } else {
              blk_msp[fd_idx].msp_message_checksum = blk_msp[fd_idx].msp_message_length_expected;
          }
          blk_msp[fd_idx].msp_message_checksum ^= blk_msp[fd_idx].msp_code;
          if ( blk_msp[fd_idx].msp_message_length_expected >= MSP_JUMBO_FRAME_MIN_SIZE ) {
              blk_msp[fd_idx].msp_message_checksum ^= blk_msp[fd_idx].msp_message_length_expected & 0xFF;
              blk_msp[fd_idx].msp_message_checksum ^= ( blk_msp[fd_idx].msp_message_length_expected & 0xFF00 ) >> 8;
          }
          for ( ii = 0; ii < blk_msp[fd_idx].msp_message_length_received; ii++ ) {
              blk_msp[fd_idx].msp_message_checksum ^= blk_msp[fd_idx].msp_buf_in[ii];
          }
          
          // Return at the first valid packet
          if ( blk_msp[fd_idx].msp_message_checksum == chunk ) {
          	blk_msp[fd_idx].msp_crcError = false;
          	return blk_msp[fd_idx].msp_message_length_received;
          }
          else {
          	fprintf( stderr, "BLK: Checksum error (%d != %d)\n", blk_msp[fd_idx].msp_message_checksum, chunk );
          	blk_msp[fd_idx].msp_crcError = true;
          	blk_msp[fd_idx].msp_packet_error++;
          	blk_msp[fd_idx].msp_state = MSP_IDLE;
          	}
          break;
      case MSP_CHECKSUM_V2:
          blk_msp[fd_idx].msp_message_checksum = 0;
          blk_msp[fd_idx].msp_message_checksum = msp_crc8_dvb_s2( blk_msp[fd_idx].msp_message_checksum, 0 ); // flag
          blk_msp[fd_idx].msp_message_checksum = msp_crc8_dvb_s2( blk_msp[fd_idx].msp_message_checksum, blk_msp[fd_idx].msp_code & 0xFF );
          blk_msp[fd_idx].msp_message_checksum = msp_crc8_dvb_s2( blk_msp[fd_idx].msp_message_checksum, 
          	(blk_msp[fd_idx].msp_code & 0xFF00) >> 8 );
          blk_msp[fd_idx].msp_message_checksum = msp_crc8_dvb_s2( blk_msp[fd_idx].msp_message_checksum,
          	blk_msp[fd_idx].msp_message_length_expected & 0xFF );
          blk_msp[fd_idx].msp_message_checksum = msp_crc8_dvb_s2( blk_msp[fd_idx].msp_message_checksum, 
          	(blk_msp[fd_idx].msp_message_length_expected & 0xFF00) >> 8 );
          for ( ii = 0; ii < blk_msp[fd_idx].msp_message_length_received; ii++ ) {
            blk_msp[fd_idx].msp_message_checksum = msp_crc8_dvb_s2( blk_msp[fd_idx].msp_message_checksum, blk_msp[fd_idx].msp_buf_in[ii] );
          }
          // Return at the first valid packet
          if ( blk_msp[fd_idx].msp_message_checksum == chunk ) {
          	blk_msp[fd_idx].msp_crcError = false;
          	return blk_msp[fd_idx].msp_message_length_received;
          }
          else {
          	fprintf( stderr, "BLK: Checksum error (%d != %d)\n", blk_msp[fd_idx].msp_message_checksum, chunk );
          	blk_msp[fd_idx].msp_crcError = true;
          	blk_msp[fd_idx].msp_packet_error++;
          	blk_msp[fd_idx].msp_state = MSP_IDLE;
          	}
          break;
      default:
          fprintf( stderr, "BLK: Unknown state detected: %d\n", blk_msp[fd_idx].msp_state );
          blk_msp[fd_idx].msp_state = MSP_IDLE;
    }
  }
	return BLK_MSP_ERROR_DEC;
}

//
//	MSPv2 encoder
//
int msp_encode( int fd_idx, uint16_t code, uint8_t *data, uint16_t data_length ) {
	uint16_t buffer_size;
	uint16_t i;
	
	// 9 bytes for protocol overhead
	buffer_size = data_length + MSP_V2_OVERHEAD;
	
	if ( buffer_size > MSP_PROTOCOL_MAX_BUF_SZ )	{
		fprintf( stderr, "BLK: Output data too big (%d)\n", buffer_size );
		return BLK_MSP_ERROR_INT;
	}
	            		
	// Encode header of the v2 protocol
	blk_msp[fd_idx].msp_buf_out[0] = MSP_SYM_BEGIN; 		// $
	blk_msp[fd_idx].msp_buf_out[1] = MSP_SYM_PROTO_V2; 	// X
	blk_msp[fd_idx].msp_buf_out[2] = MSP_SYM_TO_MWC; 		// <
	blk_msp[fd_idx].msp_buf_out[3] = 0;  								// flag
	blk_msp[fd_idx].msp_buf_out[4] = code & 0xFF;
	blk_msp[fd_idx].msp_buf_out[5] = (code >> 8) & 0xFF;
	blk_msp[fd_idx].msp_buf_out[6] = data_length & 0xFF;
	blk_msp[fd_idx].msp_buf_out[7] = (data_length >> 8) & 0xFF;
	for ( i = 0; i < data_length; i++ ) {
	  blk_msp[fd_idx].msp_buf_out[8 + i] = data[i];
	}
	
	// Last byte is the CRC
	blk_msp[fd_idx].msp_buf_out[buffer_size - 1] = msp_crc8_dvb_s2_data( blk_msp[fd_idx].msp_buf_out, 3, buffer_size - 1 );
	return buffer_size;
}

//
//	MSPv2 CRC calculation
//
uint8_t msp_crc8_dvb_s2_data( uint8_t *data, uint8_t start, uint8_t end ) {
  uint8_t crc = 0, i;
  
  for ( i = start; i < end; i++ ) {
      crc = msp_crc8_dvb_s2( crc, data[i] );
  }
  return crc;
}

uint8_t msp_crc8_dvb_s2( uint8_t crc, uint8_t ch ) {
	uint8_t i;
  
  crc ^= ch;
  for ( i = 0; i < 8; i++ ) {
      if ( crc & 0x80 ) {
        crc = ((crc << 1) & 0xFF) ^ 0xD5;
      } else {
        crc = (crc << 1) & 0xFF;
      }
  }
  return crc;
}

//
// streambuf API from betaflight
//

sbuf_t *sbufInit(sbuf_t *sbuf, uint8_t *ptr, uint8_t *end)
{
    sbuf->begin = ptr;
    sbuf->ptr = ptr;
    sbuf->end = end;
    return sbuf;
}

uint32_t sbufIndex(sbuf_t *sbuf)
{
		return (uint32_t)( sbuf->ptr - sbuf->begin );
}

void sbufWriteU8(sbuf_t *dst, uint8_t val)
{
    if ( sbufBytesRemaining( dst ) )
    	*dst->ptr++ = val;
    else {
    	fprintf( stderr, "BLK: sbuf write error, exceeding buffer boundary.\n");
    }
}

void sbufWriteU16(sbuf_t *dst, uint16_t val)
{
    sbufWriteU8(dst, val >> 0);
    sbufWriteU8(dst, val >> 8);
}

void sbufWriteU32(sbuf_t *dst, uint32_t val)
{
    sbufWriteU8(dst, val >> 0);
    sbufWriteU8(dst, val >> 8);
    sbufWriteU8(dst, val >> 16);
    sbufWriteU8(dst, val >> 24);
}

void sbufWriteU16BigEndian(sbuf_t *dst, uint16_t val)
{
    sbufWriteU8(dst, val >> 8);
    sbufWriteU8(dst, (uint8_t)val);
}

void sbufWriteU32BigEndian(sbuf_t *dst, uint32_t val)
{
    sbufWriteU8(dst, val >> 24);
    sbufWriteU8(dst, val >> 16);
    sbufWriteU8(dst, val >> 8);
    sbufWriteU8(dst, (uint8_t)val);
}


void sbufFill(sbuf_t *dst, uint8_t data, int len)
{
    if ( sbufBytesRemaining( dst ) >= len ) {
    	memset(dst->ptr, data, len);
    	dst->ptr += len;
    }
    else {
    	fprintf( stderr, "BLK: sbuf write error, exceeding buffer boundary.\n");
    }
}

void sbufWriteData(sbuf_t *dst, const void *data, int len)
{
    if ( sbufBytesRemaining( dst ) >= len ) {
    	memcpy(dst->ptr, data, len);
    	dst->ptr += len;
    }
    else {
    	fprintf( stderr, "BLK: sbuf write error, exceeding buffer boundary.\n");
    }
}

void sbufWriteString(sbuf_t *dst, const char *string)
{
    sbufWriteData(dst, string, strlen(string));
}

void sbufWriteStringWithZeroTerminator(sbuf_t *dst, const char *string)
{
    sbufWriteData(dst, string, strlen(string) + 1);
}

uint8_t sbufReadU8(sbuf_t *src)
{
    if ( sbufBytesRemaining( src ) )
    	return *src->ptr++;
    else {
    	fprintf( stderr, "BLK: sbuf read error, exceeding buffer boundary.\n");
    	return 0;
    }
}

uint16_t sbufReadU16(sbuf_t *src)
{
    uint16_t ret;
    ret = sbufReadU8(src);
    ret |= sbufReadU8(src) << 8;
    return ret;
}

uint32_t sbufReadU32(sbuf_t *src)
{
    uint32_t ret;
    ret = sbufReadU8(src);
    ret |= sbufReadU8(src) <<  8;
    ret |= sbufReadU8(src) << 16;
    ret |= sbufReadU8(src) << 24;
    return ret;
}

void sbufReadData(sbuf_t *src, void *data, int len)
{
    if ( sbufBytesRemaining( src ) >= len )
    	memcpy( data, src->ptr, len );
    else	{
    	fprintf( stderr, "BLK: sbuf read error, exceeding buffer boundary.\n");
    	memcpy( data, src->ptr, sbufBytesRemaining( src ) );
    }
}

// reader - return bytes remaining in buffer
// writer - return available space
int sbufBytesRemaining(sbuf_t *buf)
{
    return buf->end - buf->ptr;
}

uint8_t* sbufPtr(sbuf_t *buf)
{
    return buf->ptr;
}

const uint8_t* sbufConstPtr(const sbuf_t *buf)
{
    return buf->ptr;
}

// advance buffer pointer
// reader - skip data
// writer - commit written data
void sbufAdvance(sbuf_t *buf, int size)
{
    if ( sbufBytesRemaining( buf ) >= size )
    	buf->ptr += size;
    else
    	buf->ptr += sbufBytesRemaining( buf );
}

// modifies streambuf so that written data are prepared for reading
void sbufSwitchToReader(sbuf_t *buf, uint8_t *base)
{
    buf->end = buf->ptr;
    buf->ptr = base;
		}

#ifdef BLK_STANDALONE
//
//  main
//
int main( int argc, char *argv[] )  {

  int i, ii, ret, fd_idx;
  #ifdef BLK_TWO_FC
  int fd_idx2;
  #endif
  struct timespec     start, cur;
  unsigned long long  elapsed_us;
  blk_state_t					state;
  uint16_t						throttle_min[BLK_MAX_MOTORS] = { 	BLK_PWM_RANGE_MIN,
  																											BLK_PWM_RANGE_MIN,
  																											BLK_PWM_RANGE_MIN,
  																											BLK_PWM_RANGE_MIN,
  																											BLK_PWM_RANGE_MIN,
  																											BLK_PWM_RANGE_MIN,
  																											BLK_PWM_RANGE_MIN,
  																											BLK_PWM_RANGE_MIN };
  uint16_t						throttle_max[BLK_MAX_MOTORS] = { 	BLK_STEP_THROTTLE,
  																											BLK_STEP_THROTTLE,
  																											BLK_STEP_THROTTLE,
  																											BLK_STEP_THROTTLE,
  																											BLK_STEP_THROTTLE,
  																											BLK_STEP_THROTTLE,
  																											BLK_STEP_THROTTLE,
  																											BLK_STEP_THROTTLE };
  uint32_t						rpm;
  
  // Initialize serial port
  if ( blk_init_port( BLK_DEV_SERIALNB ) )  {
    fprintf( stderr, "Error initializing serial port.\n" );
    exit( -1 );
  }
  #ifdef BLK_TWO_FC
  if ( blk_init_port( BLK_DEV2_SERIALNB ) )  {
    fprintf( stderr, "Error initializing second serial port.\n" );
    exit( -1 );
  }
  #endif
  
  // Get fd index
  fd_idx = blk_get_fd( BLK_DEV_SERIALNB );
  #ifdef BLK_TWO_FC
  fd_idx2 = blk_get_fd( BLK_DEV2_SERIALNB );
  #endif
  
  // Check if fd index is valid
  if ( fd_idx == BLK_ERROR_FD )	{
  	fprintf( stderr, "Unable to find serial port descriptor.\n" );
    exit( -2 );
  }
  #ifdef BLK_TWO_FC
  if ( fd_idx2 == BLK_ERROR_FD )	{
  	fprintf( stderr, "Unable to find second serial port descriptor.\n" );
    exit( -2 );
  }
  #endif
  
  // Dump FC state
	blk_dump_fc_state( BLK_DEV_SERIALNB );
	#ifdef BLK_TWO_FC
	blk_dump_fc_state( BLK_DEV2_SERIALNB );
	#endif
	
	// Enable motor
	ret = blk_enable_motor( BLK_DEV_SERIALNB, true );
	if ( ret )  {
    fprintf( stderr, "Error %d in blk_enable_motor.\n", ret );
    exit( -3 );
  }
  #ifdef BLK_TWO_FC
  ret = blk_enable_motor( BLK_DEV2_SERIALNB, true );
	if ( ret )  {
    fprintf( stderr, "Error %d in blk_enable_motor on second FC.\n", ret );
    exit( -3 );
  }
  #endif
  
  // Testing roundtrip serial link duration
  for ( i = 0; i < BLK_NB_PING; i++ )  {
  	
  	// Get starting time
  	clock_gettime( CLOCK_MONOTONIC, &start );
  	
  	// Serial transaction with FC
    if ( ( i / BLK_STEP_PERIOD ) % 2 )	{
    	#ifdef BLK_THREADED_UPDATE
    	ret = blk_update_threaded( BLK_DEV_SERIALNB, throttle_max );
    	#else
    	ret = blk_update( BLK_DEV_SERIALNB, throttle_max );
    	#endif
	    if ( ret )  {
    		fprintf( stderr, "Error %d in blk_update.\n", ret );
    		break;
  		}
    }
    else {
    	#ifdef BLK_THREADED_UPDATE
    	ret = blk_update_threaded( BLK_DEV_SERIALNB, throttle_min );
    	#else
    	ret = blk_update( BLK_DEV_SERIALNB, throttle_min );
    	#endif
    	if ( ret )  {
    		fprintf( stderr, "Error %d in blk_update.\n", ret );
    		break;
  		}
    }
    
    // Compute time elapsed
    clock_gettime( CLOCK_MONOTONIC, &cur );
    elapsed_us =  ( cur.tv_sec * 1e6 + cur.tv_nsec / 1e3 ) -
                  ( start.tv_sec * 1e6 + start.tv_nsec / 1e3 );
    
    
    // Read current state
		ret = blk_copy_state( BLK_DEV_SERIALNB, &state );
		if ( ret )	{
    	fprintf( stderr, "Error %d in blk_copy_state.\n", ret );
    	break;
  	}
		
    // Display sensors
    
    rpm = 0;
    for ( ii = 0; ii < state.motor_count; ii++ )	{
	    rpm += state.rpm[ii];
    }
    rpm /= state.motor_count;
    if ( state.betalink_detected )
      fprintf(  stderr,
                "#:%u\t[%d\tus]\tax:%d\tay:%d\taz:%d\tgx:%d\tgy:%d\tgz:%d\tr:%d\tp:%d\ty:%d\trpm:%d\n",
                state.timestamp,
                (int)elapsed_us,
                state.acc[0], state.acc[1], state.acc[2],
                state.gyr[0], state.gyr[1], state.gyr[2],
                (int)(state.roll * BLK_MSP_ANGLE_SCALING),
                (int)(state.pitch * BLK_MSP_ANGLE_SCALING),
                (int)(state.yaw * BLK_MSP_ANGLE_SCALING),
                rpm );
    else
      fprintf(  stderr,
                "#:%d\t[%d\tus]\tax:%d\tay:%d\taz:%d\tgx:%d\tgy:%d\tgz:%d\trpm:%d\n",
                i,
                (int)elapsed_us,
                state.acc[0], state.acc[1], state.acc[2],
                state.gyr[0], state.gyr[1], state.gyr[2],
                rpm );
    
    #ifdef BLK_TWO_FC
    
    // Get starting time
  	clock_gettime( CLOCK_MONOTONIC, &start );
  	
  	// Serial transaction with FC
    if ( ( i / BLK_STEP_PERIOD ) % 2 )	{
    	#ifdef BLK_THREADED_UPDATE
    	ret = blk_update_threaded( BLK_DEV2_SERIALNB, throttle_max );
    	#else
    	ret = blk_update( BLK_DEV2_SERIALNB, throttle_max );
    	#endif
	    if ( ret )  {
    		fprintf( stderr, "Error %d in blk_update on second FC.\n", ret );
    		break;
  		}
    }
    else {
    	#ifdef BLK_THREADED_UPDATE
    	ret = blk_update_threaded( BLK_DEV2_SERIALNB, throttle_min );
    	#else
    	ret = blk_update( BLK_DEV2_SERIALNB, throttle_min );
    	#endif
    	if ( ret )  {
    		fprintf( stderr, "Error %d in blk_update on second FC.\n", ret );
    		break;
  		}
    }
    
    // Compute time elapsed
    clock_gettime( CLOCK_MONOTONIC, &cur );
    elapsed_us =  ( cur.tv_sec * 1e6 + cur.tv_nsec / 1e3 ) -
                  ( start.tv_sec * 1e6 + start.tv_nsec / 1e3 );
    
    
    // Read current state
		ret = blk_copy_state( BLK_DEV2_SERIALNB, &state );
		if ( ret )	{
    	fprintf( stderr, "Error %d in blk_copy_state on second FC.\n", ret );
    	break;
  	}
		
    // Display sensors
    
    rpm = 0;
    for ( ii = 0; ii < state.motor_count; ii++ )	{
	    rpm += state.rpm[ii];
    }
    rpm /= state.motor_count;
    if ( state.betalink_detected )
      fprintf(  stderr,
                "##:%u\t[%d\tus]\tax:%d\tay:%d\taz:%d\tgx:%d\tgy:%d\tgz:%d\tr:%d\tp:%d\ty:%d\trpm:%d\n",
                state.timestamp,
                (int)elapsed_us,
                state.acc[0], state.acc[1], state.acc[2],
                state.gyr[0], state.gyr[1], state.gyr[2],
                (int)(state.roll * BLK_MSP_ANGLE_SCALING),
                (int)(state.pitch * BLK_MSP_ANGLE_SCALING),
                (int)(state.yaw * BLK_MSP_ANGLE_SCALING),
                rpm );
    else
      fprintf(  stderr,
                "##:%d\t[%d\tus]\tax:%d\tay:%d\taz:%d\tgx:%d\tgy:%d\tgz:%d\trpm:%d\n",
                i,
                (int)elapsed_us,
                state.acc[0], state.acc[1], state.acc[2],
                state.gyr[0], state.gyr[1], state.gyr[2],
                rpm );
    #endif
              
    // Wait loop period
    usleep( BLK_PERIOD );
  }
  
  // Disable motor
	ret = blk_enable_motor( BLK_DEV_SERIALNB, false );
	if ( ret )  {
    fprintf( stderr, "Error %d in blk_enable_motor.\n", ret );
    exit( -3 );
  }

  // Restoring serial port initial configuration
  blk_release_port( BLK_DEV_SERIALNB );
  #ifdef BLK_TWO_FC
  blk_release_port( BLK_DEV2_SERIALNB );
  #endif
  
  return 0;
}
#endif
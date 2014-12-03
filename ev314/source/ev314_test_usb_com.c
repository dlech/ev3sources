/*
 * 	ev314_test_usb_com: test program to validate USB communication.
 * 
 * 	This is a host program. It cannot be run on the EV3.
 * 	It must be run on a Linux host connected to a EV3 device.
 * 
 * 	To compile: gcc -Wall -o ev314_test_usb_com ev314_test_usb_com.c -lrt -lusb-1.0
 * 
 * 	JG, 24.10.14
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <unistd.h>
#include <time.h>
#include <errno.h>
#include <libusb-1.0/libusb.h>
#include "ev314.h"

#define EV314_NB_EV314_DEVICES				3						// Maximum number of attached EV314 devices
#define EV314_TEST_NB_ITER						10000

#define EV314_USB_TIMEOUT             10    			// Milliseconds
#define EV314_INTERFACE_NUMBER        0
#define EV314_CONFIGURATION_NB        1
#define EV314_EP_OUT                  0x01
#define EV314_EP_IN                   0x81
#define EV314_PACKET_SIZE							0x400
#define EV314_RESET_RESPONSE_SIZE     5
#define EV314_POWER_RESPONSE_SIZE     5
#define EV314_VENDOR_LEGO             0x0694
#define EV314_PRODUCT_EV3             0x0005

/* Error codes */
#define EV314_OK                      0
#define EV314_USB_ERROR								1
#define EV314_NOT_PRESENT             2
#define EV314_CONFIGURATION_ERROR     3
#define EV314_IN_USE                  4
#define EV314_USB_WRITE_ERROR         5
#define EV314_USB_READ_ERROR          6
#define EV314_USB_PARTIAL_TRANS				7
#define EV314_USB_OVERFLOW						9
#define EV314_BYTECODE_ERROR          10

typedef int                         	EV314_error_t;

struct timespec												profiling_start;
unsigned long long										EV314_serial_list[EV314_NB_EV314_DEVICES];
unsigned char													EV314_nb_devices = 0;
struct libusb_device_handle						*EV314_hdl[EV314_NB_EV314_DEVICES];

/*
 * ev314_profiling_start: start timer
 * 
 */
void ev314_profiling_start( void )	{
	clock_gettime( CLOCK_MONOTONIC, &profiling_start );
}

/*
 * ev314_profiling_stop: stop timer and print time
 * 
 */
void ev314_profiling_stop( void )	{
	struct timespec						profiling_stop;
	
	clock_gettime( CLOCK_MONOTONIC, &profiling_stop );
	
	printf( 	"** Profiling duration: %d us.\n",
						(int)( ( profiling_stop.tv_sec - profiling_start.tv_sec ) * 1000000
								 + ( profiling_stop.tv_nsec - profiling_start.tv_nsec ) / 1000 ) );
}

/*
 * USB API.
 * 
 */
EV314_error_t EV314_init( void )  {
	libusb_device 									*dev, **devs;
	struct libusb_device_descriptor	desc;
	struct libusb_device_handle			*EV314_current_hdl = NULL;
	int															i, j, status;
  char														serial[EV314_LENGTH_SERIAL+1];
  char														serial_hex[EV314_LENGTH_SERIAL+3];
  unsigned long long							EV314_serial_tmp;
		
  status = libusb_init( NULL );
  
	if ( status ) 
		return EV314_USB_ERROR;
	
	/* Initialize usb serial number list and opened handles list */
	
	EV314_nb_devices = 0;
	for ( i = 0; i < EV314_NB_EV314_DEVICES; i++ )
		EV314_hdl[i] = NULL;

	if ( libusb_get_device_list( NULL, &devs ) < 0 )
		return EV314_USB_ERROR;
	
	/* 
	 * Go through device list loooking for EV3 devices.
	 * The EV3 devices will be sorted with increasing ID number.
	 */
	
	for ( i = 0; ( dev = devs[i] ) != NULL; i++ ) {

		status = libusb_get_device_descriptor( dev, &desc );

		if ( status >= 0 ) {
			if ( 	( desc.idVendor == EV314_VENDOR_LEGO ) &&
						( desc.idProduct == EV314_PRODUCT_EV3 ) )	{
				
				/* Open the device */
				status = libusb_open( dev, &EV314_current_hdl );
				if ( status < 0 )	{
					libusb_free_device_list( devs, 1 );
					return EV314_USB_ERROR;
				}
				
				/* Check the serial number */
				
				status = libusb_get_string_descriptor_ascii( EV314_current_hdl, desc.iSerialNumber, (unsigned char*)serial, sizeof( serial ) );
				if ( status == EV314_LENGTH_SERIAL ) {
					
					/* EV3 found */
					
					snprintf( serial_hex, EV314_LENGTH_SERIAL + 3, "0x%s", serial );
					EV314_serial_list[EV314_nb_devices++] = strtoull( serial_hex, NULL, 0 );
					
					/* Sort the list */
					
					for ( j = EV314_nb_devices - 1; j > 0; j-- )	{
						
						if ( EV314_serial_list[j] < EV314_serial_list[j-1] )	{
							
							/* Swapping */
							
							EV314_serial_tmp = EV314_serial_list[j];
							EV314_serial_list[j] = EV314_serial_list[j-1];
							EV314_serial_list[j-1] = EV314_serial_tmp;
						}
					}
					
					if ( EV314_nb_devices == EV314_NB_EV314_DEVICES )	{
						
						/* Max nd of devices reached */
						
						libusb_close( EV314_current_hdl ); 
						libusb_free_device_list( devs, 1 );
						return EV314_OK;
					}
				}

				libusb_close( EV314_current_hdl ); 
        EV314_current_hdl = NULL;
      }
		}
	}
	
	libusb_free_device_list( devs, 1 );
  
  return EV314_OK;
}


struct libusb_device_handle* EV314_find_and_open( unsigned char serial_id )  {
	char														serial[EV314_LENGTH_SERIAL+1];
	char														expected_serial_hex[EV314_LENGTH_SERIAL+3];
	libusb_device 									*dev, **devs;
	struct libusb_device_descriptor	desc;
	struct libusb_device_handle			*EV314_current_hdl = NULL;
	int															i, status, transfered;
  unsigned char	                  tmpbuf[EV314_PACKET_SIZE];
  
	if ( libusb_get_device_list( NULL, &devs ) < 0 )
		return NULL;
	
	if ( serial_id >= EV314_nb_devices )
		return NULL;
	
	/* Convert expected serial in char */
	
	snprintf( expected_serial_hex, EV314_LENGTH_SERIAL + 3, "%#014llx", EV314_serial_list[serial_id] );
	
	/* Go through device list loooking for an EV3 device */
	for ( i = 0; ( dev = devs[i] ) != NULL; i++ ) {

		status = libusb_get_device_descriptor( dev, &desc );

		if ( status >= 0 ) {
			if ( 	( desc.idVendor == EV314_VENDOR_LEGO ) &&
						( desc.idProduct == EV314_PRODUCT_EV3 ) )	{
				
				/* Open the device */
				status = libusb_open( dev, &EV314_current_hdl );
				if ( status < 0 )	{
					libusb_free_device_list( devs, 1 );
					return NULL;
				}
				
				/* Check the serial number */
				
				status = libusb_get_string_descriptor_ascii( EV314_current_hdl, desc.iSerialNumber, (unsigned char*)serial, sizeof( serial ) );
				if ( status == EV314_LENGTH_SERIAL ) {
					if ( strcmp( &expected_serial_hex[2], serial ) )	{
						libusb_close( EV314_current_hdl );
            EV314_current_hdl = NULL;
						continue;
					}
				}
				else {
					libusb_close( EV314_current_hdl ); 
          EV314_current_hdl = NULL;
					continue;
				}
				
				/* Detach possible kernel driver bound to interface */
  			libusb_detach_kernel_driver( EV314_current_hdl, EV314_INTERFACE_NUMBER );
  			
  			/* Claiming the interface */
				status = libusb_claim_interface( EV314_current_hdl, EV314_INTERFACE_NUMBER );
				if ( status )	{
					libusb_close( EV314_current_hdl );
					libusb_free_device_list( devs, 1 );
					return NULL;
				}
								
				/* Handle found and in good shape */
				EV314_hdl[serial_id] = EV314_current_hdl;
				
				/* Flush input buffer */				
				do {
          status = libusb_interrupt_transfer( EV314_hdl[serial_id], EV314_EP_IN, tmpbuf, EV314_PACKET_SIZE, &transfered, EV314_USB_TIMEOUT );
        } while ( ( status == 0 ) && ( transfered != 0 ) );
        
				libusb_free_device_list( devs, 1 );
        return EV314_hdl[serial_id];
      }
		}
	}
	
	libusb_free_device_list( devs, 1 );
  return NULL;
}

void EV314_exit( void )  {
	int i;
  
  for ( i = 0; i < EV314_nb_devices; i++ )	{
		if ( EV314_hdl[i] )	{
			libusb_release_interface( EV314_hdl[i], EV314_INTERFACE_NUMBER );
			libusb_close( EV314_hdl[i] );
			EV314_hdl[i] = NULL;
		}
	}
	
	EV314_nb_devices = 0;
		
  libusb_exit( NULL );
}

EV314_error_t EV314_send_buf( unsigned char serial_id, unsigned char *buf, int len )  {
	int	status, transfered = 0;
  
  if ( EV314_hdl[serial_id] == NULL )
    return EV314_CONFIGURATION_ERROR;
  
  if ( len > EV314_PACKET_SIZE )
  	return EV314_USB_OVERFLOW;
  
  status = libusb_interrupt_transfer( EV314_hdl[serial_id], EV314_EP_OUT, buf, len, &transfered, EV314_USB_TIMEOUT );
  
  if ( status  )
    return EV314_USB_WRITE_ERROR;
  
  if ( transfered != len )
  	return EV314_USB_PARTIAL_TRANS;

  return EV314_OK;
}

EV314_error_t EV314_recv_buf( unsigned char serial_id, unsigned char *buf, int len )  {
  int						i, status, transfered = 0;
  unsigned char	tmpbuf[EV314_PACKET_SIZE];
  
  if ( EV314_hdl[serial_id] == NULL )
    return EV314_CONFIGURATION_ERROR;
  
  if ( len > EV314_PACKET_SIZE )
  	return EV314_USB_OVERFLOW;
  
  status = libusb_interrupt_transfer( EV314_hdl[serial_id], EV314_EP_IN, tmpbuf, EV314_PACKET_SIZE, &transfered, EV314_USB_TIMEOUT );

  if ( status  )
    return EV314_USB_READ_ERROR;
  
  if ( transfered != EV314_PACKET_SIZE )
  	return EV314_USB_PARTIAL_TRANS;
  
  for ( i = 0; i < len; i++ )
  	buf[i] = tmpbuf[i];

  return EV314_OK;
}

int main( void )	{
	EV314_error_t 								ret;
	int														i, j;
	struct ev314_control_struct		ev314_control;
	struct ev314_state_struct			ev314_state;
	
	/* Initializing control structure */
	
	memset( &ev314_control, 0, sizeof( struct ev314_control_struct ) );
	
	printf( "** Scanning for EV3 devices.\n" );
	
	if ( EV314_init( ) )	{
		printf( "** Error while initializing libusb.\n" );
		exit( -1 );
	}
	
	printf( "** Found %d devices.\n", EV314_nb_devices );
	printf( "** Device list:\n" );
	
	for( j = 0; j < EV314_nb_devices; j++ )	{
		printf( "** Device %d: ID=%#014llx\n", j, EV314_serial_list[j] );
	}

	/* Opening all the devices */
	
	for ( j = 0; j < EV314_nb_devices; j++ )
		if ( ( EV314_find_and_open( j ) ) )
			printf( "** Device %d opened !\n", j );
		else {
			printf( "** Opening of device %d failed. Aborting.\n", j );
			exit( -2 );
		}

	/* Initialize encoders */
	
	ev314_control.magic = EV314_MAGIC;
	ev314_control.cmd = EV314_CMD_RESET_ENC;
	
	for ( j = 0; j < EV314_nb_devices; j++ )
		if ( ( ret = EV314_send_buf( j, (unsigned char*)&ev314_control, sizeof( ev314_control ) ) ) )
			printf( "** Error %d while resetting encoders of device %d.\n", ret, j );
	
	/* Initilize control packet */
	
	ev314_control.magic = EV314_MAGIC;
	ev314_control.cmd = EV314_CMD_CONTROL;
	ev314_control.motor_power[0] = 10000;
	ev314_control.motor_power[1] = 0;
	ev314_control.motor_power[2] = 0;
	ev314_control.motor_power[3] = 0;
	
	/* Entering status polling loop */
	
	for ( i = 0; i < EV314_TEST_NB_ITER; i++ )	{
		
		/* Send control */
		
		ev314_profiling_start( );
		
		for ( j = 0; j < EV314_nb_devices; j++ )
			if ( ( ret = EV314_send_buf( j, (unsigned char*)&ev314_control, sizeof( ev314_control ) ) ) )
				printf( "** Error %d while sending packet to device %d.\n", ret, j );
		
		/* Get response */
		
		for ( j = 0; j < EV314_nb_devices; j++ )	{
			
			memset( &ev314_state, 0, sizeof( struct ev314_state_struct ) );
		
			if ( ( ret = EV314_recv_buf( j, (unsigned char*)&ev314_state, sizeof( ev314_state ) ) ) )
				printf( "** Error %d while receiving packet from device %d.\n", ret, j );	
			
			/* Check response */
			
			if ( ev314_state.magic != EV314_MAGIC )
				printf( "** Received packet with bad magic number.\n" );
			
			/* Print response */
			
			printf( "Encoder values>\tA:%d\tB:%d\tC:%d\tD:%d\n", 
							ev314_state.motor_angle[0],
							ev314_state.motor_angle[1],
							ev314_state.motor_angle[2],
							ev314_state.motor_angle[3] );
		}
		
		ev314_profiling_stop( );				
	}
	
	EV314_exit( );
	
	return 0;
}

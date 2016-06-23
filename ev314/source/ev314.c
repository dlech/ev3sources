/*	Main source for EV3.14. 
 * 	It handles USB communication, LCD screen, actuators 
 * 	and sensors of the EV3 and is monitored by a watchdog.
 * 
 * 	Initial version: Maximilien Lesellier (maximilien.lesellier@gmail.com), June/July 2014
 * 	Further versions: Jacques Gangloff (jacques.gangloff@unistra.fr)
 */

#define _GNU_SOURCE

#include <stdio.h>
#include <stdlib.h>
#include <pthread.h>
#include <unistd.h>
#include <fcntl.h>
#include <time.h>
#include <sys/mman.h>
#include <string.h>
#include <sys/timerfd.h>
#include <sys/wait.h>
#include <sys/file.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <errno.h>
#include <endian.h>
#include <linux/fb.h>

#include "lms2012.h"

#include "ev314.h"

#define EV314_HALT_ON_EXIT																// Uncomment to halt when exiting
#define EV314_DISPLAY_ON																	// Comment to deactivate display
//#define EV314_PROFILING_ON																// Comment to deactivate profiling

#define EV314_LCD_BUFFER_SIZEX						178							// Screen buffer horizontal size in pixels
#define EV314_LCD_BUFFER_SIZEY						128							// Screen buffer vertical size in pixels
																													// Screen buffer size in bytes
#define EV314_LCD_BUFFER_SIZE 						( (int)( ( EV314_LCD_BUFFER_SIZEX + 7 ) / 8 ) * EV314_LCD_BUFFER_SIZEY )

#define EV314_LCD_SCREEN_SIZEX						480							// Screen effective horizontal size in pixels
#define EV314_LCD_SCREEN_SIZEY						128							// Screen effective vertical size in pixels
																													// Screen size in bytes
#define EV314_LCD_SCREEN_SIZE							( ( EV314_LCD_SCREEN_SIZEX / 8 ) * EV314_LCD_SCREEN_SIZEY )

#define EV314_LCD_NB_CHAR_X								35							// Maximum number of normal characters on one line 

#define EV314_MAX_USB_BUF_SIZE						1024						// Size of the USB buffer

#define EV314_LOGO_SIZE										22272						// Number of bytes in the logo
#define EV314_LOGO_X											174							// Horizontal size of the logo in pixels
#define EV314_LOGO_Y											128							// Horizontal size of the logo in pixels
#define EV314_LOGO_THRESHOLD							128							// Black and white thresholding for the logo

#define EV314_SPLASH_TIME									2000000					// Duration in us of the splash screen
#define EV314_LCD_REFRESH_PERIOD					300000					// Periodicity in us of LCD refresh
#define EV314_HP_REFRESH_PERIOD						200							// Periodicity in us of the HP thread
#define EV314_WATCHDOG_PERIOD							300000000				// Period of the watchdog in ns

#define EV314_HIGH_PRIORITY								2								// Priority level of the high priority thread
#define EV314_LOW_PRIORITY								1								// Priority level of the low priority thread
#define EV314_SCHED_POLICY								SCHED_FIFO			// Thread scheduler policy

#define EV314_MOTOR_PORT_A								0								// ID of motor port A
#define EV314_MOTOR_PORT_B								1								// ID of motor port B
#define EV314_MOTOR_PORT_C								2								// ID of motor port C
#define EV314_MOTOR_PORT_D								3								// ID of motor port D
#define EV314_MOTOR_CONTROL_SIZE					3								// Size of motor control data
#define EV314_MOTOR_BRAKE									1								// Brake mode (position regulation)
#define EV314_MOTOR_FLOAT									0								// Floating mode (rotor in open circuit)

#define EV314_MIN_BAT_VOLTAGE							5300						// Minimum battery voltage in millivolts
#define EV314_MAX_BAT_VOLTAGE							6820						// Maximum battery voltage in millivolts
#define EV314_WARN_BAT_VOLTAGE						5600						// Warning level of battery voltage
#define EV314_CRIT_BAT_VOLTAGE						5500						// Critical level of battery voltage
#define EV314_CRIT_HALT_VOLTAGE						5400						// Shutdown system when voltage under this level
#define EV314_BAT_FILTER_POLE							0.99993334			// Pole of the first-order battery level low-pass filter

#define EV314_SIZEOF_LED_COMMAND					2								// Size of message commande for LED control
#define EV314_LED_OFF											48							// LED is off
#define EV314_LED_GREEN_STATIC						49							// Green static LED
#define EV314_LED_RED_STATIC							50							// Red static LED
#define EV314_LED_ORANGE_STATIC						51							// Orange static LED
#define EV314_LED_GREEN_FLASH							52							// Green flashing LED
#define EV314_LED_RED_FLASH								53							// Red flashing LED
#define EV314_LED_ORANGE_FLASH						54							// Orange flashing LED
#define EV314_LED_GREEN_PULSE							55							// Green pulsing LED
#define EV314_LED_RED_PULSE								56							// Red pulsing LED
#define EV314_LED_ORANGE_PULSE						57							// Orange pulsing LED


#define EV314_HW_ID_FILE									"/home/root/ev314/sys/settings/BTser"

UBYTE	ev314_pixel_tab[] =
{
    0x00, // 000 00000000
    0xE0, // 001 11100000
    0x1C, // 010 00011100
    0xFC, // 011 11111100
    0x03, // 100 00000011
    0xE3, // 101 11100011
    0x1F, // 110 00011111
    0xFF  // 111 11111111
};

UBYTE ev314_logo[EV314_LOGO_SIZE]	=
{
#include "ev314_174x128_new.pgm"
};

char											ev314_led_command[EV314_SIZEOF_LED_COMMAND] = { 0, 0 };

int												ev314_lcd_fd = 0;								// LCD file descriptor
int												ev314_motor_fd = 0;							// Motor control file descriptor
int												ev314_encoder_fd = 0;						// Encoder data file descriptor
int												ev314_ui_fd = 0;								// Control panel file descriptor
int												ev314_analog_fd = 0;						// Analog inputs file descriptor
int												ev314_DCM_fd = 0;								// DCM access file descriptor
int												ev314_usb_fd = 0;								// USB device file descriptor
																													// USB buffer
unsigned char							ev314_usb_buf[EV314_MAX_USB_BUF_SIZE];
																													// Screen buffer
UBYTE											ev314_lcd_buffer[EV314_LCD_BUFFER_SIZE];						
UBYTE											*ev314_lcd_screen = NULL;				// Pointer to shared screen memory
MOTORDATA 								*ev314_motor_data = NULL;				// Pointer to motor data shared memory
UI 												*ev314_buttons = NULL;					// Pointer to control panel shared memory
ANALOG										*ev314_analog = NULL;						// Pointer to analog inputs shared memory

pthread_t 								ev314_thread_hp, 
													ev314_thread_lp;								// Thread descriptors
													
/* Thread shared variables */

struct ev314_state_struct ev314_state;

/*
 *	Profiling code
 */

#ifdef EV314_PROFILING_ON
struct timespec						profiling_start;

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
	
	fprintf( 	stderr, "** Profiling duration: %d us.\n",
						(int)( ( profiling_stop.tv_sec - profiling_start.tv_sec ) * 1000000
								 + ( profiling_stop.tv_nsec - profiling_start.tv_nsec ) / 1000 ) );
}
#endif

/*
 *	LCD functions
 */

/*
 * ev314_lcd_refresh: decompress screen buffer and copy it to the LCD shared memory
 * 
 */
void      ev314_lcd_refresh( void )
{
  UBYTE   *pSrc = ev314_lcd_buffer;
  UBYTE   *pDst = ev314_lcd_screen;
  ULONG   Pixels;
  UWORD   X;
  UWORD   Y;

	if ( pDst )	{

		for ( Y = 0; Y < 128; Y++ )
		{
			for ( X = 0; X < 7; X++ )
			{
				Pixels  =  (ULONG)*pSrc;
				pSrc++;
				Pixels |=  (ULONG)*pSrc << 8;
				pSrc++;
				Pixels |=  (ULONG)*pSrc << 16;
				pSrc++;

				*pDst   =  ev314_pixel_tab[Pixels & 0x07];
				pDst++;
				Pixels >>= 3;
				*pDst   =  ev314_pixel_tab[Pixels & 0x07];
				pDst++;
				Pixels >>= 3;
				*pDst   =  ev314_pixel_tab[Pixels & 0x07];
				pDst++;
				Pixels >>= 3;
				*pDst   =  ev314_pixel_tab[Pixels & 0x07];
				pDst++;
				Pixels >>= 3;
				*pDst   =  ev314_pixel_tab[Pixels & 0x07];
				pDst++;
				Pixels >>= 3;
				*pDst   =  ev314_pixel_tab[Pixels & 0x07];
				pDst++;
				Pixels >>= 3;
				*pDst   =  ev314_pixel_tab[Pixels & 0x07];
				pDst++;
				Pixels >>= 3;
				*pDst   =  ev314_pixel_tab[Pixels & 0x07];
				pDst++;
			}
			Pixels  =  (ULONG)*pSrc;
			pSrc++;
			Pixels |=  (ULONG)*pSrc << 8;
			pSrc++;

			*pDst   =  ev314_pixel_tab[Pixels & 0x07];
			pDst++;
			Pixels >>= 3;
			*pDst   =  ev314_pixel_tab[Pixels & 0x07];
			pDst++;
			Pixels >>= 3;
			*pDst   =  ev314_pixel_tab[Pixels & 0x07];
			pDst++;
			Pixels >>= 3;
			*pDst   =  ev314_pixel_tab[Pixels & 0x07];
			pDst++;
		}
	}
}

/*
 * 	ev314_clear_buffer:	fill screen buffer with white pixels
 * 
 */
void ev314_clear_buffer( void )	{

	memset( ev314_lcd_buffer, 0, EV314_LCD_BUFFER_SIZE );
}

/*
 * 	ev314_clear_screen:	fill screen with white pixels
 * 
 */
void ev314_clear_screen( void )	{

	memset( ev314_lcd_buffer, 0, EV314_LCD_BUFFER_SIZE );
	ev314_lcd_refresh( );
}


/*
 * 	LCD screen management routines.
 * 	Borrowed from Lego's "d_lcd.c" file.
 */

int ev314_lcd_init( void )	{
	struct fb_var_screeninfo 	var;
	struct fb_fix_screeninfo 	fix;

	/* Open LCD device */
	
	ev314_lcd_fd = open( LCD_DEVICE_NAME, O_RDWR );
	if ( ev314_lcd_fd < 0 )
		return -1;
	
	ioctl( ev314_lcd_fd, _IOW( 'S', 0, int ), NULL );
	
	/* Get dimensions of the screen from the framebuffer driver */
	
	ioctl( ev314_lcd_fd, FBIOGET_VSCREENINFO, &var );
  ioctl( ev314_lcd_fd, FBIOGET_FSCREENINFO, &fix );
  
  /* Check if we deal with the right hardware */
  
  if ( ( var.yres != EV314_LCD_SCREEN_SIZEY ) || ( fix.line_length != ( EV314_LCD_SCREEN_SIZEX / 8 ) ) )
		return -2;
	
	/* Map LCD memory to a user space UBYTE pointer */
	
	ev314_lcd_screen = (unsigned char *)mmap( 0, EV314_LCD_SCREEN_SIZE, PROT_WRITE | PROT_READ, MAP_SHARED, ev314_lcd_fd, 0 );
	if ( ev314_lcd_screen == MAP_FAILED )
		return -3;
	
	return 0;
}

/*
 * ev314_lcd_close: realease shared memory and close file descriptor.
 * 
 */
void ev314_lcd_close( void )
{
	
	if ( ev314_lcd_screen )	{
		munmap( (void*)ev314_lcd_screen, sizeof( ev314_lcd_screen ) );
		ev314_lcd_screen = NULL;
	}
	
	if ( ev314_lcd_fd )	{
		close( ev314_lcd_fd );
		ev314_lcd_fd = 0;
	}
}

/*
 * ev314_draw_pixel: draw one specific pixel.
 * 
 */
void ev314_draw_pixel( DATA8 Color, DATA16 X0, DATA16 Y0 )
{
	UBYTE *pImage = ev314_lcd_buffer;
	
  if ((X0 >= 0) && (X0 < EV314_LCD_BUFFER_SIZEX) && (Y0 >= 0) && (Y0 < EV314_LCD_BUFFER_SIZEY))
  {
    if (Color)
    {
			if ( (X0 >> 3) + Y0 * ((EV314_LCD_BUFFER_SIZEX + 7) >> 3) >= EV314_LCD_BUFFER_SIZE )
				fprintf( stderr, "** ev314_draw_pixel : buffer overflow. Internal error.\n" );
      pImage[(X0 >> 3) + Y0 * ((EV314_LCD_BUFFER_SIZEX + 7) >> 3)]  |=  (1 << (X0 % 8));
    }
    else
    {
			if ( (X0 >> 3) + Y0 * ((EV314_LCD_BUFFER_SIZEX + 7) >> 3) >= EV314_LCD_BUFFER_SIZE )
				fprintf( stderr, "** ev314_draw_pixel : buffer overflow. Internal error.\n" );
      pImage[(X0 >> 3) + Y0 * ((EV314_LCD_BUFFER_SIZEX + 7) >> 3)]  &= ~(1 << (X0 % 8));
    }
  }
}

/*
 * ev314_draw_logo: draw the logo
 * 
 */
void ev314_draw_logo( void )	{
	int 		i; 
	DATA16 	x = 0, y = 0;
	
	for ( i = 0; i < EV314_LOGO_SIZE; i++ )	{

		// Draw current pixel
		
		if ( ev314_logo[i] > EV314_LOGO_THRESHOLD )
			ev314_draw_pixel( 1, x, y );
		
		// Update pixel coordinates
		
		if ( ++x == EV314_LOGO_X )	{
			x = 0;
			y++;
		}
	}
}

/* 
 * Characters printing routines
 */
 
typedef   struct
{
  const   char    *pFontBits;           // Pointer to start of font bitmap
  const   DATA16  FontHeight;           // Character height (all inclusive)
  const   DATA16  FontWidth;            // Character width (all inclusive)
  const   DATA16  FontHorz;             // Number of horizontal character in font bitmap
  const   DATA8   FontFirst;            // First character supported
  const   DATA8   FontLast;             // Last character supported
}
FONTINFO;


#include  "normal_font.xbm"
#include  "small_font.xbm"
#include  "large_font.xbm"
#include  "tiny_font.xbm"


FONTINFO  FontInfo[] =
{
  [NORMAL_FONT] = {
                    .pFontBits    = (const char*)normal_font_bits,
                    .FontHeight   = 9,
                    .FontWidth    = 8,
                    .FontHorz     = 16,
                    .FontFirst    = 0x20,
                    .FontLast     = 0x7F
                  },
  [SMALL_FONT] =  {
                    .pFontBits    = (const char*)small_font_bits,
                    .FontHeight   = 8,
                    .FontWidth    = 8,
                    .FontHorz     = 16,
                    .FontFirst    = 0x20,
                    .FontLast     = 0x7F
                  },
  [LARGE_FONT] =  {
                    .pFontBits    = (const char*)large_font_bits,
                    .FontHeight   = 16,
                    .FontWidth    = 16,
                    .FontHorz     = 16,
                    .FontFirst    = 0x20,
                    .FontLast     = 0x7F
                  },
  [TINY_FONT] =   {
                    .pFontBits    = (const char*)tiny_font_bits,
                    .FontHeight   = 7,
                    .FontWidth    = 5,
                    .FontHorz     = 16,
                    .FontFirst    = 0x20,
                    .FontLast     = 0x7F
                  },

};


DATA16 ev314_get_font_width( DATA8 Font )
{
  return ( FontInfo[Font].FontWidth );
}


DATA16 ev314_get_font_height( DATA8 Font )
{
  return ( FontInfo[Font].FontHeight );
}


void ev314_draw_char( DATA8 Color, DATA16 X0, DATA16 Y0, DATA8 Font, DATA8 Char )
{
	UBYTE 	*pImage = ev314_lcd_buffer;
  DATA16  CharWidth;
  DATA16  CharHeight;
  DATA16  CharByteIndex;
  DATA16  LcdByteIndex;
  UBYTE   CharByte;
  DATA16  Tmp,X,Y,TmpX,MaxX;


  CharWidth   =  FontInfo[Font].FontWidth;
  CharHeight  =  FontInfo[Font].FontHeight;

  if ((Char >= FontInfo[Font].FontFirst) && (Char <= FontInfo[Font].FontLast))
  {
    Char -=  FontInfo[Font].FontFirst;

    CharByteIndex  =  (Char % FontInfo[Font].FontHorz) * ((CharWidth + 7) / 8);
    CharByteIndex +=  ((Char / FontInfo[Font].FontHorz) * ((CharWidth + 7) / 8) * CharHeight * FontInfo[Font].FontHorz);

    if (((CharWidth % 8) == 0) && ((X0 % 8) == 0))
    { // Font aligned

      X0             =  (X0 >> 3) << 3;
      LcdByteIndex   =  (X0 >> 3) + Y0 * ((EV314_LCD_BUFFER_SIZEX + 7) >> 3);

      if (Color)
      {
        while (CharHeight)
        {
          Tmp  =  0;
          do
          {
            if (LcdByteIndex < EV314_LCD_BUFFER_SIZE)
            {
							if ( LcdByteIndex + Tmp >= EV314_LCD_BUFFER_SIZE )
								fprintf( stderr, "** ev314_draw_char : buffer overflow. Internal error.\n" );
              pImage[LcdByteIndex + Tmp]  =  FontInfo[Font].pFontBits[CharByteIndex + Tmp];
            }
            Tmp++;
          }
          while (Tmp < (CharWidth / 8));

          CharByteIndex +=  (CharWidth * FontInfo[Font].FontHorz) / 8;
          LcdByteIndex  +=  ((EV314_LCD_BUFFER_SIZEX + 7) >> 3);
          CharHeight--;
        }
      }
      else
      {
        while (CharHeight)
        {
          Tmp  =  0;
          do
          {
            if (LcdByteIndex < EV314_LCD_BUFFER_SIZE)
            {
							if ( LcdByteIndex + Tmp >= EV314_LCD_BUFFER_SIZE )
								fprintf( stderr, "** ev314_draw_char : buffer overflow. Internal error.\n" );
              pImage[LcdByteIndex + Tmp]  = ~FontInfo[Font].pFontBits[CharByteIndex + Tmp];
            }
            Tmp++;
          }
          while (Tmp < (CharWidth / 8));

          CharByteIndex +=  (CharWidth * FontInfo[Font].FontHorz) / 8;
          LcdByteIndex  +=  ((EV314_LCD_BUFFER_SIZEX + 7) >> 3);
          CharHeight--;
        }
      }
    }
    else
    { // Font not aligned

      MaxX          =  X0 + CharWidth;

      if (Color)
      {
        for (Y = 0;Y < CharHeight;Y++)
        {
          TmpX              =  X0;

          for (X = 0;X < ((CharWidth + 7) / 8);X++)
          {
            CharByte  =  FontInfo[Font].pFontBits[CharByteIndex + X];

            for (Tmp = 0;(Tmp < 8) && (TmpX < MaxX);Tmp++)
            {
              if (CharByte & 1)
              {
                ev314_draw_pixel( 1, TmpX, Y0);
              }
              else
              {
                ev314_draw_pixel( 0, TmpX, Y0);
              }
              CharByte >>= 1;
              TmpX++;
            }
          }
          Y0++;
          CharByteIndex +=  ((CharWidth + 7) / 8) * FontInfo[Font].FontHorz;

        }
      }
      else
      {
        for (Y = 0;Y < CharHeight;Y++)
        {
          TmpX              =  X0;

          for (X = 0;X < ((CharWidth + 7) / 8);X++)
          {
            CharByte  =  FontInfo[Font].pFontBits[CharByteIndex + X];

            for (Tmp = 0;(Tmp < 8) && (TmpX < MaxX);Tmp++)
            {
              if (CharByte & 1)
              {
                ev314_draw_pixel( 0, TmpX, Y0 );
              }
              else
              {
                ev314_draw_pixel( 1, TmpX, Y0 );
              }
              CharByte >>= 1;
              TmpX++;
            }
          }
          Y0++;
          CharByteIndex +=  ((CharWidth + 7) / 8) * FontInfo[Font].FontHorz;
        }
      }
    }
  }
}


void ev314_draw_text( DATA8 Color, DATA16 X0, DATA16 Y0, DATA8 Font, DATA8 *pText )
{
	
  while (*pText)
  {
    if (X0 < (EV314_LCD_BUFFER_SIZEX - FontInfo[Font].FontWidth))
    {
      ev314_draw_char( Color, X0, Y0, Font, *pText);
      X0 +=  FontInfo[Font].FontWidth;
    }
    pText++;
  }
}

/*
 *	LED function
 */
void ev314_control_led( char state )	{
	static char current_state = EV314_LED_OFF;
	
	if ( state != current_state )	{
		
		/* Change state only if needed */
		
		ev314_led_command[0] = state;
		write( ev314_ui_fd, ev314_led_command, EV314_SIZEOF_LED_COMMAND );
		current_state = state;
	}
}
 
/*
 *	Motor control functions
 */

int ev314_motor_start( char motor_id )	{
	
	if ( ev314_motor_fd )	{
		
		switch( motor_id )	{
			case 0:
				ioctl( ev314_motor_fd, PWM_IOCTL_START_A, NULL );
				break;
			case 1:
				ioctl( ev314_motor_fd, PWM_IOCTL_START_B, NULL );
				break;
			case 2:
				ioctl( ev314_motor_fd, PWM_IOCTL_START_C, NULL );
				break;
			case 3:
				ioctl( ev314_motor_fd, PWM_IOCTL_START_D, NULL );
				break;
			default:
				return -2;
		}
	}
	else
	{
		return -1;
	}

	return 0;
}

int ev314_motor_stop( char motor_id, char mode )	{
	
	if ( ev314_motor_fd )	{
		
		switch( motor_id )	{
			case 0:
				if ( mode == EV314_MOTOR_BRAKE )
					ioctl( ev314_motor_fd, PWM_IOCTL_STOP_BR_A, NULL );
				if ( mode == EV314_MOTOR_FLOAT )
					ioctl( ev314_motor_fd, PWM_IOCTL_STOP_FL_A, NULL );
				break;
			case 1:
				if ( mode == EV314_MOTOR_BRAKE )
					ioctl( ev314_motor_fd, PWM_IOCTL_STOP_BR_B, NULL );
				if ( mode == EV314_MOTOR_FLOAT )
					ioctl( ev314_motor_fd, PWM_IOCTL_STOP_FL_B, NULL );
				break;
			case 2:
				if ( mode == EV314_MOTOR_BRAKE )
					ioctl( ev314_motor_fd, PWM_IOCTL_STOP_BR_C, NULL );
				if ( mode == EV314_MOTOR_FLOAT )
					ioctl( ev314_motor_fd, PWM_IOCTL_STOP_FL_C, NULL );
				break;
			case 3:
				if ( mode == EV314_MOTOR_BRAKE )
					ioctl( ev314_motor_fd, PWM_IOCTL_STOP_BR_D, NULL );
				if ( mode == EV314_MOTOR_FLOAT )
					ioctl( ev314_motor_fd, PWM_IOCTL_STOP_FL_D, NULL );
				break;
			default:
				return -2;
		}
	}
	else
	{
		return -1;
	}
	
	return 0;
}

int ev314_motor_voltage( char motor_id, int motor_voltage )	{
	
	SLONG casted_motor_voltage;
	
	casted_motor_voltage = (SLONG)motor_voltage;
	
	if ( ev314_motor_fd )	{
		
		switch( motor_id )	{
			case 0:
				ioctl( ev314_motor_fd, PWM_IOCTL_WRITE_PWM_A, &casted_motor_voltage );
				break;
			case 1:
				ioctl( ev314_motor_fd, PWM_IOCTL_WRITE_PWM_B, &casted_motor_voltage );
				break;
			case 2:
				ioctl( ev314_motor_fd, PWM_IOCTL_WRITE_PWM_C, &casted_motor_voltage );
				break;
			case 3:
				ioctl( ev314_motor_fd, PWM_IOCTL_WRITE_PWM_D, &casted_motor_voltage );
				break;
			default:
				return -2;
		}
	}
	else
	{
		return -1;
	}
	
	return 0;
}  

int ev314_reset_encoder( char motor_id )	{
	
	if ( ev314_motor_fd )	{
		
		switch( motor_id )	{
			case 0:
				ioctl( ev314_motor_fd, PWM_IOCTL_RESET_A, NULL );
				break;
			case 1:
				ioctl( ev314_motor_fd, PWM_IOCTL_RESET_B, NULL );
				break;
			case 2:
				ioctl( ev314_motor_fd, PWM_IOCTL_RESET_C, NULL );
				break;
			case 3:
				ioctl( ev314_motor_fd, PWM_IOCTL_RESET_D, NULL );
				break;
			default:
				return -2;
		}
	}
	else
	{
		return -1;
	}
		
	return 0;
}

int ev314_read_encoder( char motor_id )	{
	
	SLONG tachocnt;
	
	if ( ev314_motor_fd )	{
		
		switch( motor_id )	{
			case 0:
				ioctl( ev314_motor_fd, PWM_IOCTL_READ_ENC_A, &tachocnt );
				return (int)tachocnt;
			case 1:
				ioctl( ev314_motor_fd, PWM_IOCTL_READ_ENC_B, &tachocnt );
				return (int)tachocnt;
			case 2:
				ioctl( ev314_motor_fd, PWM_IOCTL_READ_ENC_C, &tachocnt );
				return (int)tachocnt;
			case 3:
				ioctl( ev314_motor_fd, PWM_IOCTL_READ_ENC_D, &tachocnt );
				return (int)tachocnt;
		}
	}
	else
	{
		return 0;
	}
		
	return 0;
}

/*
 * ev314_bat_voltage_filter : low-pass filtering of battery level.
 * 
 */
double ev314_bat_voltage_filter( double voltage )	{
	static char		init = 0;
	static double y1;
	
	if ( init )	{
		y1 = EV314_BAT_FILTER_POLE * y1 + ( 1.0 - EV314_BAT_FILTER_POLE ) * voltage;
	}
	else {
		y1 = voltage;
		init = 1;
		
	}
	return y1;
}

/*
 * ev314_thread_HP : high priority thread. USB management.
 * 
 */
void* ev314_thread_HP( )	{
	int 												my_policy;
	struct sched_param					my_param;
	int													i;
	int													ret;
	struct timespec							tmspc, passive_wait;
	struct ev314_control_struct *ev314_control;
	
	/* Thread initialization */
	
	ev314_control = (struct ev314_control_struct *)ev314_usb_buf;
	
	my_param.sched_priority = EV314_HIGH_PRIORITY;
	my_policy = EV314_SCHED_POLICY;
	if ( pthread_setschedparam( pthread_self( ), my_policy, &my_param ) )
		fprintf( stderr, "** ev314_thread_HP : unable to change thread scheduling parameters.\n" );
	
	passive_wait.tv_sec = 0;
	passive_wait.tv_nsec = EV314_HP_REFRESH_PERIOD * 1000;
		
	/* Main loop */
	
	while( 1 )	{
		
		/* Acquire current absolute time */
			
		clock_gettime( CLOCK_MONOTONIC, &tmspc );
		ev314_state.current_time = (unsigned long long)( tmspc.tv_sec * (unsigned long long)1000000000 )
														+ (unsigned long long)( tmspc.tv_nsec );
		
		/* Update encoder measurement */
		
		for ( i = 0; i < EV314_NB_MOTORS; i++ )
			ev314_state.motor_angle[i] = ev314_read_encoder( i );
			
		/* Update analog inputs measurements */
		
		for ( i = 0; i < EV314_NB_SENSORS; i++ )
			ev314_state.input_ADC[i]=ev314_analog->InPin1[i];
		
		/* Update battery current */
		
		ev314_state.battery_current = (int)ev314_analog->BatteryCurrent;
		
		/* Update battery voltage */
		
		ev314_state.battery_voltage = (unsigned int)ev314_bat_voltage_filter( (double)ev314_analog->Cell123456 * 2.0 );
		
		/* Check for a control packet */
		
		memset( &ev314_usb_buf, 0, sizeof( ev314_usb_buf ) );
		ret = read( ev314_usb_fd, ev314_usb_buf, EV314_MAX_USB_BUF_SIZE );
		
		if ( ret )	{
			if ( ret == -1 )	{
				fprintf( stderr, "** Warning: USB read error.\n" );
				ev314_state.error = EV314_ERROR_USB_READ;
			}
			else if ( ret != sizeof( struct ev314_control_struct ) )	{
				fprintf( stderr, "** Warning: error in USB packet format.\n" );
				ev314_state.error = EV314_ERROR_USB_PACKET_SIZE;
			}
			else if ( ev314_control->magic != EV314_MAGIC )	{
				fprintf( stderr, "** Warning: wrong USB magic number.\n" );
				ev314_state.error = EV314_ERROR_USB_MAGIC;
			}
			else {
				
				/* Command processing */
				
				switch ( ev314_control->cmd )	{
					case EV314_CMD_NO_CDM:
						break;
						
					case EV314_CMD_RESET_ENC:
						if ( 	( ev314_reset_encoder( EV314_MOTOR_PORT_A ) ) ||
									( ev314_reset_encoder( EV314_MOTOR_PORT_B ) ) ||
									( ev314_reset_encoder( EV314_MOTOR_PORT_C ) ) ||
									( ev314_reset_encoder( EV314_MOTOR_PORT_D ) ) )	{
							fprintf( stderr, "** Error while resetting encoders.\n" );
							ev314_state.error = EV314_ERROR_ENC_RESET;
						}
						
						for ( i = 0; i < EV314_NB_MOTORS; i++ )
							if ( ev314_read_encoder( i ) )	{
								fprintf( stderr, "** Reset failed on encoder %d (expected 0, got %d).\n", i, ev314_read_encoder( i ) );
								ev314_state.error = EV314_ERROR_ENC_RESET;
							}
						break;
						
					case EV314_CMD_HALT:
						pthread_cancel( ev314_thread_lp );	
						return NULL;
					
					case EV314_CMD_CONTROL:
					
						/* Handling saturations */
						
						for ( i = 0; i < EV314_NB_MOTORS; i++ )	{
							if ( ev314_control->motor_power[i] > EV314_MAX_CONTROL )
								ev314_control->motor_power[i] = EV314_MAX_CONTROL;
							if ( ev314_control->motor_power[i] < -EV314_MAX_CONTROL )
								ev314_control->motor_power[i] = -EV314_MAX_CONTROL;
						}
						
						if ( 	( ev314_motor_voltage( EV314_MOTOR_PORT_A, ev314_control->motor_power[0] ) ) ||
									( ev314_motor_voltage( EV314_MOTOR_PORT_B, ev314_control->motor_power[1] ) ) ||
									( ev314_motor_voltage( EV314_MOTOR_PORT_C, ev314_control->motor_power[2] ) ) ||
									( ev314_motor_voltage( EV314_MOTOR_PORT_D, ev314_control->motor_power[3] ) ) )	{
							fprintf( stderr, "** Error while setting motor voltage.\n" );
							ev314_state.error = EV314_ERROR_SET_VOLTAGE;
						}
						else {
							
							/* Update current control timestamp for the watchdog */
				
							ev314_state.control_time_stamp = ev314_state.current_time;
							
							/* Updating current control values in state struct */
							
							for ( i = 0; i < EV314_NB_MOTORS; i++ )
								ev314_state.motor_power[i] = ev314_control->motor_power[i];
								
							/* Send state through USB */
				
							ret = write( ev314_usb_fd, (void*)&ev314_state, sizeof( struct ev314_state_struct ) );
					
							if ( ret != sizeof( struct ev314_state_struct ) )	{
								fprintf( stderr, "** Warning: USB write error.\n" );
								ev314_state.error = EV314_ERROR_USB_WRITE;
							}
						}
						break;
					
					default:
						fprintf( stderr, "** Warning: unknown command received.\n" );
						ev314_state.error = EV314_ERROR_UNKNOWN_CMD;
				}
			}
		}
			
		/* Watchdog management */
		
		if ( ev314_state.current_time - ev314_state.control_time_stamp > EV314_WATCHDOG_PERIOD )	{
			
			if ( ev314_state.watchdog == EV314_WATCHDOG_OFF )	{
				
				/* Set control signals to 0V */
				
				if ( 	( ev314_motor_voltage( EV314_MOTOR_PORT_A, 0 ) ) ||
							( ev314_motor_voltage( EV314_MOTOR_PORT_B, 0 ) ) ||
							( ev314_motor_voltage( EV314_MOTOR_PORT_C, 0 ) ) ||
							( ev314_motor_voltage( EV314_MOTOR_PORT_D, 0 ) ) )	{
					fprintf( stderr, "** Error while setting motor voltage.\n" );
					ev314_state.error = EV314_ERROR_SET_VOLTAGE;
				}
				
				/* Stop the motors */
				
				if ( ev314_state.manual_release == EV314_MANUAL_RELEASE_ON )	{
					
					/* Manual motor release requested: floating mode */
					
					if ( 	( ev314_motor_stop( EV314_MOTOR_PORT_A, EV314_MOTOR_FLOAT ) ) ||
								( ev314_motor_stop( EV314_MOTOR_PORT_B, EV314_MOTOR_FLOAT ) ) ||
								( ev314_motor_stop( EV314_MOTOR_PORT_C, EV314_MOTOR_FLOAT ) ) ||
								( ev314_motor_stop( EV314_MOTOR_PORT_D, EV314_MOTOR_FLOAT ) ) )	{
						fprintf( stderr, "** Error while stopping motors.\n" );
						ev314_state.error = EV314_ERROR_STOP_MOTOR;
					}
				}
				else {
					
					/* Normal watchdog stop: braking mode */
					
					if ( 	( ev314_motor_stop( EV314_MOTOR_PORT_A, EV314_MOTOR_BRAKE ) ) ||
								( ev314_motor_stop( EV314_MOTOR_PORT_B, EV314_MOTOR_BRAKE ) ) ||
								( ev314_motor_stop( EV314_MOTOR_PORT_C, EV314_MOTOR_BRAKE ) ) ||
								( ev314_motor_stop( EV314_MOTOR_PORT_D, EV314_MOTOR_BRAKE ) ) )	{
						fprintf( stderr, "** Error while stopping motors.\n" );
						ev314_state.error = EV314_ERROR_STOP_MOTOR;
					}
				}
				
				/* Updating current control values in state struct */
						
				for ( i = 0; i < EV314_NB_MOTORS; i++ )
					ev314_state.motor_power[i] = 0;
							
				/* Update watchdog state */
				
				ev314_state.watchdog = EV314_WATCHDOG_ON;
			}
		}
		else {
			
			if ( ev314_state.watchdog == EV314_WATCHDOG_ON )	{
				
				/* Starting motors */
		
				if ( 	( ev314_motor_start( EV314_MOTOR_PORT_A ) ) ||
							( ev314_motor_start( EV314_MOTOR_PORT_B ) ) ||
							( ev314_motor_start( EV314_MOTOR_PORT_C ) ) ||
							( ev314_motor_start( EV314_MOTOR_PORT_D ) ) )	{
					fprintf( stderr, "** Error while starting motors.\n" );
					ev314_state.error = EV314_ERROR_START_MOTOR;
				}
				
				/* Updating watchdog state */
				
				ev314_state.watchdog = EV314_WATCHDOG_OFF;
			}
		}
		
		/* Monitor battery voltage. Shutdown if too low */

		if ( ev314_state.battery_voltage < EV314_CRIT_HALT_VOLTAGE )
			break;
		
		/* Passive wait */
		
		#ifdef EV314_PROFILING_ON
		ev314_profiling_start( );
		#endif
		
		nanosleep( &passive_wait, NULL );
		
		#ifdef EV314_PROFILING_ON
		ev314_profiling_stop( );
		#endif
	}
	
	pthread_cancel( ev314_thread_lp );
	
	return NULL;
}

/*
 * ev314_thread_LP : low priority thread. UI management.
 * 
 */
void* ev314_thread_LP( )	{
	int 		my_policy;
	struct 	sched_param my_param;
	char		led_state;
	#ifdef EV314_DISPLAY_ON
	DATA8		buf[EV314_LCD_NB_CHAR_X+1];
	DATA16	x, y, voltage;
	#endif
	
	/* Thread initialization */
	
	my_param.sched_priority = EV314_LOW_PRIORITY;
	my_policy = EV314_SCHED_POLICY;
	if ( pthread_setschedparam( pthread_self( ), my_policy, &my_param ) )
		fprintf( stderr, "** ev314_thread_LP : unable to change thread scheduling parameters.\n" );
		
	/* Main loop */
	
	while( 1 )	{
		
		#ifdef EV314_DISPLAY_ON
		
		/* Manage display */
		
		ev314_clear_buffer( );
		snprintf( (char*)buf, EV314_LCD_NB_CHAR_X+1, "                      " );
		ev314_draw_text( 0, 0, 0, NORMAL_FONT, buf );
		snprintf( (char*)buf, EV314_LCD_NB_CHAR_X+1, "                      " );
		ev314_draw_text( 0, 0, 8, NORMAL_FONT, buf );
		snprintf( (char*)buf, EV314_LCD_NB_CHAR_X+1, "                      " );
		ev314_draw_text( 0, 0, 16, NORMAL_FONT, buf );
		
		snprintf( (char*)buf, EV314_LCD_NB_CHAR_X+1, "======= EV3.14 =======" );
		ev314_draw_text( 0, 0, 2, NORMAL_FONT, buf );
		snprintf( (char*)buf, EV314_LCD_NB_CHAR_X+1, " ENCODERS/STATES    SENSORS/CONTROL" );
		ev314_draw_text( 0, 0, 15, TINY_FONT, buf );
		snprintf( (char*)buf, EV314_LCD_NB_CHAR_X+1, "A:          1:" );
		ev314_draw_text( 1, 0, 26, NORMAL_FONT, buf );
		snprintf( (char*)buf, EV314_LCD_NB_CHAR_X+1, "B:          2:" );
		ev314_draw_text( 1, 0, 36, NORMAL_FONT, buf );
		snprintf( (char*)buf, EV314_LCD_NB_CHAR_X+1, "C:          3:" );
		ev314_draw_text( 1, 0, 46, NORMAL_FONT, buf );
		snprintf( (char*)buf, EV314_LCD_NB_CHAR_X+1, "D:          4:" );
		ev314_draw_text( 1, 0, 56, NORMAL_FONT, buf );
		
		// Print encoder values
		snprintf( (char*)buf, EV314_LCD_NB_CHAR_X, "%d", ev314_state.motor_angle[0] );
		ev314_draw_text( 1, 16, 26, NORMAL_FONT, buf );
		snprintf( (char*)buf, EV314_LCD_NB_CHAR_X, "%d", ev314_state.motor_angle[1] );
		ev314_draw_text( 1, 16, 36, NORMAL_FONT, buf );
		snprintf( (char*)buf, EV314_LCD_NB_CHAR_X, "%d", ev314_state.motor_angle[2] );
		ev314_draw_text( 1, 16, 46, NORMAL_FONT, buf );
		snprintf( (char*)buf, EV314_LCD_NB_CHAR_X, "%d", ev314_state.motor_angle[3] );
		ev314_draw_text( 1, 16, 56, NORMAL_FONT, buf );
		
		// Print sensor values
		snprintf( (char*)buf, EV314_LCD_NB_CHAR_X, "%d", ev314_state.input_ADC[0] );
		ev314_draw_text( 1, 112, 26, NORMAL_FONT, buf );
		snprintf( (char*)buf, EV314_LCD_NB_CHAR_X, "%d", ev314_state.input_ADC[1] );
		ev314_draw_text( 1, 112, 36, NORMAL_FONT, buf );
		snprintf( (char*)buf, EV314_LCD_NB_CHAR_X, "%d", ev314_state.input_ADC[2] );
		ev314_draw_text( 1, 112, 46, NORMAL_FONT, buf );
		snprintf( (char*)buf, EV314_LCD_NB_CHAR_X, "%d", ev314_state.input_ADC[3] );
		ev314_draw_text( 1, 112, 56, NORMAL_FONT, buf );
		
		// Display battery state
		snprintf( (char*)buf, EV314_LCD_NB_CHAR_X+1, "    " );
		ev314_draw_text( 0, 0, 67, NORMAL_FONT, buf );
		snprintf( (char*)buf, EV314_LCD_NB_CHAR_X+1, "BAT:" );
		ev314_draw_text( 0, 6, 68, TINY_FONT, buf );
		for ( x = 32; x < 176; x++ )	{
			ev314_draw_pixel( 1, x, 67 );
			ev314_draw_pixel( 1, x, 75 );
		}
		for ( y = 67; y < 76; y++ )	{
			ev314_draw_pixel( 1, 32, y );
			ev314_draw_pixel( 1, 175, y );
		}
		
		voltage =	( ( ev314_state.battery_voltage - EV314_MIN_BAT_VOLTAGE ) * 140 ) / 
							( EV314_MAX_BAT_VOLTAGE - EV314_MIN_BAT_VOLTAGE );
		if ( voltage > 140 )
			voltage = 140;
		if ( voltage < 0  )
			voltage = 0;
			
		for ( y = 69; y < 74; y++ )
			for ( x = 34; x < 34  + voltage; x++ )
				ev314_draw_pixel( 1, x, y );
		
		// Display state
		snprintf( (char*)buf, EV314_LCD_NB_CHAR_X+1, "WD:         U1:" );
		ev314_draw_text( 1, 0, 77, NORMAL_FONT, buf );
		snprintf( (char*)buf, EV314_LCD_NB_CHAR_X+1, "EE:         U2:" );
		ev314_draw_text( 1, 0, 87, NORMAL_FONT, buf );
		snprintf( (char*)buf, EV314_LCD_NB_CHAR_X+1, "TC:         U3:" );
		ev314_draw_text( 1, 0, 97, NORMAL_FONT, buf );
		snprintf( (char*)buf, EV314_LCD_NB_CHAR_X+1, "TS:         U4:" );
		ev314_draw_text( 1, 0, 107, NORMAL_FONT, buf );

		snprintf( (char*)buf, EV314_LCD_NB_CHAR_X+1, "                      " );
		ev314_draw_text( 0, 0, 117, NORMAL_FONT, buf );
		
		snprintf( (char*)buf, EV314_LCD_NB_CHAR_X, "%d", ev314_state.motor_power[0] );
		ev314_draw_text( 1, 120, 77, NORMAL_FONT, buf );
		snprintf( (char*)buf, EV314_LCD_NB_CHAR_X, "%d", ev314_state.motor_power[1] );
		ev314_draw_text( 1, 120, 87, NORMAL_FONT, buf );
		snprintf( (char*)buf, EV314_LCD_NB_CHAR_X, "%d", ev314_state.motor_power[2] );
		ev314_draw_text( 1, 120, 97, NORMAL_FONT, buf );
		snprintf( (char*)buf, EV314_LCD_NB_CHAR_X, "%d", ev314_state.motor_power[3] );
		ev314_draw_text( 1, 120, 107, NORMAL_FONT, buf );
		
		if ( ev314_state.watchdog == EV314_WATCHDOG_ON )
			snprintf( (char*)buf, EV314_LCD_NB_CHAR_X+1, "ON" );
		else
			snprintf( (char*)buf, EV314_LCD_NB_CHAR_X+1, "OFF" );
		ev314_draw_text( 1, 24, 77, NORMAL_FONT, buf );
		
		snprintf( (char*)buf, EV314_LCD_NB_CHAR_X+1, "%d", ev314_state.error );
		ev314_draw_text( 1, 24, 87, NORMAL_FONT, buf );
		
		snprintf( (char*)buf, EV314_LCD_NB_CHAR_X+1, "%d", (int)( ev314_state.current_time / 1000000000 ) );
		ev314_draw_text( 1, 24, 97, NORMAL_FONT, buf );
		
		snprintf( (char*)buf, EV314_LCD_NB_CHAR_X+1, "%d", (int)( ev314_state.control_time_stamp / 1000000000 ) );
		ev314_draw_text( 1, 24, 107, NORMAL_FONT, buf );
		
		snprintf( (char*)buf, EV314_LCD_NB_CHAR_X+1, "BRICK ID: %s", ev314_state.serial );
		ev314_draw_text( 0, 33, 118, TINY_FONT, buf );
		
		// Refresh screen
		ev314_lcd_refresh( );
		
		#endif
		
		/* Manage buttons */
		
		if ( ev314_buttons->Pressed[BACK_BUTTON - 1] )
			break;
		
		if ( ev314_buttons->Pressed[ENTER_BUTTON - 1] )	{
			
			/* Manage manual motor release: toggle state */
			
			if ( ev314_state.manual_release == EV314_MANUAL_RELEASE_OFF )
				ev314_state.manual_release = EV314_MANUAL_RELEASE_ON;
			else
				ev314_state.manual_release = EV314_MANUAL_RELEASE_OFF;
			
			/* Trigger the whatchdog */
			
			ev314_state.watchdog = EV314_WATCHDOG_OFF;
			
			/* Give time to release the button */
			
			usleep( EV314_LCD_REFRESH_PERIOD );
		}
		
		/* Manage LED */
			
		/* Default state: green static LED */
	
		led_state = EV314_LED_GREEN_STATIC;
		
		if ( ev314_state.manual_release == EV314_MANUAL_RELEASE_ON )	{
			
			/* Manual release activated: orange static LED */
	
			led_state = EV314_LED_ORANGE_STATIC;
		}
		
		if ( ev314_state.watchdog == EV314_WATCHDOG_OFF )	{
			
			/* Watchdog off: green or orange flashing LED */
			
			if ( ev314_state.manual_release == EV314_MANUAL_RELEASE_ON )
				led_state = EV314_LED_ORANGE_FLASH;
			else
				led_state = EV314_LED_GREEN_FLASH;
		}
		
		if ( ev314_state.battery_voltage < EV314_WARN_BAT_VOLTAGE )	{
			
			/* Bat voltage under warning level: orange pulsing LED */
	
			led_state = EV314_LED_ORANGE_PULSE;
		}
		
		if ( ev314_state.battery_voltage < EV314_CRIT_BAT_VOLTAGE )	{
			
			/* Bat voltage under critical level: red pulsing LED */
	
			led_state = EV314_LED_RED_PULSE;
		}
		
		ev314_control_led( led_state );
		
		/* Passive wait */
		
		usleep( EV314_LCD_REFRESH_PERIOD );
	}
	
	pthread_cancel( ev314_thread_hp );
	
	return NULL;
}
	
int main( void )	{
	int 		i, ret;
	DATA8		buf[EV314_LCD_NB_CHAR_X+1];
	char    Buf[INPUTS];
	int			id_fd = 0;
	
	/* Initialize state strucure */
	
	memset( &ev314_state, 0, sizeof( struct ev314_state_struct ) );
	ev314_state.magic = EV314_MAGIC;
	
	/* Open USB device */
	
	if ( ( ev314_usb_fd = open( USBDEV_DEVICE_NAME, O_RDWR | O_NONBLOCK | O_ASYNC ) ) == -1 )	{
		fprintf( stderr, "** Error %d during control panel initialization.\n", ev314_usb_fd );
		exit( -1 );
		}
		
	read( ev314_usb_fd, ev314_usb_buf, EV314_MAX_USB_BUF_SIZE );
	
	/* Read hardware ID */
	
	if ( ( id_fd = open( EV314_HW_ID_FILE, O_RDONLY ) ) == -1 )	{
		fprintf( stderr, "** Error %d while trying to open hardware ID file.\n", id_fd );
		exit( -2 );
	}
	else {
		if ( read( id_fd, ev314_state.serial, EV314_LENGTH_SERIAL ) != EV314_LENGTH_SERIAL )	{
			fprintf( stderr, "** Error while reading hardware ID.\n" );
			exit( -2 );
		}
		
		close( id_fd );
	}
	
	/* Initialize control panel access */
	
	if( ( ev314_ui_fd = open( UI_DEVICE_NAME, O_RDWR | O_SYNC ) ) == -1 )	{
		fprintf( stderr, "** Error %d during control panel initialization.\n", ev314_ui_fd );
		exit( -3 );
	}
	
	ev314_buttons = (UI*)mmap( 0, sizeof(UI), PROT_READ | PROT_WRITE, MAP_FILE | MAP_SHARED, ev314_ui_fd, 0 );
	if ( ev314_buttons == MAP_FAILED )
	{
		fprintf( stderr, "** Mapping failed during control panel initialization.\n" );
		exit( -4 );
	}
	
	/* Initializing LCD access */

	if ( ( ret = ev314_lcd_init( ) ) )	{
		fprintf( stderr, "** Error %d during LCD initialization.\n", ret );
		exit( -5 );
	}
	
	/* Switch off LED */
	
	ev314_led_command[0] = EV314_LED_OFF;
	write( ev314_ui_fd, ev314_led_command, EV314_SIZEOF_LED_COMMAND );
	
	/* Clear screen */
	
	ev314_clear_screen( );
	
	/* Draw logo */
	
	ev314_draw_logo( );
	ev314_lcd_refresh( );
	usleep( EV314_SPLASH_TIME );
	ev314_clear_screen( );
	
	/* Initializing analog input access */
	
	if( ( ev314_analog_fd = open( ANALOG_DEVICE_NAME, O_RDWR | O_SYNC ) ) == -1 )	{
		fprintf( stderr, "** Error %d during analog sensors initialization.\n", ev314_analog_fd );
		exit( -6 );
	}		

	ev314_analog = (ANALOG*)mmap( 0, sizeof(ANALOG), PROT_READ | PROT_WRITE, MAP_FILE | MAP_SHARED, ev314_analog_fd, 0) ;
	if ( ev314_analog == MAP_FAILED )
	{
		fprintf( stderr, "** Mapping failed during analog sensors initialization.\n" );
		exit( -7 );
	}
	
	/* Initializing DCM input access */
	
	if( ( ev314_DCM_fd = open( DCM_DEVICE_NAME, O_RDWR | O_SYNC ) ) == -1 )	{
		fprintf( stderr, "** Error %d during DCM access initialization.\n", ev314_DCM_fd );
		exit( -8 );
	}	
	
	/* Initialize motor access */
	
	ev314_motor_fd = open( PWM_DEVICE_NAME, O_WRONLY | O_SYNC );
	if ( ev314_motor_fd < 0 )	{
		fprintf( stderr, "** Error %d during motor initialization.\n", ev314_motor_fd );
		exit( -9 );
	}
	
	/* Initialize encoder access */
	
	ev314_encoder_fd = open( MOTOR_DEVICE_NAME, O_RDWR | O_SYNC );  
	if ( ev314_encoder_fd < 0 )	{
		fprintf( stderr, "** Error %d during encoder initialization.\n", ev314_encoder_fd );
		exit( -10 );
	}
	
	ev314_motor_data = (MOTORDATA*)mmap( 0, sizeof(MOTORDATA) * vmOUTPUTS, PROT_READ | PROT_WRITE, MAP_FILE | MAP_SHARED, ev314_encoder_fd, 0 );
	if ( ev314_motor_data == MAP_FAILED )
	{
		fprintf( stderr, "** Mapping failed during encoder initialization.\n" );
		exit( -11 );
	}
	
	/* Resetting encoders */
	
	if ( 	( ev314_reset_encoder( EV314_MOTOR_PORT_A ) ) ||
				( ev314_reset_encoder( EV314_MOTOR_PORT_B ) ) ||
				( ev314_reset_encoder( EV314_MOTOR_PORT_C ) ) ||
				( ev314_reset_encoder( EV314_MOTOR_PORT_D ) ) )	{
		fprintf( stderr, "** Error while resetting encoders.\n" );
		exit( -12 );
	}
	
	/* Setting motor voltage to 0 */
	
	if ( 	( ev314_motor_voltage( EV314_MOTOR_PORT_A, 0 ) ) ||
				( ev314_motor_voltage( EV314_MOTOR_PORT_B, 0 ) ) ||
				( ev314_motor_voltage( EV314_MOTOR_PORT_C, 0 ) ) ||
				( ev314_motor_voltage( EV314_MOTOR_PORT_D, 0 ) ) )	{
		fprintf( stderr, "** Error while setting motor voltage.\n" );
		exit( -13 );
	}
	
	/* Stopping motors, breaking mode */
	
	if ( 	( ev314_motor_stop( EV314_MOTOR_PORT_A, EV314_MOTOR_BRAKE ) ) ||
				( ev314_motor_stop( EV314_MOTOR_PORT_B, EV314_MOTOR_BRAKE ) ) ||
				( ev314_motor_stop( EV314_MOTOR_PORT_C, EV314_MOTOR_BRAKE ) ) ||
				( ev314_motor_stop( EV314_MOTOR_PORT_D, EV314_MOTOR_BRAKE ) ) )	{
		fprintf( stderr, "** Error while stopping motors.\n" );
		exit( -14 );
	}
	
	/* Set PIN5 of all imputs in high level mode */
	
	for ( i = 0; i < INPUTS; i++ )
		Buf[i] = EV314_NXT_REFLECT;
		
	write( ev314_DCM_fd, Buf, INPUTS );
	
	/* Starting threads */
	
	pthread_create( &ev314_thread_hp, NULL, ev314_thread_HP, NULL );
	pthread_create( &ev314_thread_lp, NULL, ev314_thread_LP, NULL );
	
	/* Waiting for the threads to finish */
	
	pthread_join( ev314_thread_hp, NULL );
	pthread_join( ev314_thread_lp, NULL );
	
	/* Static red LED */
	
	ev314_led_command[0] = EV314_LED_RED_STATIC;
	write( ev314_ui_fd, ev314_led_command, EV314_SIZEOF_LED_COMMAND );
	
	/* Setting motor voltage to 0 */
	
	if ( 	( ev314_motor_voltage( EV314_MOTOR_PORT_A, 0 ) ) ||
				( ev314_motor_voltage( EV314_MOTOR_PORT_B, 0 ) ) ||
				( ev314_motor_voltage( EV314_MOTOR_PORT_C, 0 ) ) ||
				( ev314_motor_voltage( EV314_MOTOR_PORT_D, 0 ) ) )	{
		fprintf( stderr, "** Error while setting motor voltage.\n" );
		exit( -15 );
	}
	
	/* Stopping motors */
	
	if ( 	( ev314_motor_stop( EV314_MOTOR_PORT_A, EV314_MOTOR_FLOAT ) ) ||
				( ev314_motor_stop( EV314_MOTOR_PORT_B, EV314_MOTOR_FLOAT ) ) ||
				( ev314_motor_stop( EV314_MOTOR_PORT_C, EV314_MOTOR_FLOAT ) ) ||
				( ev314_motor_stop( EV314_MOTOR_PORT_D, EV314_MOTOR_FLOAT ) ) )	{
		fprintf( stderr, "** Error while stopping motors.\n" );
		exit( -16 );
	}
	
	/* Closing encoder access */
	
	if ( ev314_motor_data )	{
		munmap( (void*)ev314_motor_data, sizeof(MOTORDATA) * vmOUTPUTS );
		ev314_motor_data = NULL;
	}
	
	if ( ev314_encoder_fd )	{
		close( ev314_encoder_fd );
		ev314_encoder_fd = 0;
	}
	
	/* Closing motor access */
	
	if ( ev314_motor_fd )	{
		close( ev314_motor_fd );
		ev314_motor_fd = 0;
	}
	
	/* Set PIN5 of all imputs in low level mode */
	
	for ( i = 0; i < INPUTS; i++ )
		Buf[i] = EV314_NXT_AMBIENT;
		
	write( ev314_DCM_fd, Buf, INPUTS );
	
	/* Closing access to DCM */
	
	if ( ev314_DCM_fd )	{
		close( ev314_DCM_fd );
		ev314_DCM_fd = 0;
	}
	
	/* Closing access to analog sensors */
	
		if ( ev314_analog )	{
		munmap( (void*)ev314_analog, sizeof(UI) );
		ev314_analog = NULL;
	}
	
	if ( ev314_analog_fd )	{
		close( ev314_analog_fd );
		ev314_analog_fd = 0;
	}
	
	/* Clear screen */
	
	ev314_clear_screen( );
	
	/* Print shutdown message */
	
	snprintf( (char*)buf, EV314_LCD_NB_CHAR_X+1, "                      " );
	ev314_draw_text( 0, 0, 55, NORMAL_FONT, buf );
	snprintf( (char*)buf, EV314_LCD_NB_CHAR_X+1, "                      " );
	ev314_draw_text( 0, 0, 64, NORMAL_FONT, buf );
	snprintf( (char*)buf, EV314_LCD_NB_CHAR_X+1, "SHUTDOWN IN PROGRESS.." );
	ev314_draw_text( 0, 0, 60, NORMAL_FONT, buf );
	ev314_lcd_refresh( );
	
	/* Closing LCD access */
	
	ev314_lcd_close( );
	
	/* Closing button access */
	
	if ( ev314_buttons )	{
		munmap( (void*)ev314_buttons, sizeof(UI) );
		ev314_buttons = NULL;
	}
	
	if ( ev314_ui_fd )	{
		close( ev314_ui_fd );
		ev314_ui_fd = 0;
	}
	
	/* Closing USB access */
	
	if ( ev314_usb_fd )	{
		close( ev314_usb_fd );
		ev314_usb_fd = 0;
	}
	
	/* Shutdown */
	
	#ifdef EV314_HALT_ON_EXIT
	
	ret = system( "halt" );
	if ( ret < 0 )	{
		fprintf( stderr, "** Error while trying to shutdown system.\n" );
		exit( -17 );
	}
	#endif
	
	return 0;
}

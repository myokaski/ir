/* Name: main.c
 * Project: hid-data, example how to use HID for data transfer
 * Author: Christian Starkjohann
 * Creation Date: 2008-04-11
 * Tabsize: 4
 * Copyright: (c) 2008 by OBJECTIVE DEVELOPMENT Software GmbH
 * License: GNU GPL v2 (see License.txt), GNU GPL v3 or proprietary (CommercialLicense.txt)
 */

/*
This example should run on most AVRs with only little changes. No special
hardware resources except INT1 are used. You may have to change usbconfig.h for
different I/O pins for USB. Please note that USB D+ must be the INT1 pin, or
at least be connected to INT1 as well.
*/

#include <avr/io.h>

#include <avr/wdt.h>
#include <avr/interrupt.h>  /* for sei() */
#include <util/delay.h>     /* for _delay_ms() */
#include <avr/eeprom.h>
#include <avr/pgmspace.h>   /* required by usbdrv.h */
#include "usbdrv.h"


#define IRLED_PORT PORTB
#define IRLED_DDR  DDRB
#define IRLED_OUT  PB1

#define IRREC_PORT PORTD
#define IRREC_DDR  DDRD
#define IRREC_PIN  PIND
#define IRREC_IN   PD3

#if defined(__AVR_ATmega16__)
#  define PWMDDR     DDRD
#  define PWMOUT     PD5
#elif defined(__AVR_ATmega8__) || defined(__AVR_ATmega48__) ||\
	defined(__AVR_ATmega88__) || defined(__AVR_ATmega168__) 
#  define PWMDDR     DDRB
#  define PWMOUT     PB1
#elif defined(__AVR_ATtiny2313__)
#  define PWMDDR     DDRB
#  define PWMOUT     PB3
#  define HAVE_ADC   0
#  define USART_RXC_vect USART_RX_vect
#  define MCUCSR     MCUSR
#else
#  error "Unsupported MCU type"
#endif

#if defined(__AVR_ATmega48__) || defined(__AVR_ATmega88__) ||\
	defined(__AVR_ATmega168__)
/* map ATmega8/16 names to ATmegaX8 names */
#  define USART_RXC_vect USART_RX_vect
#  define UDR     UDR0
#  define UCSRA   UCSR0A
#  define UCSRB   UCSR0B
#  define FE      FE0
#  define TXEN    TXEN0
#  define RXEN    RXEN0
#  define RXCIE   RXCIE0
#  define UDRE    UDRE0
#  define U2X     U2X0
#  define UBRRL   UBRR0L

#  define TIMSK   TIMSK1
#  define MCUCSR  MCUSR
#endif

#define IRLED_FREQ 38000 // must be  with 12MHz CPU and prescaler 1
#define TMR1_OCR F_CPU/2/IRLED_FREQ - 1

/*
* Bits that are set inside interrupt routines, and watched outside in
* the program's main loop.
*/
volatile struct
{
	uint8_t tmr_int: 1;
	uint8_t rx_int: 1;
	uint8_t int1_int: 1;
	uint8_t irtx_int: 1;
}
intflags;

volatile enum
{
	MODE_WAIT_RECIEVE,
	MODE_RECIEVING,
	MODE_TRANSMIT,
	MODE_IDLE
} __attribute__((packed)) mode = MODE_IDLE;

#define IRMAX 200
volatile uint16_t ovfarr[IRMAX];
volatile int8_t arr[IRMAX];
volatile int16_t iarr = 0;
volatile uint16_t ovf_count=0;

ISR(TIMER1_COMPA_vect)
{
	if(mode == MODE_RECIEVING)
	{
		if(++ovf_count == 0)
		{
			ovfarr[iarr] = 0;
			intflags.int1_int = 1;
		}
	}
	if(mode == MODE_TRANSMIT)
	{
		if(ovf_count==0)
		{
			++iarr;
			if(0 == (ovf_count = ovfarr[iarr]))
			{
				TCCR1A &= ~_BV(COM1A0);
				mode = MODE_IDLE;
				intflags.irtx_int = 1;
			}
			else
			{
				if(arr[iarr] == 0)
					TCCR1A |= _BV(COM1A0);
				else 
					TCCR1A &= ~_BV(COM1A0);
			}
		}
		--ovf_count;
	}
	
}

ISR(INT1_vect)
{
	if(mode == MODE_WAIT_RECIEVE)
	{
		ovf_count=1;
		mode = MODE_RECIEVING;
	}
	if(mode == MODE_RECIEVING)
	{
		if(iarr<(IRMAX-1))
		{
			arr[iarr]=bit_is_clear(IRREC_PIN, IRREC_IN)?1:0;
			ovfarr[iarr]=ovf_count;
			ovf_count=0;
			iarr++;
		}
		if(iarr == (IRMAX-1))
		{
			intflags.int1_int = 1;
		}
	}
}


/* ------------------------------------------------------------------------- */
/* ----------------------------- USB interface ----------------------------- */
/* ------------------------------------------------------------------------- */

PROGMEM const char usbHidReportDescriptor[22] = {    /* USB report descriptor */
    0x06, 0x00, 0xff,              // USAGE_PAGE (Generic Desktop)
    0x09, 0x01,                    // USAGE (Vendor Usage 1)
    0xa1, 0x01,                    // COLLECTION (Application)
    0x15, 0x00,                    //   LOGICAL_MINIMUM (0)
    0x26, 0xff, 0x00,              //   LOGICAL_MAXIMUM (255)
    0x75, 0x08,                    //   REPORT_SIZE (8)
    0x95, 0x80,                    //   REPORT_COUNT (128)
    0x09, 0x00,                    //   USAGE (Undefined)
    0xb2, 0x02, 0x01,              //   FEATURE (Data,Var,Abs,Buf)
    0xc0                           // END_COLLECTION
};
/* Since we define only one feature report, we don't use report-IDs (which
 * would be the first byte of the report). The entire report consists of 128
 * opaque data bytes.
 */

/* The following variables store the status of the current data transfer */
static uchar    currentAddress;
static uchar    bytesRemaining;

/* ------------------------------------------------------------------------- */

/* usbFunctionRead() is called when the host requests a chunk of data from
 * the device. For more information see the documentation in usbdrv/usbdrv.h.
 */
uchar   usbFunctionRead(uchar *data, uchar len)
{
    if(len > bytesRemaining)
        len = bytesRemaining;
    eeprom_read_block(data, (uchar *)0 + currentAddress, len);
    currentAddress += len;
    bytesRemaining -= len;
    return len;
}

/* usbFunctionWrite() is called when the host sends a chunk of data to the
 * device. For more information see the documentation in usbdrv/usbdrv.h.
 */
uchar   usbFunctionWrite(uchar *data, uchar len)
{
    if(bytesRemaining == 0)
        return 1;               /* end of transfer */
    if(len > bytesRemaining)
        len = bytesRemaining;
    eeprom_write_block(data, (uchar *)0 + currentAddress, len);
    currentAddress += len;
    bytesRemaining -= len;
    return bytesRemaining == 0; /* return 1 if this was the last chunk */
}

/* ------------------------------------------------------------------------- */

usbMsgLen_t usbFunctionSetup(uchar data[8])
{
usbRequest_t    *rq = (void *)data;

    if((rq->bmRequestType & USBRQ_TYPE_MASK) == USBRQ_TYPE_CLASS){    /* HID class request */
        if(rq->bRequest == USBRQ_HID_GET_REPORT){  /* wValue: ReportType (highbyte), ReportID (lowbyte) */
            /* since we have only one report type, we can ignore the report-ID */
            bytesRemaining = 128;
            currentAddress = 0;
            return USB_NO_MSG;  /* use usbFunctionRead() to obtain data */
        }else if(rq->bRequest == USBRQ_HID_SET_REPORT){
            /* since we have only one report type, we can ignore the report-ID */
            bytesRemaining = 128;
            currentAddress = 0;
            return USB_NO_MSG;  /* use usbFunctionWrite() to receive data from host */
        }
    }else{
        /* ignore vendor type requests, we don't use any */
    }
    return 0;
}

static void
ioinit(void)
{

	/*
* Set up the 16-bit timer 1.
*
* Timer 1 will be set up as a 10-bit phase-correct PWM (WGM10 and
* WGM11 bits), with OC1A used as PWM output.  OC1A will be set when
* up-counting, and cleared when down-counting (COM1A1|COM1A0), this
* matches the behaviour needed by the STK500's low-active LEDs.
* The timer will runn on full MCU clock (1 MHz, CS10 in TCCR1B).
*/


	/*  TCCR1B speed settings
CS12	CS11	CS10	Description
0		0		0		No clock source. (Timer/Counter stopped)
0		0		1		clkI/O/1 (No prescaling)
0		1		0		clkI/O/8 (From prescaler)
0		1		1		clkI/O/64 (From prescaler)
1		0		0		clkI/O/256 (From prescaler)
1		0		1		clkI/O/1024 (From prescaler)
1		1		0		External clock source on T1 pin. Clock on falling edge.
1		1		1		External clock source on T1 pin. Clock on rising edge.
*/
	//  TCCR1A = _BV(WGM10) | _BV(WGM11) | _BV(COM1A1) | _BV(COM1A0);

//	TCCR1A = _BV();
	TCCR1B = _BV(WGM21) | _BV(CS10) | _BV(WGM13) | _BV(WGM12); // Timer1 in CTC mode, toggle OC2 Compare Match, start with no prescaling
	OCR1A = TMR1_OCR;
	
	//Enable INT1
	MCUCR = _BV(ISC00);
	GICR = _BV(INT1);

	IRLED_DDR = _BV(IRLED_OUT);

	IRREC_PORT |= _BV(IRREC_IN);


	TIMSK =  _BV(OCIE1A);


}

/* ------------------------------------------------------------------------- */

int main(void)
{
uchar   i;

    wdt_enable(WDTO_1S);
    /* Even if you don't use the watchdog, turn it off here. On newer devices,
     * the status of the watchdog (on/off, period) is PRESERVED OVER RESET!
     */
    /* RESET status: all port bits are inputs without pull-up.
     * That's the way we need D+ and D-. Therefore we don't need any
     * additional hardware initialization.
     */
    ioinit();
    usbInit();
    usbDeviceDisconnect();  /* enforce re-enumeration, do this while interrupts are disabled! */
    i = 0;
    while(--i){             /* fake USB disconnect for > 250 ms */
        wdt_reset();
        _delay_ms(1);
    }
    usbDeviceConnect();
    iarr=0;
    mode = MODE_WAIT_RECIEVE;
    sei();
    for(;;){                /* main event loop */
        wdt_reset();
        usbPoll();
    }
    return 0;
}

/* ------------------------------------------------------------------------- */

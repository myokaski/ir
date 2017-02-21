/*
* ----------------------------------------------------------------------------
* "THE BEER-WARE LICENSE" (Revision 42):
* <joerg@FreeBSD.ORG> wrote this file.  As long as you retain this notice you
* can do whatever you want with this stuff. If we meet some day, and you think
* this stuff is worth it, you can buy me a beer in return.        Joerg Wunsch
* ----------------------------------------------------------------------------
*
* More advanced AVR demonstration.  Controls a LED attached to OCR1A.
* The brightness of the LED is controlled with the PWM.  A number of
* methods are implemented to control that PWM.
*
* $Id: largedemo.c,v 1.3 2007/01/19 22:17:10 joerg_wunsch Exp $
*/

#include <stdint.h>
#include <stdlib.h>

#include <avr/eeprom.h>
#include <avr/interrupt.h>
#include <avr/io.h>
#include <avr/pgmspace.h>
#include <avr/sleep.h>
#include <avr/wdt.h>

/* Part 1: Macro definitions */

#define IRLED_PORT PORTB
#define IRLED_DDR  DDRB
#define IRLED_OUT  PB3

#define IRREC_PORT PORTD
#define IRREC_DDR  DDRD
#define IRREC_PIN  PIND
#define IRREC_IN   PD2

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



#define F_CPU 8000000UL	/* CPU clock in Hertz */

#define SOFTCLOCK_FREQ 1000000	/* internal software clock */

/*
* Timeout to wait after last PWM change till updating the EEPROM.
* Measured in internal clock ticks (approx. 100 Hz).
*/
#define EE_UPDATE_TIME (3 * SOFTCLOCK_FREQ) /* ca. 3 seconds */

/*
* Timer1 overflow interrupt will be called with F_CPU / 2048
* frequency.  This interrupt routine further divides that value,
* resulting in an internal update interval of approx. 10 ms.
* (The complicated looking scaling by 10 / addition of 9 is
* poor man's fixed-point rounding algorithm...)
*/
#define TMR1_SCALE ((F_CPU * 10) / (2048UL * SOFTCLOCK_FREQ) + 9) / 10

#define IRLED_FREQ 38000 // must be beetween 15625 - 4000000 with 8MHz CPU and prescaler 1
#define TMR2_OCR F_CPU/2/IRLED_FREQ - 1

/* Part 2: Variable definitions */

/*
* Bits that are set inside interrupt routines, and watched outside in
* the program's main loop.
*/
volatile struct
{
	uint8_t tmr_int: 1;
	uint8_t rx_int: 1;
	uint8_t int0_int: 1;
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

/*
* Last character read from the UART.
*/
volatile char rxbuff;
volatile char cha;

#define IRMAX 200
volatile uint16_t ovfarr[IRMAX];
volatile int8_t arr[IRMAX];
volatile int16_t iarr = 0;
volatile uint16_t ovf_count=0;

/*
* Where to store the PWM value in EEPROM.  This is used in order
* to remember the value across a RESET or power cycle.
*/
uint16_t ee_pwm __attribute__((section(".eeprom"))) = 42;

/*
* Current value of the PWM.
*/
int16_t pwm;

uint16_t tcnt;

/*
* EEPROM backup timer.  Bumped by the PWM update routine.  If it
* expires, the current PWM value will be written to EEPROM.
*/
int16_t pwm_backup_tmr;

/*
* Mirror of the MCUCSR register, taken early during startup.
*/
uint8_t mcucsr __attribute__((section(".noinit")));

/* Part 3: Interrupt service routines */

ISR(TIMER1_OVF_vect)
{

}

ISR(TIMER2_COMP_vect)
{
	if(mode == MODE_RECIEVING)
	{
		if(++ovf_count == 0)
		{
			ovfarr[iarr] = 0;
			intflags.int0_int = 1;
		}
	}
	if(mode == MODE_TRANSMIT)
	{
		if(ovf_count==0)
		{
			++iarr;
			if(0 == (ovf_count = ovfarr[iarr]))
			{
				TCCR2 &= ~_BV(COM20);
				mode = MODE_IDLE;
				intflags.irtx_int = 1;
			}
			else
			{
				if(arr[iarr] == 0)
					TCCR2 |= _BV(COM20);
				else 
					TCCR2 &= ~_BV(COM20);
			}
		}
		--ovf_count;
	}
	
}

ISR(INT0_vect)
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
			intflags.int0_int = 1;
		}
	}
}


/*
* UART receive interrupt.  Fetch the character received and buffer
* it, unless there was a framing error.  Note that the main loop
* checks the received character only once per 10 ms.
*/
ISR(USART_RXC_vect)
{
	uint8_t c;

	c = UDR;
	if (bit_is_clear(UCSRA, FE))
	{
		rxbuff = c;
		intflags.rx_int = 1;
	}
}

/* Part 4: Auxiliary functions */

/*
* Read out and reset MCUCSR early during startup.
*/
void handle_mcucsr(void)
__attribute__((section(".init3")))
__attribute__((naked));
void handle_mcucsr(void)
{
	mcucsr = MCUCSR;
	MCUCSR = 0;
}

/*
* Do all the startup-time peripheral initializations.
*/
static void
ioinit(void)
{
	uint16_t pwm_from_eeprom;

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
	TCCR1B =  _BV(CS11);
	OCR1A = 0;			/* set PWM value to 0 */

	OCR2 = TMR2_OCR;
	TCCR2 = _BV(WGM21) | _BV(CS20); // Timer2 in CTC mode, toggle OC2 Compare Match, start with no prescaling
	
	//Enable INT0
	MCUCR = _BV(ISC00);
	GICR = _BV(INT0);

	IRLED_DDR = _BV(IRLED_OUT);

	IRREC_PORT |= _BV(IRREC_IN);
	/*
* As the location of OC1A differs between supported MCU types, we
* enable that output separately here.  Note that the DDRx register
* *might* be the same as CONTROL_DDR above, so make sure to not
* clobber it.
*/
	PWMDDR |= _BV(PWMOUT);

	UCSRA = _BV(U2X);		/* improves baud rate error @ F_CPU = 1 MHz */
	UCSRB = _BV(TXEN)|_BV(RXEN)|_BV(RXCIE); /* tx/rx enable, rx complete intr */
	UBRRL = (F_CPU / (8 * 9600UL)) - 1;  /* 9600 Bd */


	TIMSK = _BV(TOIE1) | _BV(OCIE2);
	sei();			/* enable interrupts */

	/*
* Enable the watchdog with the largest prescaler.  Will cause a
* watchdog reset after approximately 2 s @ Vcc = 5 V
*/
	wdt_enable(WDTO_2S);

	/*
* Read the value from EEPROM.  If it is not 0xffff (erased cells),
* use it as the starting value for the PWM.
*/
	if ((pwm_from_eeprom = eeprom_read_word(&ee_pwm)) != 0xffff)
	OCR1A = (pwm = pwm_from_eeprom);
}

/*
* Some simple UART IO functions.
*/

/*
* Send character c down the UART Tx, wait until tx holding register
* is empty.
*/
static void
putchr(char c)
{

	loop_until_bit_is_set(UCSRA, UDRE);
	UDR = c;
}

/*
* Send a C (NUL-terminated) string down the UART Tx.
*/
static void
printstr(const char *s)
{

	while (*s)
	{
		if (*s == '\n')
		putchr('\r');
		putchr(*s++);
	}
}

/*
* Same as above, but the string is located in program memory,
* so "lpm" instructions are needed to fetch it.
*/
static void
printstr_p(const char *s)
{
	char c;

	for (c = pgm_read_byte(s); c; ++s, c = pgm_read_byte(s))
	{
		if (c == '\n')
		putchr('\r');
		putchr(c);
	}
}


/* Part 5: main() */

int
main(void)
{
	int8_t i;
	ioinit();

	if ((mcucsr & _BV(WDRF)) == _BV(WDRF))
	printstr_p(PSTR("\nOoops, the watchdog bit me!"));

	printstr_p(PSTR("\nHello, this is the avr-gcc/libc "
	"demo running on an "
#if defined(__AVR_ATmega16__)
	"ATmega16"
#elif defined(__AVR_ATmega8__)
	"ATmega8"
#elif defined(__AVR_ATmega48__)
	"ATmega48"
#elif defined(__AVR_ATmega88__)
	"ATmega88"
#elif defined(__AVR_ATmega168__)
	"ATmega168"
#elif defined(__AVR_ATtiny2313__)
	"ATtiny2313"
#else
	"unknown AVR"
#endif
	"\n"));

	for (;;)
	{
		wdt_reset();
		if (intflags.irtx_int)
		{		
				intflags.irtx_int = 0;
				char s[8];
				printstr_p(PSTR("\n"));
				itoa(iarr, s, 10);
				printstr(s);
				printstr_p(PSTR(" sequences transmited\n"));
		}
		if (intflags.int0_int)
		{
			intflags.int0_int = 0;
/*			char s[8];
			for(i = 0; i<iarr; i++)
			{
				printstr_p(PSTR("\n"));
				itoa(i, s, 10);
				printstr(s);
				printstr_p(PSTR(" "));

				itoa(arr[i], s, 10);
				printstr(s);
				printstr_p(PSTR(" "));

				utoa(ovfarr[i], s, 10);
				printstr(s);
				printstr_p(PSTR(" "));

			}
*/			for(i = 4; i<iarr; i++)
			{
				if(arr[i] == 0)
					continue;
				if(ovfarr[i] > 200)
					printstr_p(PSTR("X"));
				else if(ovfarr[i] > 100)
					printstr_p(PSTR("1"));
				else
					printstr_p(PSTR("0"));
				if(!((i-2)%8))
					printstr_p(PSTR(" "));
			}
			mode = MODE_IDLE;
			/////////////////test
			//printstr_p(PSTR("\nWaiting for IR signal\n"));
			//iarr=0;
			//mode = MODE_WAIT_RECIEVE;
			/////////////////////test
		}
		if (intflags.rx_int)
		{
			intflags.rx_int = 0;

			switch (rxbuff)
			{
			case 'r':
				printstr_p(PSTR("\nWaiting for IR signal\n"));
				iarr=0;
				mode = MODE_WAIT_RECIEVE;
				break;
			case 't':
				printstr_p(PSTR("\nTransmitting...\n"));
				iarr=-1;
				ovf_count = 0;
				mode = MODE_TRANSMIT;
				break;

			case 'z':
				printstr_p(PSTR("\nzzzz... zzz..."));
				for (;;)
				;
			}
		
		}
		//      sleep_mode();
	}
}

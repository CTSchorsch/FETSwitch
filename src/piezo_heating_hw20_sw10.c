/*
 * PIEZO Heating Vest Regulator
 * for PIEZO HW Rev. B 
 * 
 * Mikrocontroller Code:
 * Version 1.0
 * 
 * Resourcen: 
 * 		Timer0 	-> PWM
 * 		Timer1  -> Baudrate für RS232
 * 		USI	   	-> IIC Interface
 * 		INT0	-> Switch press detection
 * 
 * History:
 * 10.05.2017	Copy from PIEZO Rev. B
 * 29.05.2017	add double click for on/off
 * 06.03.2019	change to button less
 * 
 */
#include <avr/io.h>			
#include <util/delay.h>		
#include <avr/interrupt.h>
#include <stdio.h>
#include <avr/power.h>
#include <avr/eeprom.h>
#include "usitwim.h"
#include <string.h>
#include <avr/sleep.h>
#include <stdlib.h>
#include <avr/eeprom.h>

#define DEBUG

//Helper
#define DIMSTATE_ADDR 0
#define sbi(x,b) x |= (1 << (b))
#define cbi(x,b) x &= ~(1 << (b))

#define PORTAMAP	0x80  //PWM Invertiert
#define PWM_100		254
#define PWM_50 		128
#define PWM_25		64
#define PWM_0		0


volatile uint8_t dimmlevel=PWM_0;


/*********** RS232 *****************************************************/

#ifdef DEBUG
#define RS232_PORT		PORTA
#define RS232_TXPIN		PINA5
volatile uint8_t txcount;
volatile uint8_t txdata;
char txtbuff[20];
/*
 * BitBang RS232
 */
ISR(TIM1_COMPA_vect) 
{

	if (txcount > 0) {
		switch (txcount) {
			case 10: 
				//Startbit low
				cbi(RS232_PORT,RS232_TXPIN); break;
			case 9:
			case 8:
			case 7:
			case 6:
			case 5:
			case 4:
			case 3:
			case 2: 
				if ( txdata & 0x01 )
				  sbi(RS232_PORT,RS232_TXPIN);
				else
				  cbi(RS232_PORT,RS232_TXPIN);
				txdata = txdata >> 1;
				break;
			case 1:
				 sbi(RS232_PORT,RS232_TXPIN);
				 break;
			default:
				 sbi(RS232_PORT,RS232_TXPIN);
				 break;
		}
		txcount--;
	} else { 
		TCCR1B &= ~(1 << CS01); //Timer stoppen
		sbi(RS232_PORT,RS232_TXPIN);
	} 
}
 
void sendChar(char data)
{
	while (txcount!=0);
		txdata=data;
		txcount=10;
		TCCR1B |= (1 << CS11);  //8Mhz / 8 = 1 Mhz => 1uS
}

void sendString( char *txt )			// send string
{
  while( *txt )
    sendChar( *txt++ );
  sendChar('\n');
  sendChar('\r');
}

void initSerial( void ) 
{
   TCCR1B |= (1 << WGM12); // Timer0 no prescaler, Clear Timer on Compare, Mode 4
   OCR1A = 103;
   TIMSK1 |= (1 << OCIE1A);
   cbi(RS232_PORT, RS232_TXPIN);
}
#else
	void initSerial( void ) {}
#endif

/********** PWM *******************************************************/
/* 
 * Inits PWM on Timer 0 and Output Compare Register B
 */
void pwm_init( void )
{
	TCCR0B = 0;  // no clock select -> off
	OCR0B = 0;
	TCCR0A = (1 << COM0B1) | (1 << COM0B0) | (1 << WGM01) | (1 << WGM00); // inverting, fast pwm
}
	
/*
 * Set OCR0B for compare
 */
void pwm_set( uint8_t val )
{
	OCR0B = val;
	TCCR0B = (1 << CS02) ; // clock divider 256: 8 Mhz / 256 / 256 = 122 Hz PWM frequency
}

/********** MAIN ******************************************************/


int main (void)		
{
	//Set MasterClock to 8MHz
	clock_prescale_set(clock_div_1);

	uint8_t startupflag = 1;
	uint8_t cnt=0;

	PORTA = (0x00 ^ PORTAMAP); // mal alles aus
	DDRA = 0xFF; //A0-A2 LED Output, A7 PWM OUTPUT
	DDRB = 0x00; //Pin B2 ist Switch input, B0 Mode Input
	PORTB = (1 << PB2) | (1 << PB0); // Interne Pull-Ups 
	pwm_init();
	initSerial();
	
    //check startup dimmstate
    dimmlevel=eeprom_read_byte(DIMSTATE_ADDR);
	pwm_set(dimmlevel);
    switch (dimmlevel) {
	  case PWM_25:
		eeprom_update_byte(DIMSTATE_ADDR,PWM_50);
		break; 
	  case PWM_50:
		eeprom_update_byte(DIMSTATE_ADDR,PWM_100);
		break;
	  case PWM_100:
		eeprom_update_byte(DIMSTATE_ADDR,PWM_25);
		break;
	  default:
	    eeprom_update_byte(DIMSTATE_ADDR, PWM_25);
	    break;
	}	
	sei();
	while(1) {
		pwm_set(dimmlevel);
		_delay_ms(500);
		cnt++;
		if ( (cnt  > 10) && (startupflag == 1) ) {
			startupflag = 0;
			eeprom_update_byte(DIMSTATE_ADDR, PWM_25);
		}
	}	
	// nie erreicht 
	return 0;
}


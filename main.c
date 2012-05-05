/* Name: main.c
 * Project: 4-Key-Keyboard
 * Author: Flip van den Berg - www.flipwork.nl
 * Creation Date: February 2010
 * Based on V-USB drivers from Objective Developments - http://www.obdev.at/products/vusb/index.html
 */

#define F_CPU 20000000

#include <avr/io.h>
#include <avr/wdt.h>
#include <avr/eeprom.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <util/delay.h>
#include <stdlib.h>

#include "usbdrv.h"
#include "lcd-routines.h"
#include "encoder.h"

/* ------------------------------------------------------------------------- */

static uchar    reportBuffer[2] = {1,0};    /* buffer for HID reports */

PROGMEM const char usbHidReportDescriptor[USB_CFG_HID_REPORT_DESCRIPTOR_LENGTH] = {
    0x05, 0x01,                    // USAGE_PAGE (Generic Desktop)
    0x09, 0x04,                    // USAGE (Joystick)
    0xa1, 0x01,                    // COLLECTION (Application)
    0x85, 0x01,                    //   REPORT_ID (1)
    0x09, 0x30,                    //   USAGE (X)
    0x26, 0xff, 0x00,              //   LOGICAL_MAXIMUM (255)
    0x46, 0xff, 0x00,              //   PHYSICAL_MAXIMUM (255)
    0x75, 0x08,                    //   REPORT_SIZE (8)
    0x95, 0x01,                    //   REPORT_COUNT (1)
    0x81, 0x02,                    // INPUT (Data,Var,Abs)
    0xc0                           // END_COLLECTION
};

static uchar    idleRate;           /* in 4 ms units */
static uchar    newReport = 0;		/* current report */

static uchar	debounceTimeIsOver = 1;	/* for switch debouncing */


/* ------------------------------------------------------------------------- */

void buildReport() {

}
 
static void timerPoll(void)
{
	static unsigned int timerCnt;

	if (TIFR0 & (1<<TOV0)){ /* 22 ms timer */
		TIFR0 = 1<<TOV0;  /* clear overflow */
		if (++timerCnt >= 6){
			timerCnt = 0;
			debounceTimeIsOver = 1;
		}
	}
}



void hadUsbReset(void) { return; }

/* ------------------------------------------------------------------------- */

static void hardwareInit(void)
{
	/**** SPI initialization ****/
	// set PB2(/SS), PB3(MOSI), PB5(SCK) as output
	DDRB    = (1<<PB2)|(1<<PB3)|(1<<PB5);
	PORTB &= ~((1<<PB4)); // make sure MISO pull-up is disabled
	PORTB &= ~(1<<PB2); // clear PARALLEL INPUT

    TCCR0B = 5;      /* timer 0 prescaler: 1024 */

}

static uint8_t readByteSpi() {
	// enable SPI in Master Mode with SCK = CK/128
	SPCR    = (1<<SPE)|(1<<MSTR)|(1<<CPOL)|(1<<SPR0)|(1<<SPR1);
	volatile char IOReg;
	IOReg   = SPSR;                         // clear SPIF bit in SPSR
	IOReg   = SPDR;

	SPDR = 0x00; // shift out 8 bits (all zeroes here, nobody cares)
		     // so 8 bits will be read back in
        while (!(SPSR & (1<<SPIF)));
	
	return SPDR;
}

/* -------------------------------------------------------------------------------- */
/* ------------------------ interface to USB driver ------------------------ */
/* -------------------------------------------------------------------------------- */

uchar	usbFunctionSetup(uchar data[8])
{
usbRequest_t    *rq = (void *)data;

    usbMsgPtr = reportBuffer;
    if((rq->bmRequestType & USBRQ_TYPE_MASK) == USBRQ_TYPE_CLASS){    /* class request type */
        if(rq->bRequest == USBRQ_HID_GET_REPORT){  /* wValue: ReportType (highbyte), ReportID (lowbyte) */
            /* we only have one report type, so don't look at wValue */
		    buildReport();
            return sizeof(reportBuffer);
        }else if(rq->bRequest == USBRQ_HID_GET_IDLE){
            usbMsgPtr = &idleRate;
            return 1;
        }else if(rq->bRequest == USBRQ_HID_SET_IDLE){
            idleRate = rq->wValue.bytes[1];
        }
    }else{
        /* no vendor specific requests implemented */
    }
	return 0;
}

/* ------------------------------------------------------------------------- */
/* --------------------------------- main ---------------------------------- */
/* ------------------------------------------------------------------------- */

void parallelIn() {

	PORTB |= (1<<PB2); // set PARALLEL INPUT
	_delay_us(1);

	PORTB &= ~(1<<PB2); // clear PARALLEL INPUT
	_delay_us(1);

}

int main(void)
{
uchar   i;
    
    usbInit();
    usbDeviceDisconnect();  /* enforce re-enumeration, do this while interrupts are disabled! */
    i = 0;
    while(--i){             /* fake USB disconnect for > 250 ms */
        wdt_reset();
        _delay_ms(1);
    }
    usbDeviceConnect();

    wdt_enable(WDTO_2S);

    hardwareInit();
	
	lcd_init();
	
	
    sei();

	
	uint8_t val1 = 0;
	uint8_t val2 = 0;
	uint8_t oldstate = 0x00;
	uint8_t newstate;
	uint8_t events;
    for(;;){    /* main event loop */
		
		/* */
		parallelIn();
		newstate = readByteSpi();
		events = encoder_events(oldstate, newstate);
		if (events & ECEV_LEFT) val1--;
		if (events & ECEV_RIGHT) val1++;
	
		events = encoder_events((oldstate >> 3), (newstate >> 3));
		if (events & ECEV_LEFT) val2--;
		if (events & ECEV_RIGHT) val2++;		

		oldstate=newstate;
		
		reportBuffer[1] = val1;
		/* */
		reportBuffer[0]=1;
	
		lcd_home();
		lcd_num(val1); lcd_data(' '); lcd_num(val2);
		
        wdt_reset();
        usbPoll();

		
		newReport = 0;
		if(usbInterruptIsReady() && newReport == 0){ /* we can send another report */
        	buildReport();
           	usbSetInterrupt(reportBuffer, sizeof(reportBuffer));
        	}
        
		timerPoll();
	}
   	return 0;

}

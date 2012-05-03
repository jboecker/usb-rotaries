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

static uchar    reportBuffer[8] = {0,0,0,0,0,0,0,0};    /* buffer for HID reports */

/* Reportbuffer format:

	0  Modifier byte
	1  reserved
	2  keycode array (0)
	3  keycode array (1)
	4  keycode array (2)
	5  keycode array (3)
	6  keycode array (4)
	7  keycode array (5)
	
	<< This is the standard usb-keyboard reportbuffer. It allows for 6 simultaneous keypresses to be detected (excl. modifier keys). In this application we only use 1, so the last 5 bytes in this buffer will always remain 0. >>
	<< I decided not to optimize this in order to make it easy to add extra keys that can be pressed simultaneously>>
	
   Modifier byte: 8 bits, each individual bit represents one of the modifier keys.

   	bit0  LEFT CTRL		(1<<0)
	bit1  LEFT SHIFT	(1<<1)
	bit2  LEFT ALT		(1<<2)
	bit3  LEFT GUI		(1<<3)
	bit4  RIGHT CTRL	(1<<4)
	bit5  RIGHT SHIFT	(1<<5)
	bit6  RIGHT ALT		(1<<6)
	bit7  RIGHT GUI		(1<<7)

	an example of a reportBuffer for a CTRL+ALT+Delete keypress:

	{((1<<0)+(1<<2)),0,76,0,0,0,0,0}

	the first byte holds both the LEFT CTRL and LEFT  modifier keys the 3rd byte holds the delete key (== decimal 76)

*/

static uchar    idleRate;           /* in 4 ms units */
static uchar    newReport = 0;		/* current report */

static uchar buttonState[5] = {3,3,3,3,3};
static uchar buttonChanged[5];

static uchar	debounceTimeIsOver = 1;	/* for switch debouncing */


/* ------------------------------------------------------------------------- */

PROGMEM const char usbHidReportDescriptor[USB_CFG_HID_REPORT_DESCRIPTOR_LENGTH] = {
    0x05, 0x01,                    // USAGE_PAGE (Generic Desktop)
    0x09, 0x06,                    // USAGE (Keyboard)
    0xa1, 0x01,                    // COLLECTION (Application)
    0x05, 0x07,                    //   USAGE_PAGE (Keyboard)
    0x19, 0xe0,                    //   USAGE_MINIMUM (Keyboard LeftControl)
    0x29, 0xe7,                    //   USAGE_MAXIMUM (Keyboard Right GUI)
    0x15, 0x00,                    //   LOGICAL_MINIMUM (0)
    0x25, 0x01,                    //   LOGICAL_MAXIMUM (1)
    0x75, 0x01,                    //   REPORT_SIZE (1)
    0x95, 0x08,                    //   REPORT_COUNT (8)
    0x81, 0x02,                    //   INPUT (Data,Var,Abs)	** Modifier Byte **
    0x95, 0x01,                    //   REPORT_COUNT (1)
    0x75, 0x08,                    //   REPORT_SIZE (8)
    0x81, 0x03,                    //   INPUT (Cnst,Var,Abs)	** Reserved Byte **
    0x95, 0x05,                    //   REPORT_COUNT (5)
    0x75, 0x01,                    //   REPORT_SIZE (1)
    0x05, 0x08,                    //   USAGE_PAGE (LEDs)
    0x19, 0x01,                    //   USAGE_MINIMUM (Num Lock)
    0x29, 0x05,                    //   USAGE_MAXIMUM (Kana)
    0x91, 0x02,                    //   OUTPUT (Data,Var,Abs)	** LED Report **
    0x95, 0x01,                    //   REPORT_COUNT (1)
    0x75, 0x03,                    //   REPORT_SIZE (3)
    0x91, 0x03,                    //   OUTPUT (Cnst,Var,Abs)	** LED Report Padding **
    0x95, 0x06,                    //   REPORT_COUNT (6)		** here we define the maximum number of simultaneous keystrokes we can detect ** 
    0x75, 0x08,                    //   REPORT_SIZE (8)
    0x15, 0x00,                    //   LOGICAL_MINIMUM (0)
    0x25, 0x65,                    //   LOGICAL_MAXIMUM (101)
    0x05, 0x07,                    //   USAGE_PAGE (Keyboard)
    0x19, 0x00,                    //   USAGE_MINIMUM (Reserved (no event indicated))
    0x29, 0x65,                    //   USAGE_MAXIMUM (Keyboard Application)
    0x81, 0x00,                    //   INPUT (Data,Ary,Abs)	** Key arrays (6 bytes) **
    0xc0                           // END_COLLECTION  
};
 
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

static void buildReport(void){
	
	uchar key; 
	uchar i;

	if(newReport == 0){	
		for (i=0; i<5; ++i) {
			if (buttonChanged[i] == 1){ // if button has changed
				if (buttonState[i] != 0) 
					key = 0; // key up event
				else
					key = 30 + i; // key down event
				
				buttonChanged[i] = 0;
				reportBuffer[2+i] = key;
			}
		}
		newReport = 1;; //if no button has changed, the previous report will be sent
	}
}

static void checkButtonChange(void) {
	uchar i;
	uchar tempButtonValue[5];
	uchar x = PINB;	
	uchar mask = 1;

	for (i=0; i<5; i++) {
		if ((x & mask) != 0)
			tempButtonValue[i] = 1;
		else
			tempButtonValue[i] = 0;

		if (tempButtonValue[i] != buttonState[i]) { // if button state has changed
			buttonState[i] = tempButtonValue[i];
			debounceTimeIsOver = 0; // debounce timer starts
			newReport = 0; // initiate new report
			buttonChanged[i] = 1;
		}
		
		mask <<= 1;
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
	_delay_us(10);

	PORTB &= ~(1<<PB2); // clear PARALLEL INPUT
	_delay_us(10);

}

int main(void)
{

	hardwareInit();
	
    lcd_init();
    lcd_clear();
    lcd_string("startup");

	uint8_t left_counter = 0;
	uint8_t right_counter = 0;
	uint8_t down_counter = 0;
	uint8_t up_counter = 0;
	uint8_t value = 0;
	
	lcd_clear();
	lcd_home();
	lcd_string("<-  ->  Dn  Up");
	
	uint8_t oldstate = 0x01;
	uint8_t newstate = 0x01;
	uint8_t events = 0;

    wdt_enable(WDTO_2S);
    for(;;){ // ignore USB
		wdt_reset();
		
		parallelIn();
		oldstate = newstate;
		newstate = readByteSpi();
		events = encoder_events(oldstate, newstate);

		
		if ((events & ECEV_LEFT) > 0)	{left_counter++; value--; }
		if ((events & ECEV_RIGHT) > 0) {right_counter++; value++; }
		if ((events & ECEV_BUTTON_DOWN) > 0) down_counter++;
		if ((events & ECEV_BUTTON_UP) > 0) up_counter++;
		
		
		lcd_setcursor(0,2);
		lcd_num(left_counter);
		lcd_data(' ');
		lcd_num(right_counter);
		lcd_data(' ');
		lcd_num(down_counter);
		lcd_data(' ');
		lcd_num(up_counter);
		lcd_data(' ');
		lcd_num(value);

     }
	return 0;

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

    sei();



    for(;;){    /* main event loop */
        wdt_reset();
        usbPoll();
		if (debounceTimeIsOver == 1){
		//	checkButtonChange();
		}

		if(usbInterruptIsReady() && newReport == 0){ /* we can send another report */
        	buildReport();
           	usbSetInterrupt(reportBuffer, sizeof(reportBuffer));
        	}
        
		timerPoll();
	}
   	return 0;
}

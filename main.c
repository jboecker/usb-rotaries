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
#include <string.h>

#include "usbdrv.h"
#include "lcd-routines.h"
#include "encoder.h"

/* ------------------------------------------------------------------------- */

static uchar    reportBuffer1[6] = {1,0,0,0,0,0};
static uchar    reportBuffer2[7] = {2,0,0,0,0,0,0};

PROGMEM const char usbHidReportDescriptor[71] = {
    0x05, 0x01,                    // USAGE_PAGE (Generic Desktop)
    0x09, 0x04,                    // USAGE (Joystick)
    0xa1, 0x01,                    // COLLECTION (Application)
    0x85, 0x01,                    //   REPORT_ID (1)
    0x09, 0x30,                    //   USAGE (X)
    0x09, 0x31,                    //   USAGE (Y)
    0x09, 0x32,                    //   USAGE (Z)
    0x26, 0xff, 0x00,              //   LOGICAL_MAXIMUM (255)
    0x46, 0xff, 0x00,              //   PHYSICAL_MAXIMUM (255)
    0x75, 0x08,                    //   REPORT_SIZE (8)
    0x95, 0x03,                    //   REPORT_COUNT (3)
    0x81, 0x02,                    // INPUT (Data,Var,Abs)

    0x05, 0x09,                    // USAGE_PAGE (Button)
    0x19, 0x01,                    // USAGE_MINIMUM (Button 1)
    0x29, 0x10,                    // USAGE_MAXIMUM (Button 16)
    0x15, 0x00,                    // LOGICAL_MINIMUM (0)
    0x25, 0x01,                    // LOGICAL_MAXIMUM (1)
    0x75, 0x01,                    // REPORT_SIZE (1)
    0x95, 0x10,                     // REPORT_COUNT (16)
    0x81, 0x02,                    // INPUT (Data,Var,Abs)
	
    0x85, 0x02,                    //   REPORT_ID (2)
    0x05, 0x01,                    // USAGE_PAGE (Generic Desktop)
	0x09, 0x33,                    //   USAGE (Rx)
    0x09, 0x34,                    //   USAGE (Ry)
    0x09, 0x35,                    //   USAGE (Rz)
    0x09, 0x36,                    //   USAGE (Slider)
    0x09, 0x37,                    //   USAGE (Dial)
    0x09, 0x38,                    //   USAGE (Wheel)
    0x26, 0xff, 0x00,              //   LOGICAL_MAXIMUM (255)
    0x46, 0xff, 0x00,              //   PHYSICAL_MAXIMUM (255)
    0x75, 0x08,                    //   REPORT_SIZE (8)
    0x95, 0x06,                    //   REPORT_COUNT (6)
    0x81, 0x02,                    // INPUT (Data,Var,Abs)

    0xc0                           // END_COLLECTION
};

static uchar    idleRate;           /* in 4 ms units */
static uchar    newReport = 0;		/* current report */



/* ------------------------------------------------------------------------- */
 
static void timerPoll(void)
{
	static unsigned int timerCnt;
	static char text[] = {'W','a','s',' ','w','a','e','r','e',' ','e','i','n',' ','L','C','D',' ','o','h','n','e',' ','L','a','u','f','s','c','h','r','i','f','t','?',' ','+','+','+',' ','\0'};
	static uint8_t startoffset = 0;

	if (TIFR0 & (1<<TOV0)){ /* 22 ms timer */
		TIFR0 = 1<<TOV0;  /* clear overflow */
		if (++timerCnt >= 12){
			timerCnt = 0;
			lcd_setcursor(0,2);
			
			char* pos = text + startoffset;
			for (uint8_t i=0; i<20; i++) {
				lcd_data(*pos);
				pos++;
				if (*pos == '\0')
					pos = text;
			}

			startoffset++;
			if (startoffset >= sizeof(text)) startoffset = 0;
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

    usbMsgPtr = NULL;
    if((rq->bmRequestType & USBRQ_TYPE_MASK) == USBRQ_TYPE_CLASS){    /* class request type */
        if(rq->bRequest == USBRQ_HID_GET_REPORT){  /* wValue: ReportType (highbyte), ReportID (lowbyte) */
			if (rq->wValue.bytes[1] == 1) { // look at the ReportID
				usbMsgPtr = reportBuffer1;
				return sizeof(reportBuffer1);
			} else {
				usbMsgPtr = reportBuffer2;
				return sizeof(reportBuffer2);
			}
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
	lcd_clear();
	PROGMEM const uint8_t chardata_switch_up[8] = {
		0b00011111,
		0b00011011,
		0b00010001,
		0b00011111,
		0b00000000,
		0b00000000,
		0b00000000,
		0b00000000
	};
    lcd_generatechar(LCD_GC_CHAR0, chardata_switch_up);
	PROGMEM const uint8_t chardata_switch_down[8] = {
		0b00000000,
		0b00000000,
		0b00011111,
		0b00010001,
		0b00010001,
		0b00011111,
		0b00000000,
		0b00000000
	};
	lcd_generatechar(LCD_GC_CHAR1, chardata_switch_down);
	PROGMEM const uint8_t chardata_switch_mid[8] = {
		0b00000000,
		0b00000000,
		0b00000000,
		0b00000000,
		0b00011111,
		0b00010001,
		0b00011011,
		0b00011111
	};
	lcd_generatechar(LCD_GC_CHAR2, chardata_switch_mid);

	lcd_clear();
	lcd_home();
	
    sei();

	
	uint8_t axisValues[9] = {0,0,0,0,0,0,0,0,0};
	uint8_t currentAxis = 0;
	uint8_t oldstate = 0x00;
	uint8_t newstate;
	uint8_t events;
    for(;;){    /* main event loop */
		
		/* */
		parallelIn();
		newstate = readByteSpi();
		events = encoder_events(oldstate, newstate);
		if (events & ECEV_LEFT) axisValues[currentAxis]--;
		if (events & ECEV_RIGHT) axisValues[currentAxis]++;
		if (events & ECEV_BUTTON_DOWN) reportBuffer1[4] |= 1;
		if (events & ECEV_BUTTON_UP) reportBuffer1[4] &= ~1;
	
		events = encoder_events((oldstate >> 3), (newstate >> 3));
		if (events & ECEV_LEFT) { currentAxis--; if (currentAxis == 255) currentAxis = 8; }
		if (events & ECEV_RIGHT) { currentAxis++; if (currentAxis > 8) currentAxis = 0; }
		if (events & ECEV_BUTTON_DOWN) reportBuffer1[4] |= 2;
		if (events & ECEV_BUTTON_UP) reportBuffer1[4] &= ~2;
	
		oldstate=newstate;
		
		reportBuffer1[1] = axisValues[0];
        reportBuffer1[2] = axisValues[1];
		reportBuffer1[3] = axisValues[2];
		
		reportBuffer2[1] = axisValues[3];
		reportBuffer2[2] = axisValues[4];
		reportBuffer2[3] = axisValues[5];
		reportBuffer2[4] = axisValues[6];
		reportBuffer2[5] = axisValues[7];
		reportBuffer2[6] = axisValues[8];
		
   
		lcd_home();
		lcd_num(currentAxis); lcd_data(' '); lcd_num(axisValues[currentAxis]);
		lcd_data(' '); lcd_data(0); lcd_data(1); lcd_data(2);
		
        wdt_reset();
        usbPoll();

		
		newReport = 0;
		static uchar reportNumber = 1;
		reportNumber++; if (reportNumber == 3) reportNumber = 1;
		if(usbInterruptIsReady() && newReport == 0){ /* we can send another report */
			if (reportNumber == 1)
				usbSetInterrupt(reportBuffer1, sizeof(reportBuffer1));
			else
				usbSetInterrupt(reportBuffer2, sizeof(reportBuffer2));
					
        	}
        
		timerPoll();
	}
   	return 0;

}

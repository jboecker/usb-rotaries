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
#include "lcd2-routines.h"
#include "encoder.h"

/* ------------------------------------------------------------------------- */

static uchar    reportBuffer1[3] = {1,0,0};
static uchar    reportBuffer2[7] = {2,0,0,0,0,0,0};
static uchar    reportBufferKbd[8] = {3,0,0,0,0,0,0,0};
static uchar    newKbdReport = 0;

PROGMEM const char usbHidReportDescriptor[142] = {
	//PROGMEM const char usbHidReportDescriptor[65] = {

    0x05, 0x01,                    // USAGE_PAGE (Generic Desktop)
    0x09, 0x06,                    // USAGE (Keyboard)
    0xa1, 0x01,                    // COLLECTION (Application)
    0xa1, 0x00,                    // COLLECTION (Physical)
    0x85, 0x03,                    // REPORT ID (3)
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
    0x95, 0x05,                    //   REPORT_COUNT (5)		** here we define the maximum number of simultaneous keystrokes we can detect ** 
    0x75, 0x08,                    //   REPORT_SIZE (8)
    0x05, 0x07,                    //   USAGE_PAGE (Keyboard)
    0x19, 0x00,                    //   USAGE_MINIMUM (Reserved (no event indicated))
    0x29, 0x65,                    //   USAGE_MAXIMUM (Keyboard Application)
    0x15, 0x00,                    //   LOGICAL_MINIMUM (0)
    0x25, 0x65,                    //   LOGICAL_MAXIMUM (101)
    0x81, 0x00,                    //   INPUT (Data,Ary,Abs)	** Key arrays (6 bytes) **
	0xc0,                           // END_COLLECTION  
	0xc0,                           // END_COLLECTION  

	
    0x05, 0x01,                    // USAGE_PAGE (Generic Desktop)
    0x09, 0x04,                    // USAGE (Joystick)
	0xa1, 0x01,                    // COLLECTION (Application)
    0xa1, 0x00,                    // COLLECTION (Physical)
    0x85, 0x01,                    //   REPORT_ID (1)

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

    0xc0,                           // END_COLLECTION
	0xc0,                           // END_COLLECTION  

};

static uchar    idleRate;           /* in 4 ms units */
static uchar    newReport = 0;		/* current report */


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
			} else if (rq->wValue.bytes[1] == 2) {
				usbMsgPtr = reportBuffer2;
				return sizeof(reportBuffer2);
			} else if (rq->wValue.bytes[1] == 3) {
				usbMsgPtr = reportBufferKbd;
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
    lcd_generatechar(LCD_GC_CHAR0, chardata_switch_up);
	lcd_generatechar(LCD_GC_CHAR1, chardata_switch_down);
	lcd_generatechar(LCD_GC_CHAR2, chardata_switch_mid);

	lcd_clear();
	lcd_home();
	
	lcd2_init();
	lcd2_clear();
	lcd2_string("This is LCD 2.");
	lcd2_setcursor(0,2);
	lcd2_string("Line 2.");

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
		if (events & ECEV_BUTTON_DOWN) reportBuffer1[1] |= 1;
		if (events & ECEV_BUTTON_UP) reportBuffer1[1] &= ~1;
	
		events = encoder_events((oldstate >> 3), (newstate >> 3));
		if (events & ECEV_LEFT) { currentAxis--; if (currentAxis == 255) currentAxis = 8; }
		if (events & ECEV_RIGHT) { currentAxis++; if (currentAxis > 8) currentAxis = 0; }
		if (events & ECEV_BUTTON_DOWN) { reportBufferKbd[5] = axisValues[0]; newKbdReport = 0; }
		if (events & ECEV_BUTTON_UP) { reportBufferKbd[5] = 0; newKbdReport = 0; }
	
		oldstate=newstate;
		
		//		reportBuffer1[1] = axisValues[0];
		//        reportBuffer1[2] = axisValues[1];
		//		reportBuffer1[3] = axisValues[2];
		
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
		reportNumber++; if (reportNumber == 4) reportNumber = 1;
		if(usbInterruptIsReady() && newReport == 0){ /* we can send another report */
			if (reportNumber == 1) {
				usbSetInterrupt(reportBuffer1, sizeof(reportBuffer1));
			} else if (reportNumber == 2) {
				usbSetInterrupt(reportBuffer2, sizeof(reportBuffer2));
			} else if (reportNumber == 3 && newKbdReport == 0) {
				usbSetInterrupt(reportBufferKbd, sizeof(reportBufferKbd));
				newKbdReport = 1;
			}
		}
        
		timerPoll();
	}
   	return 0;

}

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

#include "numsticks.h"

static uchar    reportBuffers[NUMBER_OF_STICKS][8];
static uchar    reportBufferSizes[NUMBER_OF_STICKS];
static uchar    reportBufferChanged[NUMBER_OF_STICKS];

static uchar selectedPage = 0;

// report IDs start at 1
#define REPORT_ID_MAX NUMBER_OF_STICKS

PROGMEM const char usbHidReportDescriptorTemplate[50] = {

	// begin report descriptor for (1 byte ID, 3 byte buttons, 4 byte axis values)
	// length is 50 byte, report length is 8 byte
    0x05, 0x01,                    // USAGE_PAGE (Generic Desktop)
    0x09, 0x04,                    // USAGE (Joystick)
	0xa1, 0x01,                    // COLLECTION (Application)
    0xa1, 0x00,                    // COLLECTION (Physical)
    0x85, 0x01,                    //   REPORT_ID (1)
    0x05, 0x09,                    // USAGE_PAGE (Button)
    0x19, 0x01,                    // USAGE_MINIMUM (Button 1)
    0x29, 0x18,                    // USAGE_MAXIMUM (Button 24)
    0x15, 0x00,                    // LOGICAL_MINIMUM (0)
    0x25, 0x01,                    // LOGICAL_MAXIMUM (1)
    0x75, 0x01,                    // REPORT_SIZE (1)
    0x95, 0x18,                    // REPORT_COUNT (24)
    0x81, 0x02,                    // INPUT (Data,Var,Abs)
    0x05, 0x01,                    // USAGE_PAGE (Generic Desktop)
	0x09, 0x33,                    //   USAGE (Rx)
    0x09, 0x34,                    //   USAGE (Ry)
    0x09, 0x35,                    //   USAGE (Rz)
    0x09, 0x36,                    //   USAGE (Slider)
    0x26, 0xff, 0x00,              //   LOGICAL_MAXIMUM (255)
    0x46, 0xff, 0x00,              //   PHYSICAL_MAXIMUM (255)
    0x75, 0x08,                    //   REPORT_SIZE (8)
    0x95, 0x04,                    //   REPORT_COUNT (4)
    0x81, 0x02,                    // INPUT (Data,Var,Abs)
    0xc0,                           // END_COLLECTION
	0xc0,                           // END_COLLECTION  

};

static uchar    idleRate;           /* in 4 ms units */

/* ------------------------------------------------------------------------- */
 
void hadUsbReset(void) { return; }

/* ------------------------------------------------------------------------- */

static unsigned int buttonReleaseDelayTimerCount;
static uchar canReleaseButtons = 0;
static void resetButtonReleaseTimer() {
	buttonReleaseDelayTimerCount = 0;
	canReleaseButtons = 0;
}
static void timerPoll(void)
{
	if (TIFR0 & (1<<TOV0)){ /* 22 ms timer */
		TIFR0 = 1<<TOV0;  /* clear overflow */
		if (++buttonReleaseDelayTimerCount >= 2){
			buttonReleaseDelayTimerCount = 0;
			canReleaseButtons = 1;
		}
	}
}

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

usbMsgLen_t usbFunctionDescriptor(struct usbRequest *rq) {
	// the only thing this is called for is to get the HID report descriptor
	static uchar reportDescriptor[sizeof(usbHidReportDescriptorTemplate) * NUMBER_OF_STICKS];
	
	for (uint8_t i=0; i<NUMBER_OF_STICKS; i++) {
		memcpy_P(reportDescriptor + (i*sizeof(usbHidReportDescriptorTemplate)), usbHidReportDescriptorTemplate, sizeof(usbHidReportDescriptorTemplate));
		reportDescriptor[9 + i*sizeof(usbHidReportDescriptorTemplate)] = i+1; // set report id
	}
	
	usbMsgPtr = (uchar*)&reportDescriptor;
	
	return sizeof(usbHidReportDescriptorTemplate) * NUMBER_OF_STICKS;
}

uchar	usbFunctionSetup(uchar data[8])
{
usbRequest_t    *rq = (void *)data;

    usbMsgPtr = NULL;
    if((rq->bmRequestType & USBRQ_TYPE_MASK) == USBRQ_TYPE_CLASS){    /* class request type */
        if(rq->bRequest == USBRQ_HID_GET_REPORT){  /* wValue: ReportType (highbyte), ReportID (lowbyte) */
			usbMsgPtr = reportBuffers[rq->wValue.bytes[1] - 1];
			return reportBufferSizes[rq->wValue.bytes[1] - 1];
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

// bitmask; buttonTap sets a bit here, so the button is released later
static uchar buttonsAwaitingRelease[NUMBER_OF_STICKS][3];

void noAction() {}
void buttonDown(uchar reportId, uchar buttonNumber) {
	reportBuffers[reportId-1][1 + ((buttonNumber-1)/8)] |= (1 << (buttonNumber-1)%8);
	reportBufferChanged[reportId-1] = 1;
}
void buttonUp(uchar reportId, uchar buttonNumber) {
	reportBuffers[reportId-1][1 + ((buttonNumber-1)/8)] &= ~(1 << (buttonNumber-1)%8);
	reportBufferChanged[reportId-1] = 1;
}
void buttonTap(uchar reportId, uchar buttonNumber) {
	buttonDown(reportId, buttonNumber);
	buttonsAwaitingRelease[reportId-1][(buttonNumber-1)/8] |= (1 << ((buttonNumber-1)%8));
	resetButtonReleaseTimer();
}
void axisDelta(uchar reportId, uchar axisNumber, char delta) {
	uchar* value = &reportBuffers[reportId-1][3+axisNumber];
	int16_t temp = ((int16_t) *value) + (int16_t)delta;
	if ((0 <= temp) && (temp <= 255))
		*value += delta;
	reportBufferChanged[reportId-1] = 1;
}

void selectPage(uchar page) {
	selectedPage = page;
	if (page == 1) {
		lcd_select(2);
		lcd_clear();
		lcd_string("Lighting Panel");
		
		lcd_select(1);
		lcd_clear();
		lcd_string("CONS ");
		lcd_string("ENG  ");
		lcd_string("FLTI ");
		lcd_string("FLOOD");
		lcd_setcursor(0,2);
		lcd_string("FORM ");
		lcd_string("NOSE ");
		lcd_string("POS  ");
		lcd_string("SIGNL");
	} else if (page == 2) {
		lcd_select(2);
		lcd_clear();
		lcd_string("AAP");
		lcd_setcursor(0,2);
		lcd_string("Electrical Panel");
		
		lcd_select(1);
		lcd_clear();
		lcd_string("CDU  ");
		lcd_string("EGI  ");
		lcd_string("EmFld");
		lcd_string(" BAT ");
		lcd_setcursor(0,2);
		lcd_string("GenL ");
		lcd_string("GenR ");
		lcd_string("GenA ");
		lcd_string("Inv  ");
	} else if (page == 3) {
		lcd_select(2);
		lcd_clear();
		lcd_string("Fuel System");
		
		lcd_select(1);
		lcd_clear();
		lcd_string("  BOO");
		lcd_string("ST   ");
		lcd_string("TkGt ");
		lcd_string("RcvrL");
		lcd_setcursor(0,2);
		lcd_string("   PU");
		lcd_string("MPS  ");
		lcd_string("     ");
		lcd_string("     ");
	} else if (page == 4) {
		lcd_select(2);
		lcd_clear();
		lcd_string("AHCP");
		
		lcd_select(1);
		lcd_clear();
		lcd_string("MArm ");
		lcd_string("GUN  ");
		lcd_string("Laser");
		lcd_string(" TGP ");
		lcd_setcursor(0,2);
		lcd_string("CICU ");
		lcd_string("JTRS ");
		lcd_string("IFFCC");
		lcd_string("     ");
	} else if (page == 5) {
		lcd_select(2);
		lcd_clear();
		lcd_string("Intercom");
		
		lcd_select(1);
		lcd_clear();
		lcd_string("FM   ");
		lcd_string("HF   ");
		lcd_string("INT  ");
		lcd_string("VHF  ");
		lcd_setcursor(0,2);
		lcd_string("TCN  ");
		lcd_string("ILS  ");
		lcd_string("AIM  ");
		lcd_string("Vol  ");
	} else if (page == 6) {
		lcd_select(2);
		lcd_clear();
		lcd_string("TACAN and ILS");
		
		lcd_select(1);
		lcd_clear();
		lcd_string("TCN C");
		lcd_string("hanne");
		lcd_string("l    ");
		lcd_string("     ");
		lcd_setcursor(0,2);
		lcd_string("ILS F");
		lcd_string("reque");
		lcd_string("ncy  ");
		lcd_string("     ");
	} else {
		lcd_select(2);
		lcd_clear();
		lcd_string("Page "); lcd_num(page);
		
		lcd_select(1);
		lcd_clear();
	}
}
void nextPage() {
	selectedPage++;
	if (selectedPage > NUMBER_OF_STICKS) selectedPage = 1;
	selectPage(selectedPage);
}
void previousPage() {
	selectedPage--;
	if (selectedPage == 0) selectedPage = NUMBER_OF_STICKS;
	selectPage(selectedPage);
}

void handleInput(uchar events[9]) {
	uchar event;

	/* set buttons 1 to 24 to react to dials 1 through 8 */
	for (uchar i=0; i<8; i++) {
		event = events[i];
		if (event & ECEV_BUTTON_DOWN) buttonDown(selectedPage,1+(i*3));
		if (event & ECEV_BUTTON_UP) buttonUp(selectedPage,1+(i*3));
		if (event & ECEV_LEFT) buttonTap(selectedPage,2+(i*3));
		if (event & ECEV_RIGHT) buttonTap(selectedPage,3+(i*3));	
	}

	if (events[8] & ECEV_LEFT) previousPage();
	if (events[8] & ECEV_RIGHT) nextPage();
	if (events[8] & ECEV_BUTTON_UP) selectPage(1);

}

int main(void)
{
	uchar   i;
    
	memset(buttonsAwaitingRelease, 0, sizeof(buttonsAwaitingRelease));
	memset(reportBuffers, 0, NUMBER_OF_STICKS * 8);
	for (i=0; i<NUMBER_OF_STICKS;i++) {
		reportBuffers[i][0] = i+1;   // set REPORT IDs
		reportBufferChanged[i] = 1;
		reportBufferSizes[i] = 8;
	}
	
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
	
	lcd_select(1);
	lcd_init();
	lcd_clear();
	lcd_string("Hello.");
	
	lcd_select(2);
	lcd_init();
	lcd_clear();

	selectPage(1);
	
    sei();

	static uint8_t events[9] =    {0,0,0,0,0,0,0,0,0};
	static uint8_t oldstates[9] = {0,0,0,0,0,0,0,0,0};
	static uint8_t newstates[9] = {0,0,0,0,0,0,0,0,0};

    for(;;){    /* main event loop */
		/* */
		parallelIn();
		uint8_t byte = readByteSpi();
		newstates[0] = (byte >> 5);
		newstates[1] = (byte >> 2);
		newstates[2] = ((byte << 1) & 0b00000110);

		byte = readByteSpi();
		newstates[2] |= ((byte >> 7) & 0b00000001);
		newstates[3] = (byte >> 4);
		newstates[4] = (byte >> 1);
		newstates[5] = (byte << 2) & 0b00000100;

		byte = readByteSpi();
		newstates[5] |= ((byte >> 6) & 0b00000011);
		newstates[6] = (byte >> 3);
		newstates[7] = byte;
		
		byte = readByteSpi();
		newstates[8] = (byte >> 5);
		
		for (uint8_t i = 0; i<9; i++) {
			events[i] = encoder_events(oldstates[i], newstates[i]);
		}

		handleInput(events);
		
		for (uint8_t i=0; i<9; i++)
			oldstates[i] = newstates[i];

        wdt_reset();
        usbPoll();

		timerPoll();
		if (canReleaseButtons) {
			for (i=0; i<REPORT_ID_MAX; i++) {
				if (reportBufferChanged[i] == 0) {
					// check if we want to release some buttons
					for (uint8_t j=0; j<3; j++) {
						if (buttonsAwaitingRelease[i][j] > 0) {
							reportBuffers[i][1+j] &= ~(buttonsAwaitingRelease[i][j]);
							buttonsAwaitingRelease[i][j] = 0x00;
							reportBufferChanged[i] = 1;
						}
					}
				}
			}
		}

		static uint8_t startReportId = 0;
		if(usbInterruptIsReady()){ /* we can send another report */
			startReportId++;
			for (uint8_t k = 0; k < REPORT_ID_MAX; k++) {
				uint8_t i = (startReportId + k) % (REPORT_ID_MAX);
				if (reportBufferChanged[i]) {
					usbSetInterrupt(reportBuffers[i], reportBufferSizes[i]);
					reportBufferChanged[i] = 0;
					break;
				}
			}
			
		}
        
	}
   	return 0;

}

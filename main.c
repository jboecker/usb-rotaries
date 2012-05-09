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

static uchar    reportBuffers[4][8] = {{1,0,0,0,0,0,0,0},
									   {2,0,0,0,0,0,0,0},
									   {3,0,0,0,0,0,0,0},
									   {4,0,0,0,0,0,0,0}};
static uchar    reportBufferSizes[] = {8,8,8,8};
static uchar    reportBufferChanged[] = {1,1,1,1};

static uchar reportBufferOffset = 0;

// report IDs start at 1
#define REPORT_ID_MAX 4

PROGMEM const char usbHidReportDescriptor[USB_CFG_HID_REPORT_DESCRIPTOR_LENGTH] = {

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
	// begin report descriptor for (1 byte ID, 3 byte buttons, 4 byte axis values)
	// length is 50 byte, report length is 8 byte
    0x05, 0x01,                    // USAGE_PAGE (Generic Desktop)
    0x09, 0x04,                    // USAGE (Joystick)
	0xa1, 0x01,                    // COLLECTION (Application)
    0xa1, 0x00,                    // COLLECTION (Physical)
    0x85, 0x02,                    //   REPORT_ID (2)
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
	// begin report descriptor for (1 byte ID, 3 byte buttons, 4 byte axis values)
	// length is 50 byte, report length is 8 byte
    0x05, 0x01,                    // USAGE_PAGE (Generic Desktop)
    0x09, 0x04,                    // USAGE (Joystick)
	0xa1, 0x01,                    // COLLECTION (Application)
    0xa1, 0x00,                    // COLLECTION (Physical)
    0x85, 0x03,                    //   REPORT_ID (3)
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
	// begin report descriptor for (1 byte ID, 3 byte buttons, 4 byte axis values)
	// length is 50 byte, report length is 8 byte
    0x05, 0x01,                    // USAGE_PAGE (Generic Desktop)
    0x09, 0x04,                    // USAGE (Joystick)
	0xa1, 0x01,                    // COLLECTION (Application)
    0xa1, 0x00,                    // COLLECTION (Physical)
    0x85, 0x04,                    //   REPORT_ID (4)
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

	lcd2_init();
	lcd2_clear();

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

		// TODO: react to events
		for (uint8_t i = 0; i<4; i++) {
			if (events[i] & ECEV_LEFT) if (reportBuffers[reportBufferOffset][4+i] > 4) reportBuffers[reportBufferOffset][4+i] -= 5;
			if (events[i] & ECEV_RIGHT) if (reportBuffers[reportBufferOffset][4+i] < 251) reportBuffers[reportBufferOffset][4+i] += 5;
			reportBufferChanged[reportBufferOffset] = 1;
		}
		for (uint8_t i = 4; i<8; i++) {
			if (events[i] & ECEV_LEFT) if (reportBuffers[reportBufferOffset+1][i] > 4) reportBuffers[reportBufferOffset + 1][i] -= 5;
			if (events[i] & ECEV_RIGHT) if (reportBuffers[reportBufferOffset+1][i] < 251) reportBuffers[reportBufferOffset + 1][i] += 5;
			reportBufferChanged[reportBufferOffset + 1] = 1;
		}
		if (events[8] & ECEV_LEFT) {
			reportBufferOffset = 0;
			lcd2_clear();
			lcd2_string("Stick 1");
		}
		if (events[8] & ECEV_RIGHT) {
			reportBufferOffset = 2;
			lcd2_clear();
			lcd2_string("Stick 2");
		}
		
		
		for (uint8_t i=0; i<9; i++)
			oldstates[i] = newstates[i];

		// output
		/*
		lcd_home();
		lcd_num(axisValues[0]); lcd_data(' ');
		lcd_num(axisValues[1]); lcd_data(' ');
		lcd_num(axisValues[2]); lcd_data(' ');
		lcd_num(axisValues[3]); lcd_data(' ');
		lcd_setcursor(0,2);
		lcd_num(axisValues[4]); lcd_data(' ');
		lcd_num(axisValues[5]); lcd_data(' ');
		lcd_num(axisValues[6]); lcd_data(' ');
		lcd_num(axisValues[7]); lcd_data(' ');
		
		lcd2_home();
		lcd2_num(axisValues[8]);
		////
		*/
		
        wdt_reset();
        usbPoll();

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

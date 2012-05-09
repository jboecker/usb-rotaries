// Ansteuerung eines HD44780 kompatiblen LCD im 4-Bit-Interfacemodus
// http://www.mikrocontroller.net/articles/HD44780
// http://www.mikrocontroller.net/articles/AVR-GCC-Tutorial/LCD-Ansteuerung
//
// Die Pinbelegung ist ber defines in lcd-routines.h einstellbar

#include <stdlib.h> 
#include <avr/io.h>
#include "lcd2-routines.h"
#include <string.h>
#include <util/delay.h>

 
////////////////////////////////////////////////////////////////////////////////
// Erzeugt einen Enable-Puls
static void lcd2_enable( void )
{
    LCD2_PORT |= (1<<LCD2_EN);     // Enable auf 1 setzen
    _delay_us( LCD2_ENABLE_US );  // kurze Pause
    LCD2_PORT &= ~(1<<LCD2_EN);    // Enable auf 0 setzen
}
 
////////////////////////////////////////////////////////////////////////////////
// Sendet eine 4-bit Ausgabeoperation an das LCD
static void lcd2_out( uint8_t data )
{
    data &= 0xF0;                       // obere 4 Bit maskieren
 
	LCD2_PORT &= ~(1 << LCD2_BIT0);
	LCD2_PORT &= ~(1 << LCD2_BIT1);
	LCD2_PORT &= ~(1 << LCD2_BIT2);
	LCD2_PORT &= ~(1 << LCD2_BIT3);

	if (data & 16)
		LCD2_PORT |= (1 << LCD2_BIT0);
	if (data & 32)
		LCD2_PORT |= (1 << LCD2_BIT1);
	if (data & 64)
		LCD2_PORT |= (1 << LCD2_BIT2);
	if (data & 128)
		LCD2_PORT |= (1 << LCD2_BIT3);

    lcd2_enable();
}
 
////////////////////////////////////////////////////////////////////////////////
// Initialisierung: muss ganz am Anfang des Programms aufgerufen werden.
void lcd2_init( void )
{
    // verwendete Pins auf Ausgang schalten
    uint8_t pins = (1 << LCD2_BIT0) |           // 4 Datenleitungen
		(1 << LCD2_BIT1) |
		(1 << LCD2_BIT2) |
		(1 << LCD2_BIT3) |
                   (1<<LCD2_RS) |                // R/S Leitung
                   (1<<LCD2_EN);                 // Enable Leitung
    LCD2_DDR |= pins;
 
    // initial alle Ausgnge auf Null
    LCD2_PORT &= ~pins;
 
    // warten auf die Bereitschaft des LCD
    _delay_ms( LCD2_BOOTUP_MS );
    
    // Soft-Reset muss 3mal hintereinander gesendet werden zur Initialisierung
    lcd2_out( LCD2_SOFT_RESET );
    _delay_ms( LCD2_SOFT_RESET_MS1 );
 
    lcd2_enable();
    _delay_ms( LCD2_SOFT_RESET_MS2 );
 
    lcd2_enable();
    _delay_ms( LCD2_SOFT_RESET_MS3 );
 
    // 4-bit Modus aktivieren 
    lcd2_out( LCD2_SET_FUNCTION |
             LCD2_FUNCTION_4BIT );
    _delay_ms( LCD2_SET_4BITMODE_MS );
 
    // 4-bit Modus / 2 Zeilen / 5x7
    lcd2_command( LCD2_SET_FUNCTION |
                 LCD2_FUNCTION_4BIT |
                 LCD2_FUNCTION_2LINE |
                 LCD2_FUNCTION_5X7 );
 
    // Display ein / Cursor aus / Blinken aus
    lcd2_command( LCD2_SET_DISPLAY |
                 LCD2_DISPLAY_ON |
                 LCD2_CURSOR_OFF |
                 LCD2_BLINKING_OFF); 
 
    // Cursor inkrement / kein Scrollen
    lcd2_command( LCD2_SET_ENTRY |
                 LCD2_ENTRY_INCREASE |
                 LCD2_ENTRY_NOSHIFT );
 
    lcd2_clear();
}
  
////////////////////////////////////////////////////////////////////////////////
// Sendet ein Datenbyte an das LCD
void lcd2_data( uint8_t data )
{
    LCD2_PORT |= (1<<LCD2_RS);    // RS auf 1 setzen
 
    lcd2_out( data );            // zuerst die oberen, 
    lcd2_out( data<<4 );         // dann die unteren 4 Bit senden
 
    _delay_us( LCD2_WRITEDATA_US );
}
 
////////////////////////////////////////////////////////////////////////////////
// Sendet einen Befehl an das LCD
void lcd2_command( uint8_t data )
{
    LCD2_PORT &= ~(1<<LCD2_RS);    // RS auf 0 setzen
 
    lcd2_out( data );             // zuerst die oberen, 
    lcd2_out( data<<4 );           // dann die unteren 4 Bit senden
 
    _delay_us( LCD2_COMMAND_US );
}
 
////////////////////////////////////////////////////////////////////////////////
// Sendet den Befehl zur Lschung des Displays
void lcd2_clear( void )
{
    lcd2_command( LCD2_CLEAR_DISPLAY );
    _delay_ms( LCD2_CLEAR_DISPLAY_MS );
}
 
////////////////////////////////////////////////////////////////////////////////
// Sendet den Befehl: Cursor Home
void lcd2_home( void )
{
    lcd2_command( LCD2_CURSOR_HOME );
    _delay_ms( LCD2_CURSOR_HOME_MS );
}
 
////////////////////////////////////////////////////////////////////////////////
// Setzt den Cursor in Spalte x (0..15) Zeile y (1..4) 
 
void lcd2_setcursor( uint8_t x, uint8_t y )
{
    uint8_t data;
 
    switch (y)
    {
        case 1:    // 1. Zeile
            data = LCD2_SET_DDADR + LCD2_DDADR_LINE1 + x;
            break;
 
        case 2:    // 2. Zeile
            data = LCD2_SET_DDADR + LCD2_DDADR_LINE2 + x;
            break;
 
        case 3:    // 3. Zeile
            data = LCD2_SET_DDADR + LCD2_DDADR_LINE3 + x;
            break;
 
        case 4:    // 4. Zeile
            data = LCD2_SET_DDADR + LCD2_DDADR_LINE4 + x;
            break;
 
        default:
            return;                                   // fr den Fall einer falschen Zeile
    }
 
    lcd2_command( data );
}
 
////////////////////////////////////////////////////////////////////////////////
// Schreibt einen String auf das LCD
 
void lcd2_string( const char *data )
{
    while( *data != '\0' )
        lcd2_data( *data++ );
}
 
void lcd2_num(uint8_t number) {
    char buffer[4];
    itoa(number, buffer, 10);
    uint8_t num_digits = strlen(buffer);
    for (uint8_t i=0; i<(3-num_digits); i++)
        lcd2_data('0');
    lcd2_string(buffer);
}

void lcd2_bit(uint8_t expr) {
	if (expr)
		lcd2_data('1');
	else
		lcd2_data('0');
}

void lcd2_byte(uint8_t byte) {
	for (uint8_t i=0; i<8; i++) {
		lcd2_bit(byte & 0b10000000);
		byte <<= 1;
	}
}


////////////////////////////////////////////////////////////////////////////////
// Schreibt ein Zeichen in den Character Generator RAM
 
void lcd2_generatechar( uint8_t code, const uint8_t *data )
{
    // Startposition des Zeichens einstellen
    lcd2_command( LCD2_SET_CGADR | (code<<3) );
 
    // Bitmuster bertragen
    for ( uint8_t i=0; i<8; i++ )
    {
        lcd2_data( data[i] );
    }
}

// Ansteuerung eines HD44780 kompatiblen LCD im 4-Bit-Interfacemodus
// http://www.mikrocontroller.net/articles/AVR-GCC-Tutorial/LCD-Ansteuerung
//

/*
LCD1 portmap
BIT0 -> PC0
BIT1 -> PC1
BIT2 -> PC2
BIT3 -> PC3
RS   -> PC4
EN   -> PC5
 */
 
/*
LCD2 portmap
BIT0 -> PD5
BIT1 -> PD7
BIT2 -> PD6
BIT3 -> PD1
RS   -> PD0
EN   -> PD4
 */

#ifndef LCD2_ROUTINES_H
#define LCD2_ROUTINES_H
 
////////////////////////////////////////////////////////////////////////////////
// Hier die verwendete Taktfrequenz in Hz eintragen, wichtig!
 
#ifndef F_CPU
#define F_CPU 20000000
#endif
 
////////////////////////////////////////////////////////////////////////////////
// Pinbelegung fr das LCD, an verwendete Pins anpassen
// Alle LCD Pins mssen an einem Port angeschlossen sein und die 4
// Datenleitungen mssen auf aufeinanderfolgenden Pins liegen
 
//  LCD DB4-DB7 <-->  PORTD Bit PD0-PD3
#define LCD2_PORT      PORTD
#define LCD2_DDR       DDRD
#define LCD2_BIT0     PD5
#define LCD2_BIT1     PD7
#define LCD2_BIT2     PD6
#define LCD2_BIT3     PD1
 
//  LCD RS      <-->  PORTD Bit PD4     (RS: 1=Data, 0=Command)
#define LCD2_RS        PD0
 
//  LCD EN      <-->  PORTD Bit PD5     (EN: 1-Impuls fr Daten)
#define LCD2_EN        PD4
 
////////////////////////////////////////////////////////////////////////////////
// LCD Ausfhrungszeiten (MS=Millisekunden, US=Mikrosekunden)
 
#define LCD2_BOOTUP_MS           15
#define LCD2_ENABLE_US           20
#define LCD2_WRITEDATA_US        46
#define LCD2_COMMAND_US          42
 
#define LCD2_SOFT_RESET_MS1      5
#define LCD2_SOFT_RESET_MS2      1
#define LCD2_SOFT_RESET_MS3      1
#define LCD2_SET_4BITMODE_MS     5
 
#define LCD2_CLEAR_DISPLAY_MS    2
#define LCD2_CURSOR_HOME_MS      2
 
////////////////////////////////////////////////////////////////////////////////
// Zeilendefinitionen des verwendeten LCD
// Die Eintrge hier sollten fr ein LCD mit einer Zeilenlnge von 16 Zeichen passen
// Bei anderen Zeilenlngen mssen diese Eintrge angepasst werden
 
#define LCD2_DDADR_LINE1         0x00
#define LCD2_DDADR_LINE2         0x40
#define LCD2_DDADR_LINE3         0x10
#define LCD2_DDADR_LINE4         0x50
 
////////////////////////////////////////////////////////////////////////////////
// Initialisierung: muss ganz am Anfang des Programms aufgerufen werden.
void lcd2_init( void );
 
////////////////////////////////////////////////////////////////////////////////
// LCD lschen
void lcd2_clear( void );
 
////////////////////////////////////////////////////////////////////////////////
// Cursor in die 1. Zeile, 0-te Spalte
void lcd2_home( void );
 
////////////////////////////////////////////////////////////////////////////////
// Cursor an eine beliebige Position 
void lcd2_setcursor( uint8_t spalte, uint8_t zeile );
 
////////////////////////////////////////////////////////////////////////////////
// Ausgabe eines einzelnen Zeichens an der aktuellen Cursorposition 
void lcd2_data( uint8_t data );
 
////////////////////////////////////////////////////////////////////////////////
// Ausgabe eines Strings an der aktuellen Cursorposition 
void lcd2_string( const char *data );
void lcd2_num(uint8_t number);
void lcd2_bit(uint8_t truth);
////////////////////////////////////////////////////////////////////////////////
// Definition eines benutzerdefinierten Sonderzeichens.
// data muss auf ein Array[5] mit den Spaltencodes des zu definierenden Zeichens
// zeigen
void lcd2_generatechar( uint8_t code, const uint8_t *data );
 
////////////////////////////////////////////////////////////////////////////////
// Ausgabe eines Kommandos an das LCD.
void lcd2_command( uint8_t data );
 
 
////////////////////////////////////////////////////////////////////////////////
// LCD Befehle und Argumente.
// Zur Verwendung in LCD2_command
 
// Clear Display -------------- 0b00000001
#define LCD2_CLEAR_DISPLAY       0x01
 
// Cursor Home ---------------- 0b0000001x
#define LCD2_CURSOR_HOME         0x02
 
// Set Entry Mode ------------- 0b000001xx
#define LCD2_SET_ENTRY           0x04
 
#define LCD2_ENTRY_DECREASE      0x00
#define LCD2_ENTRY_INCREASE      0x02
#define LCD2_ENTRY_NOSHIFT       0x00
#define LCD2_ENTRY_SHIFT         0x01
 
// Set Display ---------------- 0b00001xxx
#define LCD2_SET_DISPLAY         0x08
 
#define LCD2_DISPLAY_OFF         0x00
#define LCD2_DISPLAY_ON          0x04
#define LCD2_CURSOR_OFF          0x00
#define LCD2_CURSOR_ON           0x02
#define LCD2_BLINKING_OFF        0x00
#define LCD2_BLINKING_ON         0x01
 
// Set Shift ------------------ 0b0001xxxx
#define LCD2_SET_SHIFT           0x10
 
#define LCD2_CURSOR_MOVE         0x00
#define LCD2_DISPLAY_SHIFT       0x08
#define LCD2_SHIFT_LEFT          0x00
#define LCD2_SHIFT_RIGHT         0x04
 
// Set Function --------------- 0b001xxxxx
#define LCD2_SET_FUNCTION        0x20
 
#define LCD2_FUNCTION_4BIT       0x00
#define LCD2_FUNCTION_8BIT       0x10
#define LCD2_FUNCTION_1LINE      0x00
#define LCD2_FUNCTION_2LINE      0x08
#define LCD2_FUNCTION_5X7        0x00
#define LCD2_FUNCTION_5X10       0x04
 
#define LCD2_SOFT_RESET          0x30
 
// Set CG RAM Address --------- 0b01xxxxxx  (Character Generator RAM)
#define LCD2_SET_CGADR           0x40
 
#define LCD2_GC_CHAR0            0
#define LCD2_GC_CHAR1            1
#define LCD2_GC_CHAR2            2
#define LCD2_GC_CHAR3            3
#define LCD2_GC_CHAR4            4
#define LCD2_GC_CHAR5            5
#define LCD2_GC_CHAR6            6
#define LCD2_GC_CHAR7            7
 
// Set DD RAM Address --------- 0b1xxxxxxx  (Display Data RAM)
#define LCD2_SET_DDADR           0x80
 
#endif 

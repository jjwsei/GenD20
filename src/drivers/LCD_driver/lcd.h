#ifndef LCD_H
#define LCD_H

#define LCD_ADDRESS	0x27 //0x3F // Use 0x4E for PCF8574  and  0x7E for PCF8574A 

#define BACKLIGHT_OFF	0x00
#define BACKLIGHT_ON	0x08

// LCD Commands
// ---------------------------------------------------------------------------
#define LCD_CLEARDISPLAY        0x01
#define LCD_RETURNHOME          0x02
#define LCD_ENTRYMODESET        0x04
#define LCD_DISPLAYCONTROL      0x08
#define LCD_CURSORSHIFT         0x10
#define LCD_FUNCTIONSET         0x20
#define LCD_SETCGRAMADDR        0x40
#define LCD_SETDDRAMADDR        0x80

// flags for display entry mode
// ---------------------------------------------------------------------------
#define LCD_ENTRYRIGHT          0x00
#define LCD_ENTRYLEFT           0x02
#define LCD_ENTRYSHIFTINCREMENT 0x01
#define LCD_ENTRYSHIFTDECREMENT 0x00

// flags for display on/off and cursor control
// ---------------------------------------------------------------------------
#define LCD_DISPLAYON           0x04
#define LCD_DISPLAYOFF          0x00
#define LCD_CURSORON            0x02
#define LCD_CURSOROFF           0x00
#define LCD_BLINKON             0x01
#define LCD_BLINKOFF            0x00

// flags for display/cursor shift
// ---------------------------------------------------------------------------
#define LCD_DISPLAYMOVE         0x08
#define LCD_CURSORMOVE          0x00
#define LCD_MOVERIGHT           0x04
#define LCD_MOVELEFT            0x00

// flags for function set
// ---------------------------------------------------------------------------
#define LCD_8BITMODE            0x10
#define LCD_4BITMODE            0x00
#define LCD_2LINE               0x08
#define LCD_1LINE               0x00
#define LCD_5x10DOTS            0x04
#define LCD_5x8DOTS             0x00

#define LCD_BACKLIGHT   		0xFF

// Define COMMAND and DATA LCD Rs (used by send method).
// ---------------------------------------------------------------------------
#define COMMAND                 0
#define DATA                    1
#define FOUR_BITS               2

#define HOME_CLEAR_EXEC      	3

#define ROW1			0
#define ROW2			1
#define ROW3			2
#define ROW4			3
   
void LCDSetBacklight(uint8_t);
void LCDInit(uint8_t cols, uint8_t lines, uint8_t dotsize, struct i2c_master_module);
void LCDClear(void);
void LCDWrite(uint8_t);
void LCDHome(void);
void LCDSetCursor(uint8_t, uint8_t);
void LCDReset(void);
void LCDprintf(const char * str, ...) __attribute__ ((format (gnu_printf, 1, 2)));

#endif

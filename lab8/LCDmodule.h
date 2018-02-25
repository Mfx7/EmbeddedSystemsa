/* File: LCDmodule.h
Contains function prototypes and definitions for LCD module interface.
User must define two parameters (see _LCDport and _LCDinitCom below) to set
the MCU port for LCD interface and to select the LCD display type (actual LCD
hardware or Proteus simulation model).

Assignment of the _LCDport pins to the LCD module connections:
   _LCDport pins      LCD module connections (pin #):
   -------------      -------------------------------
   _LCDport[3:0]  ->  D7..D4 (11..14)  4-bit parallel data
   _LCDport[4]    ->  RegSel (4)  0=>instruction, 1=>data
   _LCDport[5]    ->  RnW    (5)  0=>write, 1=>read
   _LCDport[6]    ->  Enable (6)  0-1-0 pulse, T-cycle > 1200 ns
   _LCDport[7]    ->  No connection. Used as a marker output.

Port-L pins, PL0..PL7 are connected to the dual raw header pins 49..42
on the Arduino Mega (ATmega2560) board.
*/
// Define the MCU port used for LCD interface.
// User must enable all port pins for output before calling LCD_Init().
#define  _LCDport  PORTB
// Actual PCM1602B LCD display and LM016L 2x16 LCD display in Proteus
// simulation require different initialization commands.
//#define  _LCDinitCom  0x03  // 0x03 for PCM1602B LCD display
#define  _LCDinitCom  0x02  // 0x02 for LM016L in Proteus

// LCD_Init() must be called once before any operation on the LCD module.
// It takes 4 ms to complete the initialization.
void LCD_Init(void);

// LCD_WriteInst(.) sends an instruction byte to the LCD module.
// LCD controller needs 40 us to complete the instruction.
void LCD_WriteInst(unsigned char WriteByte);

// LCD_WriteData(.) sends a display data byte to the LCD module.
// LCD controller needs 50 us to process the data.
void LCD_WriteData(unsigned char WriteByte);

// Function to clear LCD display. It takes 4 ms to complete the clear operation.
// It is faster to overwrite the previously written LCD text by sending required
// number of data bytes.
void LCD_Clear(void);

// LCD_MoveCursor(..) positions the cursor on the specified Line (1 or 2)
// and the specified Column (1 thorough 16):
// LCD controller needs ~40 us to complete the instruction.
void LCD_MoveCursor(unsigned char Line,
                    unsigned char Column);

// LCD_WriteString(.) writes an array of characters starting at the current
// cursor position on the LCD display.
// LCD controller needs ~50 us to display each character in the String.
void LCD_WriteString(char *String);

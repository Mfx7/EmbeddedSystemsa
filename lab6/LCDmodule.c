/* File LCDmodule.c:  Functions for LCD module interface.
User must define _LCDport in LCDmodule.h header file and set all of the
port pins for output before calling any of the LCD module functions.
See LCDmodule.h for information on port connections to the LCD module.
*/

#include <avr/io.h>
#include <util/delay.h>
#include "LCDmodule.h"

// The MSB of _LCDport is used as a marker for monitoring signals on a scope.
// LCD_MarkBit is the global variable used to enable the marker output.
unsigned char LCD_MarkBit = 0x00;
void LCD_SetMarker(unsigned char MarkerEn)
// Enables MSB of _LCDport as marker bit when MarkerEn is not 0.
{
  if (MarkerEn)
    LCD_MarkBit = 0x80;
  else
    LCD_MarkBit = 0x00;
  return;
}

void LCD_Init(void)
/* Initialize the LCD display for 4-bit parallel data interface. */
{
// Set 4-bit parallel interface with the LCD controller
  _LCDport = LCD_MarkBit | _LCDinitCom;
  _LCDport = LCD_MarkBit | _LCDinitCom;
  _LCDport = LCD_MarkBit | 0x40 | _LCDinitCom;  // Set Enable
  _LCDport = LCD_MarkBit | 0x40 | _LCDinitCom;
  _LCDport = LCD_MarkBit | _LCDinitCom;         // Clear Enable
  _LCDport = LCD_MarkBit | _LCDinitCom;
  _LCDport = 0x00;  // clear port output
  _delay_us(50);
// Set function (001N LFxx): N=0=> 4-bit data, L=1=> 2 display lines,
// F=0=> 5x8 dot characters
  LCD_WriteInst(0x28);
// Set function again (required for PCM1602B):
//  LCD_WriteInst(0x28);
// Set entry mode (0000 01MS): M=1=> increment cursor address (move right),
// S=0=> no display shift
  LCD_WriteInst(0x06);
// ON/OFF control (0000 1DCB): Display ON, cursor OFF, cursor blink OFF:
  LCD_WriteInst(0x0C);
// Clear display and move cursor home:
  LCD_Clear();
  return;
}

void LCD_WriteInst(unsigned char WriteByte)
/* When 4-bit interface is used, MSBs of the instruction/data bytes are sent
   first. Port output is set twice to satisfy the LCD controller timing.
   The 40 us delay at the end is the minimum time required by the LCD module
   to complete the internal operations after receiving the instructions.
   Note that, clear display and return home instructions take an additional
   1.5ms to execute. This additional delay must be provided in the caller.
*/
{
  unsigned char  MSnibble, LSnibble;

  MSnibble = WriteByte >> 4;
  LSnibble = WriteByte & 0x0F;
  if (LCD_MarkBit)
  { MSnibble |= 0x80;
    LSnibble |= 0x80;
  }
  _LCDport = MSnibble;
  _LCDport = MSnibble;
//  _delay_us(1);
  _LCDport = 0x40 | MSnibble;  // Set Enable
  _LCDport = 0x40 | MSnibble;
  _LCDport = MSnibble;         // Clear Enable
  _LCDport = MSnibble;
  _LCDport = LSnibble;
  _LCDport = LSnibble;
  _LCDport = 0x40 | LSnibble;  // Set Enable
  _LCDport = 0x40 | LSnibble;
  _LCDport = LSnibble;         // Clear Enable
  _LCDport = LSnibble;
  _LCDport = 0x00;  // clear port output
  _delay_us(40);
  return;
}

void LCD_WriteData(unsigned char WriteByte)
/* When 4-bit interface is used, MSBs of the instruction/data bytes are sent
   first. Port output is set twice to satisfy the LCD controller timing.
   The 50 us delay at the end is required by the LCD module to complete the
   internal operations after receiving the data.
*/
{
  unsigned char  MSnibble, LSnibble;

// Set the register select bit, RS=1, when sending display data to LCD:
  MSnibble = 0x10 | (WriteByte >> 4);
  LSnibble = 0x10 | (WriteByte & 0x0F);
  if (LCD_MarkBit)
  { MSnibble |= 0x80;
    LSnibble |= 0x80;
  }
// Write 4 MSBs:
  _LCDport = MSnibble;
  _LCDport = MSnibble;
  _LCDport = 0x40 | MSnibble;  // Set Enable
  _LCDport = 0x40 | MSnibble;
  _LCDport = MSnibble;         // Clear Enable
  _LCDport = MSnibble;
// Write 4 LSBs:
  _LCDport = LSnibble;
  _LCDport = LSnibble;
  _LCDport = 0x40 | LSnibble;  // Set Enable
  _LCDport = 0x40 | LSnibble;
  _LCDport = LSnibble;         // Clear Enable
  _LCDport = LSnibble;
  _LCDport = 0x00;  // clear port output
  _delay_us(50);
  return;
}

void LCD_Clear(void)
/* Clears display and resets cursor position. */
{
// Clear display (0000 0001):
  LCD_WriteInst(0x01);
  _delay_ms(2);
// Return home (0000 001x):
  LCD_WriteInst(0x02);
  _delay_ms(2);
  return;
}

void LCD_MoveCursor(unsigned char Line,
                    unsigned char Column)
/* Sets cursor position as specified by Line and Column:
   Line=1=> first line,  Column=1=> first character on the left */
{
  unsigned char  InstByte;

  if (Line == 1)
    InstByte = 0x80;
  else
    InstByte = 0xC0;
  InstByte |= (Column - 1) & 0x0F;
  LCD_WriteInst(InstByte);
  return;
}

void LCD_WriteString(char *String)
/* Displays a string of characters. End of string character is 0x00. */
{
  char  *pChar;

  pChar = String;
  while (*pChar != 0x00)
  { LCD_WriteData(*pChar);
    pChar ++;
  }
  return;
}



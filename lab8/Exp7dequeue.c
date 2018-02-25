#include <stdio.h>
#include <avr/io.h>
#include <util/delay.h>
#include "LCDmodule.h"

#define LCDbufferSize    32

// Global variables for data transfer to Timer-2 ISR:
unsigned char  LCDbuffer[LCDbufferSize];
unsigned char  Nbyte = 0;

void DeQueueLCDbuffer(void)
/* If there are queued data in LCDbuffer (Nbyte > 0), then sends a
   single byte to the LCD module. If the buffer is empty (Nbyte = 0),
   then returns without doing anything. */
{
  static unsigned char  *pLCDout = LCDbuffer;
  static unsigned char  Bcount = LCDbufferSize;
  unsigned char  ByteOut;

  if (Nbyte > 0)
// First send the 4 most significant bits of the LCD data:
  { ByteOut = *pLCDout >> 4;
// Set the register select bit, RS=1, when sending display data to LCD:
    if ( (*pLCDout & 0x80) == 0 )  ByteOut |= 0x10;
// Write 4 MSBs:
    _LCDport = ByteOut;
    _LCDport = 0x40 | ByteOut;  // Set Enable
    _LCDport = ByteOut;         // Clear Enable
    ByteOut = (ByteOut & 0xF0) | (*pLCDout & 0x0F);
    _LCDport = ByteOut;
    _LCDport = 0x40 | ByteOut;  // Set Enable
    _LCDport = ByteOut;         // Clear Enable
    _LCDport = 0x00;  // clear port output
// Update the LCDbuffer status:
    Bcount --;        // decrement buffer position counter
    if (Bcount == 0)  // check if reached end of buffer array
    { pLCDout = LCDbuffer;     // go back to the first element
      Bcount = LCDbufferSize;  // reload buffer position counter
    }
    else
      pLCDout ++;  // increment dequeue buffer pointer
    Nbyte --;      // decrement number of stored bytes
  }
    if(Nbyte == 0)
        TIMSK2 &= ~(1<<OCIE2A);
  return;
}

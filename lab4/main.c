#include<stdio.h>
#include <avr/io.h>
#include <util/delay.h>
#include "LCDmodule.h"

void PrintByte(char *OutputString,char *PromptString,char ByteVar);

int main(void)
{
    DDRD |= 0xF0;
    DDRB |= 0xFF;

    char LCDtext[16];
    unsigned char Abyte = 0;
    unsigned char current,previos;
    unsigned char flag = 0;
    LCD_Init();
    LCD_Clear();
    previos = PIND;

    LCD_MoveCursor(1,1);
    LCD_WriteString("Atten= 0");

    while(1)
    {
        current = PIND;

        if ( ( current & _BV(PIND0) ) == 0x00 && ( previos & _BV(PIND0)  ) == 0x01 )
        {
                if(Abyte > 1){
                    Abyte--;
                    flag = 1;
                }
        }

        if ( ( current & _BV(PIND1) ) == 0x00 && ( previos & _BV(PIND1)  ) == 0x02 )
        {
                if(Abyte < 11){
                    Abyte++;
                    flag = 1;
                }
        }

//        PORTD |= _BV(PIND4);
//        PrintByte(LCDtext, "Atten=", Abyte);
//        PORTD &= ~(_BV(PIND4));
//
//        PORTD |= _BV(PIND5);
//        LCD_MoveCursor(1,1);
//        LCD_WriteString(LCDtext);
//        PORTD &= ~(_BV(PIND5));


        if(flag == 1){
            PORTD |= _BV(PIND4);
            PrintByte(LCDtext, "", Abyte);
            PORTD &= ~(_BV(PIND4));
            PORTD |= _BV(PIND5);
            LCD_MoveCursor(1,7);
            LCD_WriteString(LCDtext);

            flag = 0;
            PORTD &= ~(_BV(PIND5));
        }


        previos = current;
        _delay_us(500);
    };
return 0;
}

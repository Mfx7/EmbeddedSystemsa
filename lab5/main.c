#include<stdio.h>
#include <avr/io.h>
#include <util/delay.h>
#include "LCDmodule.h"

void PrintByte(char *OutputString,char *PromptString,char ByteVar);

int main(void)
{
    DDRD |= 0xF0;
    DDRB |= 0xFF;
    DDRC |= 0xF0;

    // ## ADC init
    PRR    &= ~(1<<PRADC);
    ADMUX  |= (1<<REFS0) | (1<<ADLAR);
    ADMUX  |= (1<<MUX0);    // select ADC1 as source
    ADCSRA |= (1<<ADEN);
    ADCSRA |= (1<<ADPS2) | (1<<ADPS1);

    char LCDtext[16];
    unsigned char Abyte = 0, Atten = 0;
    unsigned char current,previos;
    previos = PIND;

    LCD_Init();
    LCD_Clear();

    LCD_MoveCursor(1,1);
    LCD_WriteString("Atten= 0");

    while(1)
    {
        PORTD |= 0x10;
        current = PIND;

        ADCSRA |= (1<<ADSC);
        while( ((ADCSRA) &(1<<ADSC)) == (1<<ADSC));
        PORTD &= ~(0x10);
        Abyte = ADCH;

        if ( ( current & _BV(PIND0) ) == 0x00 && ( previos & _BV(PIND0)  ) == 0x01 )
        {
                if(Atten > 1) Atten--;
        }
        if ( ( current & _BV(PIND1) ) == 0x00 && ( previos & _BV(PIND1)  ) == 0x02 )
        {
            if(Atten < 6) Atten++;
        }
//      second part
//        if(Atten != 0)
//            Abyte = (Abyte>>(Atten-1));
//        else
//            Abyte = Abyte;
//
//          PORTB = Abyte;


        PrintByte(LCDtext, "", Abyte);

        LCD_MoveCursor(1,7);
        LCD_WriteString(LCDtext);

        previos = current;
        _delay_us(650);

    };
return 0;
}

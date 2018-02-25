#include <stdio.h>
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include "LCDmodule.h"

void PrintByte(char *OutputString,char *PromptString,char ByteVar);

//ISR(TIMER1_COMPA_vect)
//{
//    PORTD |= _BV(PORTD4);
//    static unsigned char Abyte = 0;
//    char LCDtext[16];
//
//    PORTD |= _BV(PORTD5);
//
//    ADCSRA |= (1<<ADSC);
//    while( ((ADCSRA) &(1<<ADSC)) == (1<<ADSC));
//    PORTD &= ~_BV(PORTD5);
//    Abyte = ADCH;
//    PrintByte(LCDtext, "", Abyte);
//
//    LCD_MoveCursor(1,1);
//    LCD_WriteString(LCDtext);
//
//
//    OCR0A = Abyte;   // set timer0 pwm as ADC output
//
//    PORTD &= ~_BV(PORTD4);
//}

ISR(TIMER1_COMPA_vect)
{
    PORTD |= _BV(PORTD4);

    char LCDtext[16];
    static unsigned char Abyte = 0, Atten = 0;
    static unsigned char current = 0,previos = 0;

    current = PIND;

    PORTD |= _BV(PORTD5);
    // read ADC
    ADCSRA |= (1<<ADSC);
    while( ((ADCSRA) &(1<<ADSC)) == (1<<ADSC));
    PORTD &= ~_BV(PORTD5);

    Abyte = ADCH;

    if ( ( current & _BV(PIND0) ) == 0x00 && ( previos & _BV(PIND0)  ) == 0x01 )
    {
            if(Atten > 1) Atten--;
    }
    if ( ( current & _BV(PIND1) ) == 0x00 && ( previos & _BV(PIND1)  ) == 0x02 )
    {
        if(Atten < 6) Atten++;
    }

    if(Atten != 0)
        Abyte = (Abyte>>(Atten-1));
    else
        Abyte = Abyte;

    PrintByte(LCDtext, "", Abyte);

    LCD_MoveCursor(1,1);
    LCD_WriteString(LCDtext);
    previos = current;

    OCR0A = Abyte;   // set timer0 pwm as ADC output

    PORTD &= ~_BV(PORTD4);
}

int main(void)
{
    DDRD |= 0xF0;
    DDRB |= 0xFF;
    DDRC |= 0xF0;

    // ## ADC init
    ADMUX  |= (1<<REFS0) | (1<<ADLAR);
    ADMUX  |= (1<<MUX0);    // select ADC1 as source
    ADCSRA |= (1<<ADEN);
    ADCSRA |= (1<<ADPS2) | (1<<ADPS1);

    // ## set timer 0 -> prescalar ->1,  fast pwm WGM0[2:0] = 011 , COM0A[1:0] = 10
    TCCR0A |= (1<<COM0A1) | (0<<COM0A0) | (1<<WGM01) | (1<<WGM00);
    TCCR0B |= (0<<WGM02)  | (1<<CS00);

    // ## set timer 1  -> prescalar -> 8 CS11=1, WGM1[3:0] = 0100 , enable interrupt for OCIE1A
    TCCR1B |= (1<<WGM12)  | (1<<CS11);
    TIMSK1 |= (1<<OCIE1A);
    OCR1A  = 0x03E8;

    LCD_Init();
    LCD_Clear();

    // Enable global interrupts:
    sei();
    while(1);

    return 0;
}

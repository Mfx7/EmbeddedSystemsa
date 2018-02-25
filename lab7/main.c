#include <stdio.h>
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include "LCDmodule.h"

#define LCDbufferSize    32

void PrintByte(char *OutputString,char *PromptString,char ByteVar);
void DeQueueLCDbuffer(void);
void EnQueueLCDbuffer(unsigned char ByteIn);

// Global variables for data transfer to Timer-2 ISR:
extern unsigned char  LCDbuffer[LCDbufferSize];
extern unsigned char  Nbyte;
unsigned char Atten = 1, prv = 0;
unsigned char LCDaccess = 0;

ISR(TIMER1_COMPA_vect)
{
    PORTD |= _BV(PORTD4);

    static unsigned char Abyte = 0;
    unsigned char *pText;
    char LCDtext[16];

    // read ADC
    ADCSRA |= (1<<ADSC);
    while( ((ADCSRA) &(1<<ADSC)) == (1<<ADSC));

    Abyte = ADCH;

    if(Atten != 0)
        Abyte = (Abyte>>(Atten-1));
    else
        Abyte = Abyte;


//    if(LCDaccess == 0)
//    {
//        PrintByte(LCDtext, "", Abyte);
//        LCD_MoveCursor(1,1);
//        LCD_WriteString(LCDtext);
//    }

    if(LCDaccess == 0){

        PrintByte(LCDtext, "", Abyte);
        EnQueueLCDbuffer(0x80);
        pText = (unsigned char *)LCDtext;
        while (*pText != 0x00)
        {
            EnQueueLCDbuffer(*pText);
            pText ++;
        };
    }

    OCR0A = Abyte;   // set timer0 pwm as ADC output

    PORTD &= ~_BV(PORTD4);

}

ISR(TIMER2_COMPA_vect)
{
    PORTD |= _BV(PORTD5); // Set second ISR timing marker.
    if(Nbyte != LCDbufferSize)
        DeQueueLCDbuffer();

    PORTD &= ~_BV(PORTD5); // Clear second ISR timing marker.
    TCNT2 = 0;
}

void EnQueueLCDbuffer(unsigned char ByteIn)
{
    static unsigned char *pEnq = LCDbuffer;
    static unsigned char *pEOB = LCDbuffer+LCDbufferSize;

    if(Nbyte != LCDbufferSize)
    {
        *pEnq = ByteIn;
        pEnq++;
        Nbyte++;
        if(pEnq == pEOB)
            pEnq = LCDbuffer;
    }

    if(Nbyte == 1)
        TIMSK2 |= (1<<OCIE2A);

}

int main(void)
{
    unsigned char current = 0,previos =0 ;
    unsigned char *pText;
    char LCDtext[16];

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

    // ## set timer 1-2 -> prescalar ->8,
    TCCR2A |= (1<<WGM21);
    TCCR2B |= (1<<CS21);
    TIMSK2 |= (1<<OCIE2A);
    OCR2A   = 50;

    LCD_Init();
    LCD_Clear();

    sei();

    while(1)
    {
        current = PIND;
        if ( ( current & _BV(PIND0) ) == 0x00 && ( previos & _BV(PIND0)  ) == 0x01 )
        {
                if(Atten > 1) Atten--;
        }
        if ( ( current & _BV(PIND1) ) == 0x00 && ( previos & _BV(PIND1)  ) == 0x02 )
        {
            if(Atten < 11) Atten++;
        }

//        PrintByte(LCDtext, "", Atten);
//        LCDaccess = 1;
//        LCD_MoveCursor(2,1);
//        LCD_WriteString(LCDtext);
//        LCDaccess = 0;

        if(prv != Atten){
                LCDaccess = 1;
                PrintByte(LCDtext, "", Atten);
                EnQueueLCDbuffer(0xC0);
                // LCD_WriteString(LCDtext); // replaced by the following
                pText = (unsigned char *)LCDtext; // initialize pointer to output text
                // Copy text output to LCDbuffer:
                while (*pText != 0x00)
                {
                    EnQueueLCDbuffer(*pText);
                    pText ++;
                };
                LCDaccess = 0;

        }
        prv = Atten;
        previos = current;
    }


    return 0;
}

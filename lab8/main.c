#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include "LCDmodule.h"

#define LCDbufferSize    32
#define USARTbufferSize  64
#define USART_BAUDRATE 9600
#define BAUD_PRESCALE (((F_CPU / (USART_BAUDRATE * 16UL))) - 1)

void PrintByte(char *OutputString,char *PromptString,char ByteVar);
void DeQueueLCDbuffer(void);
void EnQueueLCDbuffer(unsigned char ByteIn);
void DeQueueUSARTbuffer(void);
void EnQueueUSARTbuffer(unsigned char ByteIn);

// Global variables for data transfer to Timer-2 ISR:
extern unsigned char  LCDbuffer[LCDbufferSize];
extern unsigned char  Nbyte;
unsigned char Atten = 1;
unsigned char LCDaccess = 0;

// Global variables for USART
unsigned char  USARTbuffer[USARTbufferSize];
unsigned char  Numbyte = 0;

ISR(TIMER1_COMPA_vect)
{

    static unsigned char Abyte = 0 , counter = 0;
    unsigned char *pText;
    char LCDtext[16];

    // read ADC
    ADCSRA |= (1<<ADSC);
    while( ((ADCSRA) &(1<<ADSC)) == (1<<ADSC));

    Abyte = ADCH;

    if((Atten%2)== 1)
    {
        Abyte=Abyte>>(Atten-1)/2;
    }
    else
    {
        Abyte=(Abyte>>((Atten)/2))+(Abyte>>((Atten)/2 +1));
    }
    PrintByte(LCDtext, "", Abyte);
    EnQueueLCDbuffer(0x80);
    pText = (unsigned char *)LCDtext;
    while (*pText != 0x00)
    {
        EnQueueLCDbuffer(*pText);
        pText ++;
    };

    if(counter == 10)
    {
        PORTD |= _BV(PORTD4);
        pText = (unsigned char *)LCDtext;
        EnQueueUSARTbuffer(0x20);
        while (*pText != 0x00)
        {
            EnQueueUSARTbuffer(*pText);
            pText ++;
        };

        counter = 0;
        PORTD &= ~_BV(PORTD4);
    }

    counter++;
    OCR0A = Abyte;   // set timer0 pwm as ADC output



}

ISR(TIMER2_COMPA_vect)
{
    TCNT2=0;
    DeQueueLCDbuffer();
}

ISR(USART_RX_vect)
{

    static unsigned char prvAtten = 0, data;
    char LCDtext[16];
    unsigned char *pText;

    data = UDR0;
    // determine value of Atten
    if( ( data & 0xDF) == 0x55)
        Atten = (Atten>1) ? Atten-1 : Atten;
    else if ( ( data & 0xDF) == 0x44)
        Atten = (Atten<11) ? Atten+1 : Atten;

    // if previos different from current Arren
    if(prvAtten != Atten)
    {
        PrintByte(LCDtext, "Atten= ", Atten);
        EnQueueLCDbuffer(0xC0);
        EnQueueUSARTbuffer(0x0D);
        pText = (unsigned char *)LCDtext;
        while (*pText != 0x00)
        {
            EnQueueLCDbuffer(*pText);
            EnQueueUSARTbuffer(*pText);
            pText ++;
        };
    }
    prvAtten = Atten;

}

ISR(USART_UDRE_vect)
{
    PORTD |= _BV(PORTD5); // Set second ISR timing marker.
    DeQueueUSARTbuffer();
    PORTD &= ~_BV(PORTD5); // Clear second ISR timing marker.
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
    TIMSK2 |= (1<<OCIE2A);

}

void EnQueueUSARTbuffer(unsigned char ByteIn)
{
    static unsigned char *pEnq = USARTbuffer;
    static unsigned char *pEOB = USARTbuffer+USARTbufferSize;

    if(Numbyte != USARTbufferSize)
    {
        *pEnq = ByteIn;
        pEnq++;
        Numbyte++;
        if(pEnq == pEOB)
            pEnq = USARTbuffer;
    }
    UCSR0B |= (1<<UDRIE0);
}

void DeQueueUSARTbuffer()
{
    static unsigned char *pDeq = USARTbuffer;
    static unsigned char *pEOB = USARTbuffer+USARTbufferSize;

    if(Numbyte != 0)
    {
        UDR0 = *pDeq ;
        pDeq++;
        Numbyte--;
        if(pDeq == pEOB)
            pDeq = USARTbuffer;
    }
    if(Numbyte == 0)
        UCSR0B &= ~(1<<UDRIE0);
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
    TCCR0A |= (1<<COM0A1) | (1<<WGM01) | (1<<WGM00);
    TCCR0B |= (1<<CS00);

    // ## set timer 1  -> prescalar -> 8 CS11=1, WGM1[3:0] = 0100 , enable interrupt for OCIE1A
    TCCR1B |= (1<<WGM12)  | (1<<CS11);
    TIMSK1 |= (1<<OCIE1A);
    OCR1A  = 0x03E8;

    // ## set timer 1-2 -> prescalar ->8,
    TCCR2A |= (1<<WGM21);
    TCCR2B |= (1<<CS21);
    TIMSK2 |= (1<<OCIE2A);
    OCR2A   = 50;

    // ## set USART registers
    UBRR0   = BAUD_PRESCALE; // baud rate 9600
    UCSR0B |= (1<<RXEN0)|(1<<TXEN0);
    UCSR0B |= (1<<RXCIE0) | (1<<UDRIE0); // enable Receiver complete and Data register empty
    UCSR0C |= (1<<UCSZ00) | (1<<UCSZ01); //= 011 for standard 8-bit data transmission


    LCD_Init();
    LCD_Clear();

    sei();

    while(1)
    {
//        PORTD |= _BV(PORTD4); // Set timing marker.
//        _delay_ms(50);
//        PORTD &= ~_BV(PORTD4); // Clear timing marker.
//        _delay_ms(50);
    }


    return 0;
}

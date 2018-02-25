#include <avr/io.h>
#include <util/delay.h>

void my_delay()
{
    int i;
    for(i=0;i<794;i++)
        __asm__ volatile ( "NOP" );
}

int main(void)
{
    unsigned char current,previos;
    unsigned char counter = 0;

    DDRD |= 0xFC;
    DDRB |= 0xFF;

    previos = PIND;
    PORTB = 0x01;
    while(1)
    {
        current = PIND;

        if(( current & _BV(PIND0) ) == 0x00 && ( previos & _BV(PIND0) ) == 0x00)
        {
            if(counter ==250){
                current = 0x00;
                previos = 0x01;
                counter = 0;
            }else{
                 counter++;
            }
        }

        // if button pressed
        if ( ( current & _BV(PIND0) ) == 0x00 && ( previos & _BV(PIND0)  ) == 0x01 )
        {
            if ( ( PIND & _BV(PIND1) ) == 0x00 )
                if(PORTB == 0x01){
                    PORTB = 0x01;
                    PORTB = 0x01;
                    PORTB = 0x80;
                }
                else{
                    PORTB = (PORTB>>1);
                }
            else
                if(PORTB == 0x80){
                    PORTB = 0x80;
                    PORTB = 0x80;
                    PORTB = 0x01;

                }
                else
                    PORTB = (PORTB<<1);
        }

        previos = current;
        my_delay();
    };
return 0;
}

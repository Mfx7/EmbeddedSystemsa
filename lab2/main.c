#include <avr/io.h>
#include <util/delay.h>

int main(void)
{
    DDRD |= 0xFC;
    DDRB |= 0xFF;

    PORTB = 0x01;
    while(1)
    {

        // if button pressed
        if ( ( PIND & _BV(PIND0) ) == 0x00 )
        {
            if ( ( PIND & _BV(PIND1) ) == 0x00 )
                if(PORTB == 0x01){
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
        else
        {
            PORTB = PORTB;
        }

    };
return 0;
}

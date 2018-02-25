#include <avr/io.h>
#include <util/delay.h>

int main(void)
{
    DDRB |= 0xFF;
    while(1)
    {
        PORTB = 0x0F;
        //PORTB = 0x0F;
        //PORTB = 0x0F;
        //_delay_ms(500);
        PORTB = 0x80;
        //_delay_ms(500);
    };
return 0;
}

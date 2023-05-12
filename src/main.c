#include <avr/io.h>
#include <util/delay.h>

#define stepPin 5
#define dirPin 2
#define dirPin2 4
//#define enPin 8

int main(void)
{
    DDRD |= (1 << DDD2) | (1 << DDD5) | (1 << DDD4)| (1 << DDD5); // Set dirPin and stepPin as output

    while (1) {
        //clockwise movement
        PORTD = 0b00010100; // Set dirPin HIGH to move in a particular direction
        for (int x = 0; x < 800; x++) {
            PORTD |= (1 << PORTD5)| (1 << PORTD6); // Set stepPin HIGH
            _delay_us(500);
            PORTD &= ~(1 << PORTD5)| ~(1 << PORTD6); // Set stepPin LOW
            _delay_us(500);
        }
        _delay_ms(1000); // One second delay
        PORTD = 0b00000000; // Set dirPin LOW to change direction of rotation
        for (int x = 0; x < 800; x++) {
            PORTD |= (1 << PORTD5)| (1 << PORTD6); // Set stepPin HIGH
            _delay_us(500);
            PORTD &= ~(1 << PORTD5)| ~(1 << PORTD6); // Set stepPin LOW
            _delay_us(500);
        }
        _delay_ms(1000);
    }
    return 0;
}
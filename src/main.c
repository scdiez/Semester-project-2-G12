#include <avr/io.h>
#include <util/delay.h>

#define stepPin 5
#define dirPin 2
//#define enPin 8

void move_motor1 (uint16_t, uint8_t);

int main(void)
{
    DDRD |= (1 << DDD2) | (1 << DDD5); // Set dirPin and stepPin as output
    DDRB |= (1 << DDB0); // Set enPin as output
    //PORTB &= ~(1 << PORTB0); // Set enPin LOW to enable the driver

    while (1) {
        move_motor1(800,1);
        _delay_ms(500);
        move_motor1(800,0);
        _delay_ms(500);
    }
    return 0;
}

void move_motor1 (uint16_t steps, uint8_t direction){
    if (direction){
        PORTD |= (1 << PORTD2); // Set dirPin HIGH to move clockwise
    }
    else{
        PORTD |= ~(1 << PORTD2); // Set dirPin LOW to move counterclockwise
    }
    for (int x = 0; x <= steps; x++) {
            PORTD |= (1 << PORTD5); // Set stepPin HIGH
            _delay_us(500);
            PORTD &= ~(1 << PORTD5); // Set stepPin LOW
            _delay_us(500);
        }
}
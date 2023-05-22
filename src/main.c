#include <avr/io.h>
#include <util/delay.h>

#define stepPin 5
#define dirPin 2

void move_clockwise(int);
void move_counter_clockwise(int);

int main(void)
{
    DDRD |= (1 << DDD2) | (1 << DDD5); // Set dirPin and stepPin as output

    while (1) {
        /*PORTD |= (1 << PORTD2); // Set dirPin HIGH to move in a particular direction
        for (int x = 0; x < 800; x++) {
            PORTD |= (1 << PORTD5); // Set stepPin HIGH
            _delay_us(500);
            PORTD &= ~(1 << PORTD5); // Set stepPin LOW
            _delay_us(500);
        }
        _delay_ms(1000); // One second delay
        PORTD &= ~(1 << PORTD2); // Set dirPin LOW to change direction of rotation
        for (int x = 0; x < 800; x++) {
            PORTD |= (1 << PORTD5); // Set stepPin HIGH
            _delay_us(500);
            PORTD &= ~(1 << PORTD5); // Set stepPin LOW
            _delay_us(500);*/
			move_clockwise(3000);
			_delay_ms(500);
			move_counter_clockwise(3000);
			_delay_ms(500);
        }


    return 0;
}

void move_clockwise(steps){
	 PORTD |= (1 << PORTD2); // Set dirPin HIGH to move in a particular direction
        for (int x = 0; x < steps; x++) {
            PORTD |= (1 << PORTD5); // Set stepPin HIGH
            _delay_us(100);
            PORTD &= ~(1 << PORTD5); // Set stepPin LOW
            _delay_us(100);
        }
}
void move_counter_clockwise(Steps){
	PORTD &= ~(1 << PORTD2); // Set dirPin LOW to change direction of rotation
        for (int x = 0; x < Steps; x++) {
            PORTD |= (1 << PORTD5); // Set stepPin HIGH
            _delay_us(500);
            PORTD &= ~(1 << PORTD5); // Set stepPin LOW
            _delay_us(500);
        }
        _delay_ms(1000);
}
#include <avr/io.h>
#include <util/delay.h>
#define stepPin2 6
#define dirPin2 4
#define stepPin1 5
#define dirPin1 2
#define enPin1 8
#define enPin2 9

void move_right(uint16_t);
void move_left(uint16_t);
void move_up(uint16_t);
void move_down(uint16_t);

int main(void)
{
    DDRD |= (1 << DDD4) | (1 << DDD6) | (1 << DDD2) | (1 << DDD5); // Set dirPin and stepPin as output
    DDRB |= (1 << DDB0); // Set enPin as output
    PORTB &= ~(1 << PORTB0); // Set enPin LOW to enable the driver
    DDRB |= (1 << DDB1); // Set enPin as output
    PORTB &= ~(1 << PORTB1); // Set enPin LOW to enable the driver
    while (1) {
        move_right(800);
    }
}
void move_right(uint16_t r_steps){
//counter, counter
        PORTD |= ~(1 << PORTD4); // Set dirPin HIGH to move in a particular direction
        PORTD |= ~(1 << PORTD2);
        for (int x = 0; x < r_steps; x++) {
            PORTD |= (1 << PORTD6);
            PORTD |= (1 << PORTD5); // Set stepPin HIGH
            _delay_us(500);
            PORTD &= ~(1 << PORTD6);
            PORTD &= ~(1 << PORTD5); // Set stepPin LOW
            _delay_us(500);
        }
}
void move_left(uint16_t l_steps){
//clock, clock 
        PORTD |= (1 << PORTD4); // Set dirPin HIGH to move in a particular direction
        PORTD |= (1 << PORTD2);
        for (int x = 0; x < l_steps; x++) {
            PORTD |= (1 << PORTD6);
            PORTD |= (1 << PORTD5); // Set stepPin HIGH
            _delay_us(500);
            PORTD &= ~(1 << PORTD6);
            PORTD &= ~(1 << PORTD5); // Set stepPin LOW
            _delay_us(500);
        }
}

void move_up(uint16_t u_steps){
//counter, clock 
        PORTD |= ~(1 << PORTD4); // Set dirPin HIGH to move in a particular direction
        PORTD |= (1 << PORTD2);
        for (int x = 0; x < u_steps; x++) {
            PORTD |= (1 << PORTD6);
            PORTD |= (1 << PORTD5); // Set stepPin HIGH
            _delay_us(500);
            PORTD &= ~(1 << PORTD6);
            PORTD &= ~(1 << PORTD5); // Set stepPin LOW
            _delay_us(500);
        }
}

void move_down(uint16_t d_steps){
//clock, counter 
        PORTD |= (1 << PORTD4); // Set dirPin HIGH to move in a particular direction
        PORTD |= ~(1 << PORTD2);
        for (int x = 0; x < d_steps; x++) {
            PORTD |= (1 << PORTD6);
            PORTD |= (1 << PORTD5); // Set stepPin HIGH
            _delay_us(500);
            PORTD &= ~(1 << PORTD6);
            PORTD &= ~(1 << PORTD5); // Set stepPin LOW
            _delay_us(500);
        }
}
#include <avr/io.h>
#include <util/delay.h>
#define stepPin2 6
#define dirPin2 4
#define stepPin1 5
#define dirPin1 2

void move_right(uint16_t);
void move_left(uint16_t);
void move_up(uint16_t);
void move_down(uint16_t);

int main(void)
{
    DDRD = 0b01110100; 
    while (1) {
        PORTD = 0b00000000;
        for (int x = 0; x < 800; x++) {
            PORTD = 0b01100000; // Set stepPin HIGH
            _delay_us(500);
            PORTD = 0b00000000; // Set stepPin LOW
            _delay_us(500);
        }
        _delay_ms(1000);
        PORTD = 0b00010100;
        for (int x = 0; x < 800; x++) {
            PORTD = 0b01100000; // Set stepPin HIGH
            _delay_us(500);
            PORTD = 0b00000000; // Set stepPin LOW
            _delay_us(500);
        }
    }
}
void move_right(uint16_t r_steps){
//counter, counter
        PORTD = 0b00000000;
        for (int x = 0; x < r_steps; x++) {
            PORTD = 0b01100000; // Set stepPin HIGH
            _delay_us(500);
            PORTD = 0b00000000; // Set stepPin LOW
            _delay_us(500);
        }
}
void move_left(uint16_t l_steps){
//clock, clock 
       PORTD = 0b00010100;
        for (int x = 0; x < l_steps; x++) {
            PORTD = 0b01100000; // Set stepPin HIGH
            _delay_us(500);
            PORTD = 0b00000000; // Set stepPin LOW
            _delay_us(500);
        }
}

void move_up(uint16_t u_steps){
//counter, clock 
        PORTD = 0b00000100;
        for (int x = 0; x < u_steps; x++) {
            PORTD = 0b01100000; // Set stepPin HIGH
            _delay_us(500);
            PORTD = 0b00000000; // Set stepPin LOW
            _delay_us(500);
        }
}

void move_down(uint16_t d_steps){
//clock, counter 
        PORTD = 0b00010000;
        for (int x = 0; x < d_steps; x++) {
            PORTD |= (1 << PORTD6);
            PORTD |= (1 << PORTD5); // Set stepPin HIGH
            _delay_us(500);
           PORTD = 0b00000000; // Set stepPin LOW
            _delay_us(500);
        }
}
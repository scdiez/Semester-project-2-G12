#include <avr/io.h>
#include <util/delay.h>

#define stepPin 5
#define dirPin 2
#define stepPin2 6
#define dirPin2 4

void move_right(int, int);
void move_left(int, int);

int main(void)
{
    DDRD = 0b001110100; // Set dirPin and stepPin as output

    while (1) {
			PORTD = 0b00010100;
			move_right(800, 500);
			PORTD = 0b00000000;
			move_left(800, 500);
        }


    return 0;
}

void move_right(int steps, int delay){
	 //PORTD = 0b00010100; // Set dirPin HIGH to move in a particular direction
        for (int x = 0; x < steps; x++) {
            PORTD = 0b01100000;// Set stepPin HIGH
            _delay_us(delay);
            PORTD = 0b00000000;; // Set stepPin LOW
            _delay_us(delay);
        }
}
void move_left(int steps, int delay){
 	//PORTD = 0b00000000; // Set dirPin LOW to change direction of rotation
        for (int x = 0; x < steps; x++) {
            PORTD = 0b01100000;// Set stepPin HIGH
            _delay_us(delay);
            PORTD = 0b00000000;; // Set stepPin LOW
            _delay_us(delay);
        }
}
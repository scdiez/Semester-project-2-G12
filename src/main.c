#define __DELAY_BACKWARD_COMPATIBLE__ //so that delays can be set with variables

#include <avr/io.h>
#include <stdio.h>
#include <util/delay.h>
#include <stdlib.h>
#include "i2cmaster.h"
#include "lcd.h"
#include "usart.h"
#include <time.h>

void move_left(int, int);
void move_right(int, int);
void move_up (int, int);
void move_down(int, int);

volatile int delay;

int target_x,target_y, move_x, move_y;
int current_x=0;
int current_y=0;;

int main(void) {
    uart_init();
    io_redirect();

	DDRD |= (1 << DDD4) | (1 << DDD6); // Set dirPin and stepPin as output
	DDRD |= (1 << DDD2) | (1 << DDD5);

	//SPEAKER "PRESS A BUTTON TO PLAY" 
	//move hoop to 0,0
	move_down(100, 500);
	move_left(100, 500);

	//IF BUTTON 1 IS PRESSED
	//SPEAKER "You've selected no vision single player"
	//SPEAKER "The basketball hoop will now move and make a sound at its final position, try to put the ball inside it"

    srand(time(0));
    target_x = rand() % 50;
    target_y = rand() % 50;
    
	move_x = target_x - current_x;
	move_y = target_y - current_y;

	if (move_x>0){
		move_right(move_x, 400);
	}
	if (move_x<0){
		move_left(move_x, 400);
	}
	if (move_y>0){
		move_right(move_y, 400);
	}
	if (move_y<0){
		move_left(move_y, 400);
	}
	
	//SPEAKER "Beep"



}

void move_left(int steps, int delay){
	PORTD |= (1 << PORTD4); // Set dirPin HIGH to move in a particular direction
	PORTD |= (1 << PORTD2); // Set dirPin HIGH to move in a particular direction
        for (int x = 0; x < steps; x++) {
            PORTD |= (1 << PORTD6); // Set stepPin HIGH
			PORTD |= (1 << PORTD5); // Set stepPin HIGH
            _delay_us(delay);
            PORTD &= ~(1 << PORTD6); // Set stepPin LOW
			PORTD &= ~(1 << PORTD5); // Set stepPin LOW
            _delay_us(delay);
			}
}
void move_right(int steps, int delay){
	PORTD &= ~(1 << PORTD4); // Set dirPin LOW to change direction of rotation
	PORTD &= ~(1 << PORTD2);
        for (int x = 0; x < steps; x++) {
            PORTD |= (1 << PORTD6); // Set stepPin HIGH
			PORTD |= (1 << PORTD5); // Set stepPin HIGH
            _delay_us(delay);
            PORTD &= ~(1 << PORTD6); // Set stepPin LOW
			PORTD &= ~(1 << PORTD5); // Set stepPin LOW
            _delay_us(delay);
        }
}

void move_up(int steps, int delay){
	PORTD |= (1 << PORTD4); // Set dirPin HIGH to move in a particular direction
	PORTD &= ~(1 << PORTD2);
        for (int x = 0; x < steps; x++) {
            PORTD |= (1 << PORTD6); // Set stepPin HIGH
			PORTD |= (1 << PORTD5); // Set stepPin HIGH
            _delay_us(delay);
            PORTD &= ~(1 << PORTD6); // Set stepPin LOW
			PORTD &= ~(1 << PORTD5); // Set stepPin LOW
            _delay_us(delay);
        }
}

void move_down(int steps, int delay){
	PORTD |= (1 << PORTD2); // Set dirPin HIGH to move in a particular direction
	PORTD &= ~(1 << PORTD4);
        for (int x = 0; x < steps; x++) {
            PORTD |= (1 << PORTD6); // Set stepPin HIGH
			PORTD |= (1 << PORTD5); // Set stepPin HIGH
            _delay_us(delay);
            PORTD &= ~(1 << PORTD6); // Set stepPin LOW
			PORTD &= ~(1 << PORTD5); // Set stepPin LOW
            _delay_us(delay);
        }
}



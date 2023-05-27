#include <stdio.h>
#include <avr/io.h>
#include <util/delay.h>
#include <stdlib.h>
#include "i2cmaster.h"          //"" are used as these are user defined libraries
#include "lcd.h"
#include "usart.h"
#include <avr/interrupt.h>

#define F_CPU 16000000UL //DEFINE BAUDRATE AS 9600

#define trigPin 8
#define echopin 9

volatile unsigned long pulse_start;
volatile unsigned long pulse_end;
volatile unsigned long pulse_duration;
volatile int distance;
unsigned int trial_time = 0;
int counter = 0;
int sensorflag = 0;
int sensornumber;

void start_pulse(void);

void end_pulse(void);

int detect_ball (void);


//functions for motor movement
void move_left(int, int);
void move_right(int, int);
void move_up (int, int);
void move_down(int, int);
void change_position(void);
void zero(void);

//variables for motor movement 
int target_x,target_y, move_x, move_y;
int current_x=0;
int current_y=0;

int main(void)
{
    sei(); // Enable global interrupts
    uart_init();
    io_redirect();

    DDRB |= (1 << DDB0);  //Set trigPin as output
    DDRB &= ~(1 << DDB1);// Set echoPin as an input
    PORTB |= (1 << PORTB1);// Enable internal pull-up resistor for echoPin


	 //motor configuration
    DDRD |= (1 << DDD4) | (1 << DDD6); // Set dirPin1 and stepPin1 as output
	DDRD |= (1 << DDD2) | (1 << DDD5); // Set dirPin2 and stepPin2 as output
	

	for (int i = 0; i<6; i++){
		printf("%d \n", i);
		sensornumber = detect_ball();
		if (sensornumber == 3){
			move_right(1000, 500);
		}
		if (sensornumber == 2){
			move_left(1000, 500);
		}
		return 0;
	}
	
}

void start_pulse() {
    pulse_start = TCNT1; // Record the timer value at the start of the pulse
}

void end_pulse() {
    pulse_end = TCNT1; // Record the timer value at the end of the pulse
    pulse_duration = pulse_end - pulse_start; // Calculate the pulse duration
}

int detect_ball (void){
	sensorflag = 0;
    while (sensorflag == 0) {
        // Clears the trigPin
        PORTB &= ~(1 << PORTB0);
        _delay_us(2);
        PORTB |= (1 << PORTB0);// Sets the trigPin on HIGH state for 10 microseconds
        _delay_us(10);
        PORTB &= ~(1 << PORTB0);
        while ((PINB & (1 << PINB1)) == 0) {} // Wait for the falling edge on echoPin

        // Start Timer/Counter1
        TCCR1B |= (1 << CS11); // Set prescaler to 8
        start_pulse(); // Record the start time of the pulse
        while (PINB & (1 << PINB1)) {} // Wait for the rising edge on echoPin
        end_pulse(); // Record the end time of the pulse
        TCCR1B = 0; // Stop Timer/Counter1
		trial_time = trial_time + pulse_duration;
		if (trial_time >= 32000){
			counter ++;
			trial_time = 0;
		}

        distance = pulse_duration * 0.017 / 2;// Calculate the distance
        if (distance < 9.5) {
			sensorflag = 1;
			return (2); //Number of audio for "You scored a point"
        }

        if (distance > 9.5 && counter>= 120) { //edit counter value for a longer time for shooting 
            return (3); //Audio for "Try again to shoot" 
			counter = 0;
        }
    }
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

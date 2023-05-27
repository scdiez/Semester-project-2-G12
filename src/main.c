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

void start_pulse(void);

void end_pulse(void);

int detect_ball (void);

int main(void)
{
    sei(); // Enable global interrupts
    uart_init();
    io_redirect();

    DDRB |= (1 << DDB0);  //Set trigPin as output
    DDRB &= ~(1 << DDB1);// Set echoPin as an input
    PORTB |= (1 << PORTB1);// Enable internal pull-up resistor for echoPin

	printf("%d", detect_ball());
	printf("%d", detect_ball());
	printf("%d", detect_ball());
	printf("%d", detect_ball());

    return 0;
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
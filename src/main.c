#include <stdio.h>
#include <avr/io.h>
#include <util/delay.h>
#include <stdlib.h>
#include "i2cmaster.h"          //"" are used as these are user defined libraries
#include "lcd.h"
#include "usart.h"
#include <avr/interrupt.h>

#define F_CPU 16000000UL //DEFINE BAUDRATE AS 9600

#define trigPin 2
#define echopin 4

volatile unsigned long pulse_start;
volatile unsigned long pulse_end;
volatile unsigned long pulse_duration;
volatile int distance;
unsigned int trial_time = 0;
int counter;

void start_pulse() {
    pulse_start = TCNT1; // Record the timer value at the start of the pulse
}

void end_pulse() {
    pulse_end = TCNT1; // Record the timer value at the end of the pulse
    pulse_duration = pulse_end - pulse_start; // Calculate the pulse duration
}

int main(void)
{
    sei(); // Enable global interrupts
    uart_init();
    io_redirect();

    DDRD |= (1 << DDD2);
    DDRD &= ~(1 << DDD4);     // Set echoPin as an input
    PORTD |= (1 << PORTD4);     // Enable internal pull-up resistor for echoPin


    while (1) {
        PORTD &= ~(1 << PORTD2); // Clears the trigPin
        _delay_us(2);
        PORTD |= (1 << PORTD2);    // Sets the trigPin on HIGH state for 10 microseconds
        _delay_us(10);
        PORTD &= ~(1 << PORTD2);
        while ((PIND & (1 << PIND4)) == 0) {}  // Wait for the falling edge on echoPin
        // Start Timer/Counter1
        TCCR1B |= (1 << CS11); // Set prescaler to 8
        start_pulse();        // Record the start time of the pulse
        while (PIND & (1 << PIND4)) {} // Wait for the rising edge on echoPin
        end_pulse();     // Record the end time of the pulse
        TCCR1B = 0;   // Stop Timer/Counter1

		trial_time = trial_time + pulse_duration;
        distance = pulse_duration * 0.017 / 2; // Calculate the distance
		printf("%u", trial_time);

		if (trial_time>=65000){
			counter++;
			trial_time=0;
		}
        if (distance < 9.5) {
            printf("Nice shot! \n");
        }


        if (distance > 9.5 && counter>=10) {
            printf("Try Again \n");
			counter++;
        }
    }

    return 0;
}

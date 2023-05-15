#include <stdio.h>
#include <avr/io.h>
#include <util/delay.h>
#include <stdlib.h>
#include "i2cmaster.h"          //"" are used as these are user defined libraries
#include "lcd.h"
#include "usart.h"
#include <avr/interrupt.h>

#define F_CPU 16000000UL

//DEFINE BAUDRATE AS 9600

#define trigPin 2
#define echopin 4

volatile unsigned long pulse_start;
volatile unsigned long pulse_end;
volatile unsigned long pulse_duration;
volatile int distance;

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

    // Set echoPin as an input
    DDRD &= ~(1 << DDD4);

    // Enable internal pull-up resistor for echoPin
    PORTD |= (1 << PORTD4);

    while (1) {
        // Clears the trigPin
        PORTD &= ~(1 << PORTD2);
        _delay_us(2);

        // Sets the trigPin on HIGH state for 10 microseconds
        PORTD |= (1 << PORTD2);
        _delay_us(10);
        PORTD &= ~(1 << PORTD2);

        // Wait for the falling edge on echoPin
        while ((PIND & (1 << PIND4)) == 0) {}

        // Start Timer/Counter1
        TCCR1B |= (1 << CS11); // Set prescaler to 8

        // Record the start time of the pulse
        start_pulse();

        // Wait for the rising edge on echoPin
        while (PIND & (1 << PIND4)) {}

        // Record the end time of the pulse
        end_pulse();

        // Stop Timer/Counter1
        TCCR1B = 0;

        // Calculate the distance
        distance = pulse_duration * 0.034 / 2;

        // Prints the distance on the Serial Monitor
        printf("Distance: %d\n", distance);
    }

    return 0;
}
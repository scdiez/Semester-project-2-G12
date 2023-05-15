#include <stdio.h>
#include <avr/io.h>
#include <util/delay.h>
#include <stdlib.h>
#include "i2cmaster.h"          //"" are used as these are user defined libraries
#include "lcd.h"
#include "usart.h"

#define F_CPU 16000000UL

//DEFINE BAUDRATE AS 9600

#define trigPin 2
#define echopin 3

volatile float timer;
volatile float miliseconds;
volatile float distanceincm;


int main(void)
{
    uart_init();
    io_redirect();
    //sensor configuration
    DDRD |= (1 << DDD2); // Set trig as output
    DDRD &= ~(1 << DDD3); //Set echo pin as input 

    //timer configuration
    TCCR0A |= (1<<WGM01); //Timer mode to CTC
    OCR0A = 0xF9; //set value to count to
    
    PORTB |= ~(1 << PORTD2); //set pin 2 to low 

    while (1) {
        PORTD |= (1 << PORTD2); // Set Pin 2 to high (Send signal)
        PORTD |= (1 << PORTD3); //Set pin 3 to high 
        _delay_ms(10);
        PORTD |= ~(1 << PORTD2); //set pin 2 to low (stop signal)
        TCCR0A |= (1<<WGM01); //Timer mode to CTC
        OCR0A = 0xF9; //set value to count to
        TCCR0B |= (1<<CS01) | (1<<CS00); //Start the timer and prescaler to 64
        while((TIFR0 & (1<<OCF0A))==0){//wait the overflov event
        }
        //reset overflow flag 
        TIFR0 = (1<<OCF0A);
        timer ++;
        if(PORTD3 == 0){
            miliseconds = timer;
            timer = 0;
            distanceincm = 0.17*1000*miliseconds;
            printf("%d", distanceincm);
        }

    }
    return 0;
}
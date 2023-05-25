#include <avr/io.h>
#include <util/delay.h>
#include <stdio.h>
#include "usart.h"

#define stepPin 5
#define dirPin 2
#define enPin 8

int main(void)
{
    //uart_init();
    //io_redirect();
    DDRD |= (1 << DDD2) | (1 << DDD5); // Set dirpin and stepPin as output

    while (1) {
        PORTD |= (1 << PORTD2); // Set dirPin HIGH to move in a particular direction
        //printf("Clockwise \n");
        for (int x = 0; x < 1600; x++) {
            PORTD |= (1 << PORTD5); // Set stepPin HIGH
            _delay_us(500);
            PORTD &= ~(1 << PORTD5); // Set stepPin LOW
            _delay_us(500);
        }
        PORTD &= ~(1 << PORTD2); // Set dirPin LOW to change direction of rotation
        printf("Anticlockwise \n");
        for (int x = 0; x < 1600; x++) {
            PORTD |= (1 << PORTD5); // Set stepPin HIGH
            _delay_us(500);
            PORTD &= ~(1 << PORTD5); // Set stepPin LOW
            _delay_us(500);
        }
    }
    return 0;
}
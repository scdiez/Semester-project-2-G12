#define F_CPU 16000000UL


#include <avr/io.h>
#include <util/delay.h>
#include <stdio.h>
#include "usart.h"


int main(void) {    


    uart_init(); // open the communication to the microcontroller
    io_redirect(); // redirect input and output to the communication


    printf("Test");


    DDRC = 0xF0;
    PORTC = 0x3F;


    while(1)
    {
        if (PINC == 0b00111011)
        {
            printf("4");
			_delay_ms(500);
        }
        if (PINC == 0b00110111)
        {
            printf("1");
			_delay_ms(500);
        }
        if (PINC == 0b00101111)
        {
            printf("2");
			_delay_ms(500);
        }
        if (PINC == 0b00011111)
        {
            printf("3");
			_delay_ms(500);
        }
    }


       
    return 0;
}



